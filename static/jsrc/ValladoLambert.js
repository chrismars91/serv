


/**
 * amazing library 10/10 https://github.com/jorgepiloto/lamberthub/tree/main
 * Vallado Lambert Solver for Satellite Earth Model
 * All units in kilometers and seconds
 */

// Earth's gravitational parameter in km³/s²
const MU_EARTH = 398600.4418;

/**
 * Gamma function approximation using Lanczos approximation
 */
function gamma(z) {
    const g = 7;
    const C = [
        0.99999999999980993,
        676.5203681218851,
        -1259.1392167224028,
        771.32342877765313,
        -176.61502916214059,
        12.507343278686905,
        -0.13857109526572012,
        9.9843695780195716e-6,
        1.5056327351493116e-7
    ];

    if (z < 0.5) {
        return Math.PI / (Math.sin(Math.PI * z) * gamma(1 - z));
    }

    z -= 1;
    let x = C[0];
    for (let i = 1; i < g + 2; i++) {
        x += C[i] / (z + i);
    }

    const t = z + g + 0.5;
    const sqrt2pi = Math.sqrt(2 * Math.PI);

    return sqrt2pi * Math.pow(t, z + 0.5) * Math.exp(-t) * x;
}

/**
 * Third Stumpff function
 */
function c3(psi) {
    const eps = 1.0;
    
    if (psi > eps) {
        const sqrtPsi = Math.sqrt(psi);
        return (sqrtPsi - Math.sin(sqrtPsi)) / (psi * sqrtPsi);
    } else if (psi < -eps) {
        const sqrtNegPsi = Math.sqrt(-psi);
        return (Math.sinh(sqrtNegPsi) - sqrtNegPsi) / (-psi * sqrtNegPsi);
    } else {
        let res = 1.0 / 6.0;
        let delta = (-psi) / gamma(2 + 3 + 1);
        let k = 1;
        
        while (Math.abs(delta) > 1e-15 && k < 100) {
            res = res + delta;
            k += 1;
            delta = Math.pow(-psi, k) / gamma(2 * k + 3 + 1);
        }
        
        return res;
    }
}

/**
 * Second Stumpff function
 */
function c2(psi) {
    const eps = 1.0;
    
    if (psi > eps) {
        return (1 - Math.cos(Math.sqrt(psi))) / psi;
    } else if (psi < -eps) {
        return (Math.cosh(Math.sqrt(-psi)) - 1) / (-psi);
    } else {
        let res = 1.0 / 2.0;
        let delta = (-psi) / gamma(2 + 2 + 1);
        let k = 1;
        
        while (Math.abs(delta) > 1e-15 && k < 100) {
            res = res + delta;
            k += 1;
            delta = Math.pow(-psi, k) / gamma(2 * k + 2 + 1);
        }
        
        return res;
    }
}

/**
 * Vector operations
 */
class VectorL3 {
    constructor(x, y, z) {
        this.x = x;
        this.y = y;
        this.z = z;
    }

    static dot(v1, v2) {
        return v1.x * v2.x + v1.y * v2.y + v1.z * v2.z;
    }

    static cross(v1, v2) {
        return new VectorL3(
            v1.y * v2.z - v1.z * v2.y,
            v1.z * v2.x - v1.x * v2.z,
            v1.x * v2.y - v1.y * v2.x
        );
    }

    static norm(v) {
        return Math.sqrt(VectorL3.dot(v, v));
    }

    static subtract(v1, v2) {
        return new VectorL3(v1.x - v2.x, v1.y - v2.y, v1.z - v2.z);
    }

    static scale(v, scalar) {
        return new VectorL3(v.x * scalar, v.y * scalar, v.z * scalar);
    }

    static equals(v1, v2, tolerance = 1e-10) {
        return Math.abs(v1.x - v2.x) < tolerance &&
               Math.abs(v1.y - v2.y) < tolerance &&
               Math.abs(v1.z - v2.z) < tolerance;
    }

    toArray() {
        return [this.x, this.y, this.z];
    }
}

/**
 * Input validation functions
 */
function validateInputs(mu, r1, r2, tof, M) {
    if (mu <= 0) {
        throw new Error("Gravitational parameter must be positive!");
    }
    
    if (VectorL3.norm(r1) === 0 || VectorL3.norm(r2) === 0) {
        throw new Error("Position vector cannot be the null vector [0,0,0]!");
    }
    
    if (VectorL3.equals(r1, r2)) {
        throw new Error("Initial and final position vectors cannot be equal!");
    }
    
    if (tof <= 0) {
        throw new Error("Time of flight must be positive!");
    }
    
    if (M < 0) {
        throw new Error("Number of revolutions must be equal or greater than zero!");
    }
    
    return true;
}

/**
 * Get transfer angle
 */
function getTransferAngle(r1, r2, prograde) {
    const crossProduct = VectorL3.cross(r1, r2);
    const crossNorm = VectorL3.norm(crossProduct);
    
    // Check if vectors are collinear
    if (crossNorm < 1e-10) {
        // Check if same direction (0) or opposite (π)
        return VectorL3.dot(r1, r2) > 0 ? 0 : Math.PI;
    }
    
    // Unit normal vector
    const h = VectorL3.scale(crossProduct, 1.0 / crossNorm);
    
    // Projection onto reference plane (z-axis)
    const alpha = h.z;
    
    // Minimum angle between vectors
    const r1Norm = VectorL3.norm(r1);
    const r2Norm = VectorL3.norm(r2);
    const cosTheta = VectorL3.dot(r1, r2) / (r1Norm * r2Norm);
    const theta0 = Math.acos(Math.max(-1, Math.min(1, cosTheta)));
    
    // Determine transfer angle based on motion direction
    let dtheta;
    if (prograde) {
        dtheta = alpha > 0 ? theta0 : 2 * Math.PI - theta0;
    } else {
        dtheta = alpha < 0 ? theta0 : 2 * Math.PI - theta0;
    }
    
    return dtheta;
}

/**
 * Helper functions for Vallado algorithm
 */
function getA(r1Norm, r2Norm, dtheta) {
    const tm = dtheta < Math.PI ? 1 : -1;
    return tm * Math.sqrt(r1Norm * r2Norm * (1 + Math.cos(dtheta)));
}

function yAtPsi(psi, r1Norm, r2Norm, A) {
    const c2Psi = c2(psi);
    const c3Psi = c3(psi);
    return (r1Norm + r2Norm) + A * (psi * c3Psi - 1) / Math.sqrt(c2Psi);
}

function XAtPsi(psi, y) {
    return Math.sqrt(y / c2(psi));
}

function tofVallado(mu, psi, X, A, y) {
    return (Math.pow(X, 3) * c3(psi) + A * Math.sqrt(y)) / Math.sqrt(mu);
}

/**
 * Main Vallado Lambert solver
 */
function vallado2013(mu, r1, r2, tof, options = {}) {
    const {
        M = 0,
        prograde = true,
        lowPath = true,
        maxiter = 100,
        atol = 1e-5,
        rtol = 1e-7,
        fullOutput = false
    } = options;

    // Validate inputs
    validateInputs(mu, r1, r2, tof, M);

    // Get fundamental geometry
    const r1Norm = VectorL3.norm(r1);
    const r2Norm = VectorL3.norm(r2);
    const cVec = VectorL3.subtract(r2, r1);
    const cNorm = VectorL3.norm(cVec);
    const dtheta = getTransferAngle(r1, r2, prograde);

    // Compute transfer angle parameter
    const A = getA(r1Norm, r2Norm, dtheta);
    if (Math.abs(A) < 1e-10) {
        throw new Error("Cannot compute orbit, phase angle is 180 degrees");
    }

    // Initial guess and bounds for bisection
    let psi = 0.0;
    let psiLow = -4 * Math.PI * Math.PI;
    let psiUp = 4 * Math.PI * Math.PI;

    const startTime = performance.now();
    let numiter;

    for (numiter = 1; numiter <= maxiter; numiter++) {
        // Evaluate y at current psi
        let y = yAtPsi(psi, r1Norm, r2Norm, A);

        if (A > 0.0) {
            // Readjust psiLow until y > 0
            while (y < 0.0) {
                psiLow = psi;
                const c2Psi = c2(psi);
                const c3Psi = c3(psi);
                psi = 0.8 * (1.0 / c3Psi) * (1.0 - (r1Norm * r2Norm) * Math.sqrt(c2Psi) / A);
                y = yAtPsi(psi, r1Norm, r2Norm, A);
            }
        }

        const X = XAtPsi(psi, y);
        const tofNew = tofVallado(mu, psi, X, A, y);

        // Convergence check
        if (Math.abs((tofNew - tof) / tof) < rtol) {
            const endTime = performance.now();
            const tpi = (endTime - startTime) / numiter;

            // Calculate velocities
            const f = 1 - y / r1Norm;
            const g = A * Math.sqrt(y / mu);
            const gdot = 1 - y / r2Norm;

            // v1 = (r2 - f * r1) / g
            const fr1 = VectorL3.scale(r1, f);
            const r2MinusFr1 = VectorL3.subtract(r2, fr1);
            const v1 = VectorL3.scale(r2MinusFr1, 1.0 / g);

            // v2 = (gdot * r2 - r1) / g
            const gdotR2 = VectorL3.scale(r2, gdot);
            const gdotR2MinusR1 = VectorL3.subtract(gdotR2, r1);
            const v2 = VectorL3.scale(gdotR2MinusR1, 1.0 / g);

            if (fullOutput) {
                return {
                    v1: v1,
                    v2: v2,
                    numiter: numiter,
                    tpi: tpi
                };
            } else {
                return { v1: v1, v2: v2 };
            }
        }

        // Bisection step
        const condition = tofNew <= tof;
        if (condition) {
            psiLow = psiLow + (psi - psiLow);
        } else {
            psiUp = psiUp + (psi - psiUp);
        }

        psi = (psiUp + psiLow) / 2;
    }

    throw new Error("Exceeded maximum number of iterations!");
}





// const r1 = new VectorL3(0.159321004, 0.579266185, 0.05235960);      // Initial position [km]
// const r2 = new VectorL3(0.057594337, 0.605750797, 0.068345246);   // Final position [km]
// const tof = 0.010794065;       

// const result = vallado2013(39.47692641, r1, r2, tof, {
//     prograde: true,
//     fullOutput: true
// });
// console.log('Initial velocity:', result.v1.toArray(), '[km/s]');
// console.log('Final velocity:', result.v2.toArray(), '[km/s]');
// console.log('Iterations:', result.numiter);
// console.log('Time per iteration:', result.tpi.toFixed(3), 'ms');  
// const MU_EARTH = 398600.4418;