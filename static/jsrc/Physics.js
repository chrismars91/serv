class RK2Propagator {
    constructor() {
        this.GM = 398600.5;
        this.RE = 6378.0;
        this.J2 = 0.00108263;

        this.k1 = new Array(6).fill(0);
        this.k2 = new Array(6).fill(0);
        this.temp_state = new Array(6).fill(0);
    }

    derivative(state, result) {
        // Position derivative = velocity
        result[0] = state[3];
        result[1] = state[4];
        result[2] = state[5];

        const x = state[0];
        const y = state[1];
        const z = state[2];

        const r = Math.sqrt(x*x + y*y + z*z);
        const r2 = r * r;
        const r3 = r2 * r;

        const ax = -this.GM * x / r3;
        const ay = -this.GM * y / r3;
        const az = -this.GM * z / r3;

        result[3] = ax;
        result[4] = ay;
        result[5] = az;
    }

    step(state, dt) {
        // k1 = f(t, y)
        this.derivative(state, this.k1);
        for (let i = 0; i < 6; i++) {
            this.k1[i] *= dt;
            this.temp_state[i] = state[i] + 0.5 * this.k1[i];
        }

        // k2 = f(t+dt/2, y + k1/2)
        this.derivative(this.temp_state, this.k2);
        for (let i = 0; i < 6; i++) {
            this.k2[i] *= dt;
            state[i] += this.k2[i]; // y_{n+1} = y_n + k2
        }
    }
}



class RK45Propagator {
    constructor() {
        this.GM = 398600.5;
        this.RE = 6378.0;
        this.J2 = 0.00108263;
        
        // Dormand-Prince coefficients (matches Python scipy.integrate.RK45)
        this.C = [0, 1/5, 3/10, 4/5, 8/9, 1];
        
        this.A = [
            [0, 0, 0, 0, 0],
            [1/5, 0, 0, 0, 0],
            [3/40, 9/40, 0, 0, 0],
            [44/45, -56/15, 32/9, 0, 0],
            [19372/6561, -25360/2187, 64448/6561, -212/729, 0],
            [9017/3168, -355/33, 46732/5247, 49/176, -5103/18656]
        ];
        
        this.B = [35/384, 0, 500/1113, 125/192, -2187/6784, 11/84];
        this.E = [-71/57600, 0, 71/16695, -71/1920, 17253/339200, -22/525, 1/40];
        
        // Working arrays to avoid repeated allocations
        this.k = [
            [0, 0, 0, 0, 0, 0],  // k1
            [0, 0, 0, 0, 0, 0],  // k2
            [0, 0, 0, 0, 0, 0],  // k3
            [0, 0, 0, 0, 0, 0],  // k4
            [0, 0, 0, 0, 0, 0],  // k5
            [0, 0, 0, 0, 0, 0]   // k6
        ];
        this.temp_state = [0, 0, 0, 0, 0, 0];
    }

    derivative(state, result) {


        /*
        const norm_r = Math.sqrt(x*x + y*y + z*z);
        const r_2 = norm_r * norm_r;
        const norm_r_3 = r_2 * norm_r;
        const norm_r_4 = r_2 * r_2;
        const ax = -x * this.GM / norm_r_3;
        const ay = -y * this.GM / norm_r_3;
        const az = -z * this.GM / norm_r_3;
        // const z1_2 = z * z;  // r[2] ** 2
        // const jx = x / norm_r * (5 * z1_2 / r_2 - 1);
        // const jy = y / norm_r * (5 * z1_2 / r_2 - 1);
        // const jz = z / norm_r * (5 * z1_2 / r_2 - 3);
        // const j2_coeff = 1.5 * this.J2 * this.GM * (this.RE * this.RE) / norm_r_4;
        // ax += j2_coeff * jx;
        // ay += j2_coeff * jy;
        // az += j2_coeff * jz;
        */

        // Position derivative is velocity
        result[0] = state[3];  // dx/dt = vx
        result[1] = state[4];  // dy/dt = vy
        result[2] = state[5];  // dz/dt = vz

        const x = state[0];
        const y = state[1];
        const z = state[2];

        const norm_r = Math.sqrt(x*x + y*y + z*z);
        const r_2 = norm_r * norm_r;
        const norm_r_3 = r_2 * norm_r;
        const norm_r_4 = r_2 * r_2;
        
        const ax = -x * this.GM / norm_r_3;
        const ay = -y * this.GM / norm_r_3;
        const az = -z * this.GM / norm_r_3;


        result[3] = ax;  // dvx/dt = ax
        result[4] = ay;  // dvy/dt = ay
        result[5] = az;  // dvz/dt = az
    }

    step(state, dt) {

        // k1
        this.derivative(state, this.k[0]);
        for (let i = 0; i < 6; i++) {this.k[0][i] *= dt;}

        // k2 through k6
        for (let stage = 1; stage < 6; stage++) {
            for (let i = 0; i < 6; i++) {
                this.temp_state[i] = state[i];
                for (let j = 0; j < stage; j++) {this.temp_state[i] += this.A[stage][j] * this.k[j][i];}
            }
            
            this.derivative(this.temp_state, this.k[stage]);
            for (let i = 0; i < 6; i++) {this.k[stage][i] *= dt;}
        }

        // Update 5th order
        for (let i = 0; i < 6; i++) {
            for (let j = 0; j < 6; j++) {state[i] += this.B[j] * this.k[j][i];}
        }
    }
}



function elementsToState(a, e, i, omega, Omega, nu, GM = 398600.4418) {
    const p = a * (1 - e * e);
    const r_mag = p / (1 + e * Math.cos(nu));
    const r_pqw = [
        r_mag * Math.cos(nu),
        r_mag * Math.sin(nu),
        0
    ];
    const v_pqw = [
        -Math.sqrt(GM/p) * Math.sin(nu),
        Math.sqrt(GM/p) * (e + Math.cos(nu)),
        0
    ];

    const cos_Omega = Math.cos(Omega);
    const sin_Omega = Math.sin(Omega);
    const cos_i = Math.cos(i);
    const sin_i = Math.sin(i);
    const cos_omega = Math.cos(omega);
    const sin_omega = Math.sin(omega);

    const R11 = cos_Omega * cos_omega - sin_Omega * sin_omega * cos_i;
    const R12 = -cos_Omega * sin_omega - sin_Omega * cos_omega * cos_i;
    const R13 = sin_Omega * sin_i;
    const R21 = sin_Omega * cos_omega + cos_Omega * sin_omega * cos_i;
    const R22 = -sin_Omega * sin_omega + cos_Omega * cos_omega * cos_i;
    const R23 = -cos_Omega * sin_i;
    const R31 = sin_omega * sin_i;
    const R32 = cos_omega * sin_i;
    const R33 = cos_i;

    const r_ijk = [
        R11 * r_pqw[0] + R12 * r_pqw[1] + R13 * r_pqw[2],
        R21 * r_pqw[0] + R22 * r_pqw[1] + R23 * r_pqw[2],
        R31 * r_pqw[0] + R32 * r_pqw[1] + R33 * r_pqw[2]
    ];

    const v_ijk = [
        R11 * v_pqw[0] + R12 * v_pqw[1] + R13 * v_pqw[2],
        R21 * v_pqw[0] + R22 * v_pqw[1] + R23 * v_pqw[2],
        R31 * v_pqw[0] + R32 * v_pqw[1] + R33 * v_pqw[2]
    ];

    return [r_ijk[0], r_ijk[2], -r_ijk[1], v_ijk[0], v_ijk[2], -v_ijk[1]];
}



class UmbraChecker {
    constructor() {
        this.R_EARTH = 0.6371;
        this._sunDir = new Vector3();
        this._projVec = new Vector3();
        this._perp = new Vector3();
        this.r_object = new Vector3();
        this.r_sun = new Vector3(); 
    }
    inUmbra(r_object, r_sun) {
        this._sunDir.copy(r_sun).normalize();
        const proj = r_object.dot(this._sunDir);
        if (proj <= 0) return false;
        this._projVec.copy(this._sunDir).multiplyScalar(proj);
        this._perp.copy(r_object).sub(this._projVec);
        const perpDist = this._perp.length();
        return perpDist < this.R_EARTH;
    }
}



function leastSquaresPosError(x1, y1, z1, x2, y2, z2) {
    return Math.pow(Math.pow(x1-x2,2)+Math.pow(y1-y2,2)+Math.pow(z1-z2,2),.5);
}

