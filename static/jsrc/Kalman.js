function gaussianRandom(mean=0, stdev=1) {
    const u = 1 - Math.random(); // Converting [0,1) to (0,1]
    const v = Math.random();
    const z = Math.sqrt( -2.0 * Math.log( u ) ) * Math.cos( 2.0 * Math.PI * v );
    return z * stdev + mean;
}


class LinearKalmanFilter {
    constructor(sensor_noise_std, processes_model_std, dt = 0.001) {
        this.x = new Float64Array(6);
        this.P = new Float64Array(36);
        this.P[0] = 1*100000;    // P[0,0]
        this.P[7] = 1*100000;    // P[1,1] 
        this.P[14] = 1*100000;   // P[2,2]
        this.P[21] = 0.1*100000; // P[3,3]
        this.P[28] = 0.1*100000; // P[4,4]
        this.P[35] = 0.1*100000; // P[5,5]
        this.Q = new Float64Array(36);
        const q_var = processes_model_std * processes_model_std;
        this.Q[0] = this.Q[7] = this.Q[14] = q_var;
        this.Q[21] = this.Q[28] = this.Q[35] = q_var;
        this.R = new Float64Array(9);
        const r_var = sensor_noise_std * sensor_noise_std;
        this.R[0] = this.R[4] = this.R[8] = r_var;
        
        this.F = new Float64Array(36);
        this.F[0] = this.F[7] = this.F[14] = 1;
        this.F[21] = this.F[28] = this.F[35] = 1;
        this.F[3] = this.F[10] = this.F[17] = dt;

        this.H = new Float64Array(18);
        this.H[0] = this.H[7] = this.H[14] = 1;
        
        // Working matrices
        this.temp6x6 = new Float64Array(36);
        this.temp6x3 = new Float64Array(18);
        this.temp3x3 = new Float64Array(9);
        this.temp3x6 = new Float64Array(18);
        this.temp6x1 = new Float64Array(6);
        this.temp3x1 = new Float64Array(3);
        
        this.K = new Float64Array(18);
        
        this.y = new Float64Array(3);
        
        this.S = new Float64Array(9);
        
        this.I = new Float64Array(36);
        this.I[0] = this.I[7] = this.I[14] = this.I[21] = this.I[28] = this.I[35] = 1;
        
        this.dt = dt;
    }
    
    predict(dt) {
        this.F[3] = this.F[10] = this.F[17] = dt;
        this.matrixVectorMultiply6x6(this.F, this.x, this.temp6x1);
        this.x.set(this.temp6x1);
        this.matrixMultiply6x6(this.F, this.P, this.temp6x6);
        this.matrixMultiplyTranspose6x6(this.temp6x6, this.F, this.P);
        this.matrixAdd6x6(this.P, this.Q, this.P);
    }
    
    update(z1, z2, z3) {
        this.matrixVectorMultiply3x6(this.H, this.x, this.temp3x1);
        this.y[0] = z1 - this.temp3x1[0];
        this.y[1] = z2 - this.temp3x1[1];
        this.y[2] = z3 - this.temp3x1[2];
        
        this.matrixMultiply3x6_6x6(this.H, this.P, this.temp3x6);
        this.matrixMultiplyTranspose3x6_3x6(this.temp3x6, this.H, this.S);
        this.matrixAdd3x3(this.S, this.R, this.S);
        
        this.matrixMultiplyTranspose6x6_3x6(this.P, this.H, this.temp6x3);
        this.invertMatrix3x3(this.S, this.temp3x3);
        this.matrixMultiply6x3_3x3(this.temp6x3, this.temp3x3, this.K);
        
        this.matrixVectorMultiply6x3(this.K, this.y, this.temp6x1);
        this.vectorAdd6(this.x, this.temp6x1, this.x);
        
        this.matrixMultiply6x3_3x6(this.K, this.H, this.temp6x6);
        this.matrixSubtract6x6(this.I, this.temp6x6, this.temp6x6); // I - K*H
        
        const temp6x6_2 = new Float64Array(36);
        this.matrixMultiply6x6(this.temp6x6, this.P, temp6x6_2);
        this.matrixMultiplyTranspose6x6(temp6x6_2, this.temp6x6, this.P);
        
        this.matrixMultiply6x3_3x3(this.K, this.R, this.temp6x3);
        this.matrixMultiplyTranspose6x3(this.temp6x3, this.K, this.temp6x6);
        this.matrixAdd6x6(this.P, this.temp6x6, this.P);
    }
    
    getPrediction(dt) {
        return [
            this.x[0] + this.x[3] * dt,
            this.x[1] + this.x[4] * dt,
            this.x[2] + this.x[5] * dt
        ];
    }
    
    matrixVectorMultiply6x6(A, v, result) {
        for (let i = 0; i < 6; i++) {
            result[i] = 0;
            for (let j = 0; j < 6; j++) {
                result[i] += A[i * 6 + j] * v[j];
            }
        }
    }
    
    matrixVectorMultiply3x6(A, v, result) {
        for (let i = 0; i < 3; i++) {
            result[i] = 0;
            for (let j = 0; j < 6; j++) {
                result[i] += A[i * 6 + j] * v[j];
            }
        }
    }
    
    matrixVectorMultiply6x3(A, v, result) {
        for (let i = 0; i < 6; i++) {
            result[i] = 0;
            for (let j = 0; j < 3; j++) {
                result[i] += A[i * 3 + j] * v[j];
            }
        }
    }
    
    matrixMultiply6x6(A, B, result) {
        for (let i = 0; i < 6; i++) {
            for (let j = 0; j < 6; j++) {
                result[i * 6 + j] = 0;
                for (let k = 0; k < 6; k++) {
                    result[i * 6 + j] += A[i * 6 + k] * B[k * 6 + j];
                }
            }
        }
    }
    
    matrixMultiplyTranspose6x6(A, B, result) {
        for (let i = 0; i < 6; i++) {
            for (let j = 0; j < 6; j++) {
                result[i * 6 + j] = 0;
                for (let k = 0; k < 6; k++) {
                    result[i * 6 + j] += A[i * 6 + k] * B[j * 6 + k];
                }
            }
        }
    }
    
    matrixMultiply3x6_6x6(A, B, result) {
        for (let i = 0; i < 3; i++) {
            for (let j = 0; j < 6; j++) {
                result[i * 6 + j] = 0;
                for (let k = 0; k < 6; k++) {
                    result[i * 6 + j] += A[i * 6 + k] * B[k * 6 + j];
                }
            }
        }
    }
    
    matrixMultiplyTranspose3x6_3x6(A, B, result) {
        for (let i = 0; i < 3; i++) {
            for (let j = 0; j < 3; j++) {
                result[i * 3 + j] = 0;
                for (let k = 0; k < 6; k++) {
                    result[i * 3 + j] += A[i * 6 + k] * B[j * 6 + k];
                }
            }
        }
    }
    
    matrixMultiplyTranspose6x6_3x6(A, B, result) {
        for (let i = 0; i < 6; i++) {
            for (let j = 0; j < 3; j++) {
                result[i * 3 + j] = 0;
                for (let k = 0; k < 6; k++) {
                    result[i * 3 + j] += A[i * 6 + k] * B[j * 6 + k];
                }
            }
        }
    }
    
    matrixMultiply6x3_3x3(A, B, result) {
        for (let i = 0; i < 6; i++) {
            for (let j = 0; j < 3; j++) {
                result[i * 3 + j] = 0;
                for (let k = 0; k < 3; k++) {
                    result[i * 3 + j] += A[i * 3 + k] * B[k * 3 + j];
                }
            }
        }
    }
    
    matrixMultiply6x3_3x6(A, B, result) {
        for (let i = 0; i < 6; i++) {
            for (let j = 0; j < 6; j++) {
                result[i * 6 + j] = 0;
                for (let k = 0; k < 3; k++) {
                    result[i * 6 + j] += A[i * 3 + k] * B[k * 6 + j];
                }
            }
        }
    }
    
    matrixMultiplyTranspose6x3(A, B, result) {
        for (let i = 0; i < 6; i++) {
            for (let j = 0; j < 6; j++) {
                result[i * 6 + j] = 0;
                for (let k = 0; k < 3; k++) {
                    result[i * 6 + j] += A[i * 3 + k] * B[j * 3 + k];
                }
            }
        }
    }
    
    matrixAdd6x6(A, B, result) {
        for (let i = 0; i < 36; i++) {
            result[i] = A[i] + B[i];
        }
    }
    
    matrixAdd3x3(A, B, result) {
        for (let i = 0; i < 9; i++) {
            result[i] = A[i] + B[i];
        }
    }
    
    matrixSubtract6x6(A, B, result) {
        for (let i = 0; i < 36; i++) {
            result[i] = A[i] - B[i];
        }
    }
    
    vectorAdd6(a, b, result) {
        for (let i = 0; i < 6; i++) {
            result[i] = a[i] + b[i];
        }
    }
    
    invertMatrix3x3(A, result) {

        const a11 = A[0], a12 = A[1], a13 = A[2];
        const a21 = A[3], a22 = A[4], a23 = A[5];
        const a31 = A[6], a32 = A[7], a33 = A[8];
        
        const det = a11*(a22*a33 - a23*a32) - a12*(a21*a33 - a23*a31) + a13*(a21*a32 - a22*a31);
        
        if (Math.abs(det) < 1e-12) {
            throw new Error("Matrix is singular and cannot be inverted");
        }
        
        const invDet = 1.0 / det;
        
        result[0] = (a22*a33 - a23*a32) * invDet;
        result[1] = (a13*a32 - a12*a33) * invDet;
        result[2] = (a12*a23 - a13*a22) * invDet;
        result[3] = (a23*a31 - a21*a33) * invDet;
        result[4] = (a11*a33 - a13*a31) * invDet;
        result[5] = (a13*a21 - a11*a23) * invDet;
        result[6] = (a21*a32 - a22*a31) * invDet;
        result[7] = (a12*a31 - a11*a32) * invDet;
        result[8] = (a11*a22 - a12*a21) * invDet;
    }
    
    changeProcessesUncertainty(processes_model_std) {
        const q_var = processes_model_std * processes_model_std;
        this.Q[0] = this.Q[7] = this.Q[14] = q_var;
        this.Q[21] = this.Q[28] = this.Q[35] = q_var;
    }
    
    changeSensorUncertainty(sensor_noise_std) {
        const r_var = sensor_noise_std * sensor_noise_std;
        this.R[0] = this.R[4] = this.R[8] = r_var;
    }
    
    getTraceP() {
        return this.P[0] + this.P[7] + this.P[14] + this.P[21] + this.P[28] + this.P[35];
    }
    
    getState() {
        return Array.from(this.x);
    }
    
    getCovariance() {
        return Array.from(this.P);
    }
}


// class LinearKalmanFilter {
//     constructor(sensor_noise_std, processes_model_std, dt = 1.0) {
//         this.x = new Float64Array(6);
//         this.P = new Float64Array(36);
//         this.P[0] = 1; this.P[7] = 1; this.P[14] = 1;
//         this.P[21] = 0.001; this.P[28] = 0.001; this.P[35] = 0.001;
//         this.pvar = processes_model_std * processes_model_std;
//         this.rvar = sensor_noise_std * sensor_noise_std;
//         this.tempP = new Float64Array(36);
//         this.tempP2 = new Float64Array(36);
//         this.estimation_variance = new Float64Array(3);
//         this.predictions = new Float64Array(3);
//         this.estimates = new Float64Array(3);
//         this.dt = dt;
//     }

//     predict(dt) {
//         // Update state: x = F * x
//         this.x[0] += this.x[3] * dt;
//         this.x[1] += this.x[4] * dt;
//         this.x[2] += this.x[5] * dt;
//         this.updateCovariancePredict(dt);
//         this.predictions[0] = this.x[0];
//         this.predictions[1] = this.x[1];
//         this.predictions[2] = this.x[2];
//         this.estimation_variance[0] = this.P[0];   // P[0,0]
//         this.estimation_variance[1] = this.P[7];   // P[1,1]
//         this.estimation_variance[2] = this.P[14];  // P[2,2]
//     }

//     update(z1, z2, z3) {
//         const y0 = z1 - this.x[0];
//         const y1 = z2 - this.x[1];
//         const y2 = z3 - this.x[2];
//         const s0_inv = 1.0 / (this.P[0] + this.rvar);   // 1/(P[0,0] + R[0,0])
//         const s1_inv = 1.0 / (this.P[7] + this.rvar);   // 1/(P[1,1] + R[1,1])
//         const s2_inv = 1.0 / (this.P[14] + this.rvar);  // 1/(P[2,2] + R[2,2])
//         const k00 = this.P[0] * s0_inv;    // K[0,0]
//         const k11 = this.P[7] * s1_inv;    // K[1,1]
//         const k22 = this.P[14] * s2_inv;   // K[2,2]
//         const k30 = this.P[18] * s0_inv;   // K[3,0] = P[3,0] * s0_inv
//         const k41 = this.P[25] * s1_inv;   // K[4,1] = P[4,1] * s1_inv
//         const k52 = this.P[32] * s2_inv;   // K[5,2] = P[5,2] * s2_inv
//         this.x[0] += k00 * y0;
//         this.x[1] += k11 * y1;
//         this.x[2] += k22 * y2;
//         this.x[3] += k30 * y0;
//         this.x[4] += k41 * y1;
//         this.x[5] += k52 * y2;
//         this.updateCovarianceUpdate(k00, k11, k22, k30, k41, k52);
//         this.estimates[0] = this.x[0];
//         this.estimates[1] = this.x[1];
//         this.estimates[2] = this.x[2];
//     }

//     updateCovariancePredict(dt) {
//         const dt_sq = dt * dt;
//         const p00 = this.P[0], p11 = this.P[7], p14 = this.P[14];
//         const p03 = this.P[3], p14_orig = this.P[25], p25 = this.P[32];
//         const p33 = this.P[21], p44 = this.P[28], p55 = this.P[35];
//         this.P[0] = p00 + 2 * dt * p03 + dt_sq * p33 + this.pvar;  // P[0,0]
//         this.P[7] = p11 + 2 * dt * p14_orig + dt_sq * p44 + this.pvar;  // P[1,1]
//         this.P[14] = p14 + 2 * dt * p25 + dt_sq * p55 + this.pvar;  // P[2,2]
//         this.P[21] = p33 + this.pvar;  // P[3,3]
//         this.P[28] = p44 + this.pvar;  // P[4,4]
//         this.P[35] = p55 + this.pvar;  // P[5,5]
//         this.P[3] = p03 + dt * p33;    // P[0,3]
//         this.P[18] = p03 + dt * p33;   // P[3,0]
//         this.P[25] = p14_orig + dt * p44;   // P[4,1]
//         this.P[31] = p14_orig + dt * p44;   // P[1,4]
//         this.P[32] = p25 + dt * p55;   // P[5,2]
//         this.P[38] = p25 + dt * p55;   // P[2,5]
//     }

//     updateCovarianceUpdate(k00, k11, k22, k30, k41, k52) {
//         const ikh00 = 1 - k00;
//         const ikh11 = 1 - k11;
//         const ikh22 = 1 - k22;
//         const p00 = this.P[0], p11 = this.P[7], p14 = this.P[14];
//         const p03 = this.P[3], p14_orig = this.P[25], p25 = this.P[32];
//         const p33 = this.P[21], p44 = this.P[28], p55 = this.P[35];
//         this.P[0] = ikh00 * ikh00 * p00 + k00 * k00 * this.rvar;
//         this.P[7] = ikh11 * ikh11 * p11 + k11 * k11 * this.rvar;
//         this.P[14] = ikh22 * ikh22 * p14 + k22 * k22 * this.rvar;
//         this.P[21] = p33;
//         this.P[28] = p44; 
//         this.P[35] = p55;
//         this.P[3] = ikh00 * (-k30 * p00 + p03);
//         this.P[18] = this.P[3]; // Symmetric
//         this.P[25] = ikh11 * (-k41 * p11 + p14_orig);
//         this.P[31] = this.P[25]; // Symmetric
//         this.P[32] = ikh22 * (-k52 * p14 + p25);
//         this.P[38] = this.P[32]; // Symmetric
//     }

//     changeProcessesUncertainty(processes_model_std) {
//         this.pvar = processes_model_std * processes_model_std;
//     }

//     changeSensorUncertainty(sensor_noise_std) {
//         this.rvar = sensor_noise_std * sensor_noise_std;
//     }

//     get_prediction(dt) {
//         return [
//             this.x[0] + this.x[3] * dt,
//             this.x[1] + this.x[4] * dt,
//             this.x[2] + this.x[5] * dt
//         ];
//     }


//     getTraceP() {
//         return this.P[0] + this.P[7] + this.P[14] + this.P[21] + this.P[28] + this.P[35];
//     }


// }