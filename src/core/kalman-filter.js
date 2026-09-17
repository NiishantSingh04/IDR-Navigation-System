/**
 * Extended Kalman Filter (EKF) for Error-State INS
 * Implements the error-state formulation from Mikov et al.
 * State vector: [dN, dE, dVn, dVe, dPsi, bAx, bAy, bGz]
 *   dN, dE     - position errors (meters)
 *   dVn, dVe   - velocity errors (m/s)
 *   dPsi       - heading error (rad)
 *   bAx, bAy   - accelerometer bias (m/s^2)
 *   bGz        - gyroscope bias (rad/s)
 */

const STATE_DIM = 8;

/**
 * Create a zero-initialized NxN matrix
 */
function zeros(n) {
  return Array.from({ length: n }, () => new Float64Array(n));
}

/**
 * Create identity NxN matrix
 */
function eye(n) {
  const M = zeros(n);
  for (let i = 0; i < n; i++) M[i][i] = 1;
  return M;
}

/**
 * Deep copy a matrix
 */
function copyMatrix(M) {
  return M.map(row => Float64Array.from(row));
}

/**
 * Matrix multiply A (mxn) * B (nxp) => C (mxp)
 */
function matMul(A, B) {
  const m = A.length, n = B.length, p = B[0].length;
  const C = Array.from({ length: m }, () => new Float64Array(p));
  for (let i = 0; i < m; i++) {
    for (let j = 0; j < p; j++) {
      let sum = 0;
      for (let k = 0; k < n; k++) sum += A[i][k] * B[k][j];
      C[i][j] = sum;
    }
  }
  return C;
}

/**
 * Matrix transpose
 */
function matT(M) {
  const m = M.length, n = M[0].length;
  const T = Array.from({ length: n }, () => new Float64Array(m));
  for (let i = 0; i < m; i++)
    for (let j = 0; j < n; j++)
      T[j][i] = M[i][j];
  return T;
}

/**
 * Matrix addition
 */
function matAdd(A, B) {
  return A.map((row, i) => Float64Array.from(row.map((v, j) => v + B[i][j])));
}

/**
 * Matrix subtraction
 */
function matSub(A, B) {
  return A.map((row, i) => Float64Array.from(row.map((v, j) => v - B[i][j])));
}

/**
 * Matrix * vector
 */
function matVec(M, v) {
  return M.map(row => row.reduce((s, val, j) => s + val * v[j], 0));
}

/**
 * Invert a small matrix (up to 8x8) using Gauss-Jordan
 */
function matInv(M) {
  const n = M.length;
  const aug = M.map((row, i) => {
    const r = new Float64Array(2 * n);
    for (let j = 0; j < n; j++) r[j] = row[j];
    r[n + i] = 1;
    return r;
  });

  for (let i = 0; i < n; i++) {
    // Find pivot
    let maxVal = Math.abs(aug[i][i]), maxRow = i;
    for (let k = i + 1; k < n; k++) {
      if (Math.abs(aug[k][i]) > maxVal) {
        maxVal = Math.abs(aug[k][i]);
        maxRow = k;
      }
    }
    [aug[i], aug[maxRow]] = [aug[maxRow], aug[i]];

    const pivot = aug[i][i];
    if (Math.abs(pivot) < 1e-20) {
      aug[i][i] = 1e-10; // regularize
    }
    const invPivot = 1 / aug[i][i];
    for (let j = 0; j < 2 * n; j++) aug[i][j] *= invPivot;

    for (let k = 0; k < n; k++) {
      if (k === i) continue;
      const factor = aug[k][i];
      for (let j = 0; j < 2 * n; j++) aug[k][j] -= factor * aug[i][j];
    }
  }

  return aug.map(row => Float64Array.from(row.slice(n)));
}

/**
 * Extended Kalman Filter for INS error states
 */
export class ErrorStateEKF {
  constructor(config = {}) {
    this.n = STATE_DIM;

    // Error state vector
    this.x = new Float64Array(this.n);

    // Covariance matrix P
    this.P = eye(this.n);
    // Initial uncertainties
    this.P[0][0] = config.posVar || 100;      // dN
    this.P[1][1] = config.posVar || 100;      // dE
    this.P[2][2] = config.velVar || 25;       // dVn
    this.P[3][3] = config.velVar || 25;       // dVe
    this.P[4][4] = config.headingVar || 0.1;  // dPsi
    this.P[5][5] = config.accelBiasVar || 0.01; // bAx
    this.P[6][6] = config.accelBiasVar || 0.01; // bAy
    this.P[7][7] = config.gyroBiasVar || 0.001; // bGz

    // Process noise
    this.Q = eye(this.n);
    this.Q[0][0] = config.posProcessNoise || 0.5;
    this.Q[1][1] = config.posProcessNoise || 0.5;
    this.Q[2][2] = config.velProcessNoise || 0.2;
    this.Q[3][3] = config.velProcessNoise || 0.2;
    this.Q[4][4] = config.headingProcessNoise || 0.01;
    this.Q[5][5] = config.accelBiasProcessNoise || 0.001;
    this.Q[6][6] = config.accelBiasProcessNoise || 0.001;
    this.Q[7][7] = config.gyroBiasProcessNoise || 0.0001;

    // GNSS measurement noise
    this.R_gnss = zeros(2);
    this.R_gnss[0][0] = config.gnssNoise || 9;  // ~3m std
    this.R_gnss[1][1] = config.gnssNoise || 9;

    // Speed measurement noise
    this.R_speed = zeros(2);
    this.R_speed[0][0] = config.speedNoise || 1;
    this.R_speed[1][1] = config.speedNoise || 1;
  }

  /**
   * Prediction step: propagate error state
   * @param {number} dt - time step in seconds
   * @param {number} speed - estimated speed in m/s
   * @param {number} heading - heading in radians
   */
  predict(dt, speed, heading) {
    // State transition matrix
    const F = eye(this.n);

    // Position errors grow with velocity errors
    F[0][2] = dt; // dN += dVn * dt
    F[1][3] = dt; // dE += dVe * dt

    // Position affected by heading error
    F[0][4] = -speed * Math.sin(heading) * dt;
    F[1][4] = speed * Math.cos(heading) * dt;

    // Velocity affected by accel bias
    F[2][5] = dt * Math.cos(heading);
    F[2][6] = -dt * Math.sin(heading);
    F[3][5] = dt * Math.sin(heading);
    F[3][6] = dt * Math.cos(heading);

    // Heading affected by gyro bias
    F[4][7] = dt;

    // Propagate state
    this.x = matVec(F, this.x);

    // Propagate covariance: P = F * P * F' + Q * dt
    const FP = matMul(F, this.P);
    const FPFt = matMul(FP, matT(F));
    const Qdt = this.Q.map(row => Float64Array.from(row.map(v => v * dt)));
    this.P = matAdd(FPFt, Qdt);
  }

  /**
   * GNSS position measurement update
   * @param {number} dN - position innovation north (meters)
   * @param {number} dE - position innovation east (meters)
   * @returns {{dN: number, dE: number, dPsi: number}} corrections
   */
  updateGNSS(dN, dE) {
    const H = zeros(2);
    // Measurement: z = [dN, dE] -> maps to state [0, 1]
    H[0] = new Float64Array(this.n);
    H[1] = new Float64Array(this.n);
    H[0][0] = 1;
    H[1][1] = 1;

    const z = new Float64Array([dN, dE]);
    const Hx = matVec(H, this.x);
    const innovation = z.map((v, i) => v - Hx[i]);

    return this._update(H, this.R_gnss, innovation);
  }

  /**
   * Speed / NHC measurement update
   * @param {number} dVn - velocity innovation north
   * @param {number} dVe - velocity innovation east
   */
  updateSpeed(dVn, dVe) {
    const H = Array.from({ length: 2 }, () => new Float64Array(this.n));
    H[0][2] = 1;
    H[1][3] = 1;

    const z = new Float64Array([dVn, dVe]);
    const Hx = matVec(H, this.x);
    const innovation = z.map((v, i) => v - Hx[i]);

    return this._update(H, this.R_speed, innovation);
  }

  /**
   * Generic KF update step
   */
  _update(H, R, innovation) {
    const Ht = matT(H);
    const PHt = matMul(this.P, Ht);
    const S = matAdd(matMul(H, PHt), R);
    const Sinv = matInv(S);
    const K = matMul(PHt, Sinv);

    // Update state: x = x + K * innovation
    const Kinv = matVec(K, Array.from(innovation));
    this.x = Float64Array.from(this.x.map((v, i) => v + Kinv[i]));

    // Update covariance: P = (I - K*H) * P
    const KH = matMul(K, H);
    const IKH = eye(this.n).map((row, i) =>
      Float64Array.from(row.map((v, j) => v - KH[i][j]))
    );
    this.P = matMul(IKH, this.P);

    return {
      dN: this.x[0],
      dE: this.x[1],
      dVn: this.x[2],
      dVe: this.x[3],
      dPsi: this.x[4],
      bAx: this.x[5],
      bAy: this.x[6],
      bGz: this.x[7]
    };
  }

  /**
   * Get the position uncertainty (1-sigma) in meters
   */
  getPositionUncertainty() {
    return {
      north: Math.sqrt(Math.abs(this.P[0][0])),
      east: Math.sqrt(Math.abs(this.P[1][1])),
      horizontal: Math.sqrt(Math.abs(this.P[0][0]) + Math.abs(this.P[1][1]))
    };
  }

  /**
   * Reset the filter state
   */
  reset() {
    this.x = new Float64Array(this.n);
  }

  /**
   * Clone the filter
   */
  clone() {
    const ekf = new ErrorStateEKF();
    ekf.x = Float64Array.from(this.x);
    ekf.P = copyMatrix(this.P);
    ekf.Q = copyMatrix(this.Q);
    ekf.R_gnss = copyMatrix(this.R_gnss);
    ekf.R_speed = copyMatrix(this.R_speed);
    return ekf;
  }
}
