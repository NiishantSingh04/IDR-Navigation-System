/**
 * Signal Processing Utilities
 * Low-pass filtering, bias estimation, and vibration detection
 */

/**
 * Simple moving average filter
 * @param {number[]} data - input signal
 * @param {number} windowSize - window size
 * @returns {number[]} filtered signal
 */
export function movingAverage(data, windowSize) {
  const result = new Array(data.length);
  const half = Math.floor(windowSize / 2);

  for (let i = 0; i < data.length; i++) {
    let sum = 0, count = 0;
    for (let j = Math.max(0, i - half); j <= Math.min(data.length - 1, i + half); j++) {
      sum += data[j];
      count++;
    }
    result[i] = sum / count;
  }
  return result;
}

/**
 * First-order low-pass filter (exponential moving average)
 * @param {number[]} data - input signal
 * @param {number} alpha - smoothing factor (0-1, lower = more smoothing)
 * @returns {number[]} filtered signal
 */
export function lowPassFilter(data, alpha = 0.1) {
  const result = new Array(data.length);
  result[0] = data[0];
  for (let i = 1; i < data.length; i++) {
    result[i] = alpha * data[i] + (1 - alpha) * result[i - 1];
  }
  return result;
}

/**
 * Complementary filter for attitude estimation
 * Combines gyroscope (short-term accurate) and accelerometer (long-term stable)
 * @param {number} gyroAngle - angle from gyroscope integration
 * @param {number} accelAngle - angle from accelerometer
 * @param {number} alpha - gyroscope weight (0.95-0.99 typical)
 * @returns {number} filtered angle
 */
export function complementaryFilter(gyroAngle, accelAngle, alpha = 0.98) {
  return alpha * gyroAngle + (1 - alpha) * accelAngle;
}

/**
 * Estimate sensor bias from stationary data
 * @param {number[]} data - sensor readings during stationary period
 * @returns {{mean: number, std: number}}
 */
export function estimateBias(data) {
  const mean = data.reduce((a, b) => a + b, 0) / data.length;
  const variance = data.reduce((a, b) => a + (b - mean) ** 2, 0) / data.length;
  return { mean, std: Math.sqrt(variance) };
}

/**
 * Detect high-frequency vibrations (engine harmonics, road bumps)
 * @param {number[]} data - accelerometer data
 * @param {number} threshold - vibration threshold in m/s^2
 * @returns {boolean[]} true where vibration detected
 */
export function detectVibrations(data, threshold = 3.0) {
  const filtered = lowPassFilter(data, 0.3);
  return data.map((v, i) => Math.abs(v - filtered[i]) > threshold);
}

/**
 * Compute magnitude of 3D vector
 */
export function magnitude3D(x, y, z) {
  return Math.sqrt(x * x + y * y + z * z);
}

/**
 * Remove gravity component from accelerometer readings
 * @param {number} ax - accel X
 * @param {number} ay - accel Y
 * @param {number} az - accel Z
 * @param {number} gx - gravity X
 * @param {number} gy - gravity Y
 * @param {number} gz - gravity Z
 * @returns {{x: number, y: number, z: number}} linear acceleration
 */
export function removeGravity(ax, ay, az, gx, gy, gz) {
  return {
    x: ax - gx,
    y: ay - gy,
    z: az - gz
  };
}
