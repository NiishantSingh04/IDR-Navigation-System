/**
 * Strapdown Inertial Navigation System (INS) Engine
 * Implements IMU mechanization: integrates accelerometer and gyroscope
 * measurements to propagate position, velocity, and attitude.
 */

const DEG2RAD = Math.PI / 180;
const RAD2DEG = 180 / Math.PI;
const EARTH_RADIUS = 6371000; // meters
const GRAVITY = 9.80665; // m/s^2

/**
 * Create a 3x3 rotation matrix from Euler angles (ZYX convention)
 * @param {number} yaw - heading in radians
 * @param {number} pitch - pitch in radians
 * @param {number} roll - roll in radians
 * @returns {number[][]} 3x3 rotation matrix
 */
export function eulerToRotationMatrix(yaw, pitch, roll) {
  const cy = Math.cos(yaw), sy = Math.sin(yaw);
  const cp = Math.cos(pitch), sp = Math.sin(pitch);
  const cr = Math.cos(roll), sr = Math.sin(roll);

  return [
    [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
    [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
    [-sp, cp * sr, cp * cr]
  ];
}

/**
 * Multiply 3x3 matrix by 3x1 vector
 */
export function matVecMul(M, v) {
  return [
    M[0][0] * v[0] + M[0][1] * v[1] + M[0][2] * v[2],
    M[1][0] * v[0] + M[1][1] * v[1] + M[1][2] * v[2],
    M[2][0] * v[0] + M[2][1] * v[1] + M[2][2] * v[2]
  ];
}

/**
 * Transpose 3x3 matrix
 */
export function transpose3(M) {
  return [
    [M[0][0], M[1][0], M[2][0]],
    [M[0][1], M[1][1], M[2][1]],
    [M[0][2], M[1][2], M[2][2]]
  ];
}

/**
 * Convert lat/lng position change to meters (local tangent plane)
 * @param {number} dLat - latitude change in degrees
 * @param {number} dLng - longitude change in degrees
 * @param {number} refLat - reference latitude in degrees
 * @returns {{north: number, east: number}} displacement in meters
 */
export function latlngToMeters(dLat, dLng, refLat) {
  const north = dLat * DEG2RAD * EARTH_RADIUS;
  const east = dLng * DEG2RAD * EARTH_RADIUS * Math.cos(refLat * DEG2RAD);
  return { north, east };
}

/**
 * Convert meter displacement to lat/lng delta
 */
export function metersToLatLng(north, east, refLat) {
  const dLat = (north / EARTH_RADIUS) * RAD2DEG;
  const dLng = (east / (EARTH_RADIUS * Math.cos(refLat * DEG2RAD))) * RAD2DEG;
  return { dLat, dLng };
}

/**
 * INS State
 */
export class INSState {
  constructor(lat, lng, heading) {
    this.lat = lat;
    this.lng = lng;
    this.heading = heading * DEG2RAD; // radians
    this.pitch = 0;
    this.roll = 0;
    this.vNorth = 0; // m/s
    this.vEast = 0;
    this.vDown = 0;
    this.accelBias = [0, 0, 0]; // m/s^2
    this.gyroBias = [0, 0, 0]; // rad/s
  }

  clone() {
    const s = new INSState(this.lat, this.lng, this.heading * RAD2DEG);
    s.heading = this.heading;
    s.pitch = this.pitch;
    s.roll = this.roll;
    s.vNorth = this.vNorth;
    s.vEast = this.vEast;
    s.vDown = this.vDown;
    s.accelBias = [...this.accelBias];
    s.gyroBias = [...this.gyroBias];
    return s;
  }
}

/**
 * Strapdown INS mechanization step
 * @param {INSState} state - current state
 * @param {Object} imu - IMU measurements {accel_x, accel_y, accel_z, gyro_yaw, gyro_pitch, gyro_roll}
 * @param {number} dt - time step in seconds
 * @returns {INSState} updated state
 */
export function insPropagate(state, imu, dt) {
  const s = state.clone();

  // Compensate biases
  const ax = imu.accel_x - s.accelBias[0];
  const ay = imu.accel_y - s.accelBias[1];
  const az = imu.accel_z - s.accelBias[2];

  const wx = imu.gyro_roll - s.gyroBias[0];
  const wy = imu.gyro_pitch - s.gyroBias[1];
  const wz = imu.gyro_yaw - s.gyroBias[2];

  // Update attitude
  s.roll += wx * dt;
  s.pitch += wy * dt;
  s.heading += wz * dt;

  // Normalize heading
  while (s.heading > Math.PI) s.heading -= 2 * Math.PI;
  while (s.heading < -Math.PI) s.heading += 2 * Math.PI;

  // Body-to-nav rotation matrix
  const Rbn = eulerToRotationMatrix(s.heading, s.pitch, s.roll);

  // Transform body accelerations to navigation frame
  const accelBody = [ax, ay, az];
  const accelNav = matVecMul(Rbn, accelBody);

  // Remove gravity (navigation frame: North-East-Down)
  accelNav[2] += GRAVITY;

  // Update velocity
  s.vNorth += accelNav[0] * dt;
  s.vEast += accelNav[1] * dt;
  s.vDown += accelNav[2] * dt;

  // Update position
  const displacement = metersToLatLng(s.vNorth * dt, s.vEast * dt, s.lat);
  s.lat += displacement.dLat;
  s.lng += displacement.dLng;

  return s;
}

/**
 * Simplified dead reckoning using speed + heading only
 * (More stable than full strapdown for noisy smartphone IMU)
 */
export function deadReckonSpeedHeading(state, speed_ms, yawRate_rads, dt) {
  const s = state.clone();

  // Update heading
  s.heading += yawRate_rads * dt;
  while (s.heading > Math.PI) s.heading -= 2 * Math.PI;
  while (s.heading < -Math.PI) s.heading += 2 * Math.PI;

  // Calculate displacement
  const dist = speed_ms * dt;
  const dNorth = dist * Math.cos(s.heading);
  const dEast = dist * Math.sin(s.heading);

  const delta = metersToLatLng(dNorth, dEast, s.lat);
  s.lat += delta.dLat;
  s.lng += delta.dLng;
  s.vNorth = speed_ms * Math.cos(s.heading);
  s.vEast = speed_ms * Math.sin(s.heading);

  return s;
}

/**
 * Haversine distance between two lat/lng points in meters
 */
export function haversineDistance(lat1, lng1, lat2, lng2) {
  const dLat = (lat2 - lat1) * DEG2RAD;
  const dLng = (lng2 - lng1) * DEG2RAD;
  const a = Math.sin(dLat / 2) ** 2 +
    Math.cos(lat1 * DEG2RAD) * Math.cos(lat2 * DEG2RAD) *
    Math.sin(dLng / 2) ** 2;
  return EARTH_RADIUS * 2 * Math.atan2(Math.sqrt(a), Math.sqrt(1 - a));
}

export { DEG2RAD, RAD2DEG, EARTH_RADIUS, GRAVITY };
