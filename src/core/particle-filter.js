/**
 * Rao-Blackwellized Particle Filter (RBPF) for Map-Aided Dead Reckoning
 * 
 * Implements the algorithm from Mikov et al. (ICINS):
 * - Each particle maintains its own EKF instance (Kalman Filter Bank)
 * - Position errors (N, E) are represented by particles
 * - All other states are estimated by the individual KFs
 * - Roadmap corrections via particle weighing
 * - Systematic resampling
 */

import { ErrorStateEKF } from './kalman-filter.js';
import { haversineDistance, metersToLatLng, latlngToMeters, DEG2RAD } from './ins-engine.js';

/**
 * Single particle in the RBPF
 */
class Particle {
  constructor(lat, lng, heading, weight, ekfConfig) {
    this.lat = lat;
    this.lng = lng;
    this.heading = heading; // radians
    this.weight = weight;
    this.ekf = new ErrorStateEKF(ekfConfig);
    this.speed = 0; // m/s
  }

  clone() {
    const p = new Particle(this.lat, this.lng, this.heading, this.weight);
    p.ekf = this.ekf.clone();
    p.speed = this.speed;
    return p;
  }
}

/**
 * RBPF Configuration
 */
const DEFAULT_CONFIG = {
  numParticles: 100,
  posSpread: 0.0001,      // initial position spread in degrees
  headingSpread: 10,       // initial heading spread in degrees
  dMin: 25,                // roadmap distance threshold (meters)
  resampleThreshold: 0.5,  // N_eff / N threshold for resampling
  mixingInterval: 5,       // seconds between marginalization mixing
  speedNoiseFactor: 0.05,  // speed measurement noise factor
  headingNoiseFactor: 0.02 // heading noise factor per step
};

/**
 * Rao-Blackwellized Particle Filter
 */
export class ParticleFilter {
  constructor(config = {}) {
    this.config = { ...DEFAULT_CONFIG, ...config };
    this.particles = [];
    this.initialized = false;
    this.lastMixTime = 0;
    this.stepCount = 0;
  }

  /**
   * Initialize the particle filter with a known position and heading
   */
  initialize(lat, lng, heading) {
    const N = this.config.numParticles;
    this.particles = [];

    for (let i = 0; i < N; i++) {
      // Spread particles around initial position
      const pLat = lat + (Math.random() - 0.5) * 2 * this.config.posSpread;
      const pLng = lng + (Math.random() - 0.5) * 2 * this.config.posSpread;
      const pHeading = (heading + (Math.random() - 0.5) * 2 * this.config.headingSpread) * DEG2RAD;

      this.particles.push(new Particle(pLat, pLng, pHeading, 1 / N, {
        posVar: 50,
        velVar: 10,
        headingVar: 0.05,
      }));
    }

    this.initialized = true;
    this.lastMixTime = 0;
    this.stepCount = 0;
  }

  /**
   * Prediction step: propagate each particle using dead reckoning
   * @param {number} speed_ms - estimated speed in m/s
   * @param {number} yawRate_rads - yaw rate in rad/s
   * @param {number} dt - time step in seconds
   */
  predict(speed_ms, yawRate_rads, dt) {
    if (!this.initialized) return;

    const speedNoise = this.config.speedNoiseFactor;
    const headingNoise = this.config.headingNoiseFactor;

    for (const p of this.particles) {
      // Add process noise
      const noisySpeed = speed_ms * (1 + (Math.random() - 0.5) * 2 * speedNoise);
      const noisyYawRate = yawRate_rads + (Math.random() - 0.5) * 2 * headingNoise;

      // Update heading
      p.heading += noisyYawRate * dt;
      while (p.heading > Math.PI) p.heading -= 2 * Math.PI;
      while (p.heading < -Math.PI) p.heading += 2 * Math.PI;

      // Propagate position
      const dist = noisySpeed * dt;
      const dNorth = dist * Math.cos(p.heading);
      const dEast = dist * Math.sin(p.heading);

      const delta = metersToLatLng(dNorth, dEast, p.lat);
      p.lat += delta.dLat;
      p.lng += delta.dLng;
      p.speed = noisySpeed;

      // Propagate each particle's EKF
      p.ekf.predict(dt, noisySpeed, p.heading);
    }

    this.stepCount++;
  }

  /**
   * GNSS measurement update
   * @param {number} gpsLat - GNSS latitude
   * @param {number} gpsLng - GNSS longitude
   */
  updateGNSS(gpsLat, gpsLng) {
    if (!this.initialized) return;

    for (const p of this.particles) {
      const { north: dN, east: dE } = latlngToMeters(
        gpsLat - p.lat, gpsLng - p.lng, p.lat
      );

      // Update particle weight based on measurement likelihood
      const dist = Math.sqrt(dN * dN + dE * dE);
      const sigma = 5; // GNSS accuracy ~5m
      const likelihood = Math.exp(-0.5 * (dist / sigma) ** 2);
      p.weight *= likelihood;

      // Update particle's EKF
      const corrections = p.ekf.updateGNSS(dN, dE);

      // Apply position corrections from EKF
      const posDelta = metersToLatLng(corrections.dN * 0.3, corrections.dE * 0.3, p.lat);
      p.lat += posDelta.dLat;
      p.lng += posDelta.dLng;
      p.heading += corrections.dPsi * 0.1;
    }

    this._normalizeWeights();
    this._resampleIfNeeded();
  }

  /**
   * Speed measurement update (Non-Holonomic Constraint)
   * Assumes zero lateral velocity
   * @param {number} speed_ms - measured speed in m/s
   */
  updateSpeed(speed_ms) {
    if (!this.initialized) return;

    for (const p of this.particles) {
      const expectedVn = speed_ms * Math.cos(p.heading);
      const expectedVe = speed_ms * Math.sin(p.heading);

      p.ekf.updateSpeed(
        expectedVn - p.speed * Math.cos(p.heading),
        expectedVe - p.speed * Math.sin(p.heading)
      );
    }
  }

  /**
   * Roadmap correction: weight particles by distance to nearest road
   * Implements equation (11) from Mikov: w(d) = 1/log(max(d_min, d))
   * @param {Function} nearestRoadFn - function(lat, lng) => distance in meters
   */
  correctWithRoadmap(nearestRoadFn) {
    if (!this.initialized) return;

    const dMin = this.config.dMin;

    for (const p of this.particles) {
      const d = nearestRoadFn(p.lat, p.lng);
      const w = 1 / Math.log(Math.max(dMin, d));
      p.weight *= w;
    }

    this._normalizeWeights();
    this._resampleIfNeeded();
  }

  /**
   * Get weighted average position estimate
   */
  getEstimate() {
    if (!this.initialized || this.particles.length === 0) {
      return { lat: 0, lng: 0, heading: 0, speed: 0, uncertainty: 0 };
    }

    let lat = 0, lng = 0, heading = 0, speed = 0;
    let sinH = 0, cosH = 0;

    for (const p of this.particles) {
      lat += p.weight * p.lat;
      lng += p.weight * p.lng;
      sinH += p.weight * Math.sin(p.heading);
      cosH += p.weight * Math.cos(p.heading);
      speed += p.weight * p.speed;
    }

    heading = Math.atan2(sinH, cosH);

    // Calculate position spread as uncertainty measure
    let varN = 0, varE = 0;
    for (const p of this.particles) {
      const { north: dN, east: dE } = latlngToMeters(p.lat - lat, p.lng - lng, lat);
      varN += p.weight * dN * dN;
      varE += p.weight * dE * dE;
    }
    const uncertainty = Math.sqrt(varN + varE);

    return { lat, lng, heading, speed, uncertainty };
  }

  /**
   * Get all particle positions for visualization
   */
  getParticlePositions() {
    return this.particles.map(p => ({
      lat: p.lat,
      lng: p.lng,
      weight: p.weight
    }));
  }

  /**
   * Get effective number of particles
   */
  getEffectiveParticleCount() {
    let sumSq = 0;
    for (const p of this.particles) sumSq += p.weight * p.weight;
    return sumSq > 0 ? 1 / sumSq : 0;
  }

  /**
   * Normalize particle weights to sum to 1
   */
  _normalizeWeights() {
    let sum = 0;
    for (const p of this.particles) sum += p.weight;
    if (sum > 0) {
      for (const p of this.particles) p.weight /= sum;
    } else {
      const uniform = 1 / this.particles.length;
      for (const p of this.particles) p.weight = uniform;
    }
  }

  /**
   * Resample particles if effective count is too low
   * Uses systematic resampling
   */
  _resampleIfNeeded() {
    const N = this.particles.length;
    const Neff = this.getEffectiveParticleCount();

    if (Neff > this.config.resampleThreshold * N) return;

    // Systematic resampling
    const cumWeights = new Float64Array(N);
    cumWeights[0] = this.particles[0].weight;
    for (let i = 1; i < N; i++) {
      cumWeights[i] = cumWeights[i - 1] + this.particles[i].weight;
    }

    const step = 1 / N;
    let u = Math.random() * step;
    const newParticles = [];

    let j = 0;
    for (let i = 0; i < N; i++) {
      while (j < N - 1 && cumWeights[j] < u) j++;
      const clone = this.particles[j].clone();
      clone.weight = 1 / N;

      // Add small perturbation to avoid particle collapse
      clone.lat += (Math.random() - 0.5) * 0.00001;
      clone.lng += (Math.random() - 0.5) * 0.00001;
      clone.heading += (Math.random() - 0.5) * 0.01;

      newParticles.push(clone);
      u += step;
    }

    this.particles = newParticles;
  }
}
