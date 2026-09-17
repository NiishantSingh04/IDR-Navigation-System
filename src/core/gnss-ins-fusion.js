/**
 * GNSS + INS Fusion Controller
 * 
 * Manages the seamless transition between:
 * 1. GNSS-aided INS mode (normal operation)
 * 2. Dead reckoning mode (GNSS outage)
 * 
 * Orchestrates the INS engine, Kalman filter, particle filter,
 * and map matching to produce continuous position estimates.
 */

import { INSState, deadReckonSpeedHeading, haversineDistance, DEG2RAD, RAD2DEG, latlngToMeters, metersToLatLng } from './ins-engine.js';
import { ErrorStateEKF } from './kalman-filter.js';
import { ParticleFilter } from './particle-filter.js';
import { RoadNetwork, NHCConstraint } from './map-matching.js';

/**
 * Navigation modes
 */
export const NavMode = {
  GNSS_INS: 'GNSS_INS',           // Normal GNSS-aided operation
  DEAD_RECKONING: 'DEAD_RECKONING', // GNSS outage, DR only
  TRANSITIONING: 'TRANSITIONING',   // Switching between modes
};

/**
 * Fusion engine result for a single timestep
 */
export class FusionResult {
  constructor() {
    this.timestamp = 0;
    this.mode = NavMode.GNSS_INS;
    // Positions
    this.gt_lat = 0;
    this.gt_lng = 0;
    this.gnss_lat = 0;
    this.gnss_lng = 0;
    this.ins_lat = 0;
    this.ins_lng = 0;
    this.fused_lat = 0;
    this.fused_lng = 0;
    this.pf_lat = 0;
    this.pf_lng = 0;
    // Errors
    this.ins_error = 0;
    this.fused_error = 0;
    this.pf_error = 0;
    // Metrics
    this.speed = 0;
    this.heading = 0;
    this.uncertainty = 0;
    this.neff = 0;
    this.gnss_available = true;
    // Sensor data
    this.accel_x = 0;
    this.accel_y = 0;
    this.accel_z = 0;
    this.gyro_yaw = 0;
    this.gyro_pitch = 0;
    this.gyro_roll = 0;
    // Particles
    this.particles = [];
  }
}

/**
 * Main GNSS+INS Fusion Engine
 */
export class FusionEngine {
  constructor(config = {}) {
    this.config = {
      numParticles: config.numParticles || 100,
      dMin: config.dMin || 25,
      useMapMatching: config.useMapMatching !== false,
      useNHC: config.useNHC !== false,
      useParticleFilter: config.useParticleFilter !== false,
      ekfConfig: config.ekfConfig || {},
      ...config,
    };

    this.ekf = new ErrorStateEKF(this.config.ekfConfig);
    this.pf = new ParticleFilter({
      numParticles: this.config.numParticles,
      dMin: this.config.dMin,
    });
    this.roadNetwork = new RoadNetwork();

    this.insState = null;
    this.mode = NavMode.GNSS_INS;
    this.gnssOutage = false;
    this.simulatedOutage = false;
    this.outageStartIndex = -1;
    this.outageEndIndex = -1;

    // State tracking
    this.lastGNSSTime = 0;
    this.totalDistance = 0;
    this.drDistance = 0;
    this.results = [];
    this.currentIndex = 0;
  }

  /**
   * Initialize with the full dataset to build road network
   * @param {Array} data - full dataset array
   */
  initialize(data) {
    if (!data || data.length === 0) return;

    // Build road network from GPS trace
    const trace = data.map(d => ({ lat: d.gt_lat, lng: d.gt_lng }));
    this.roadNetwork.buildFromTrace(trace);

    // Initialize INS state from first data point
    const first = data[0];
    this.insState = new INSState(first.gt_lat, first.gt_lng, first.gt_heading);
    this.insState.vNorth = 0;
    this.insState.vEast = 0;

    // Initialize particle filter
    this.pf.initialize(first.gt_lat, first.gt_lng, first.gt_heading);

    this.results = [];
    this.currentIndex = 0;
    this.totalDistance = 0;
    this.drDistance = 0;
  }

  /**
   * Configure GNSS outage simulation
   * @param {number} startIndex - data index where outage begins
   * @param {number} endIndex - data index where outage ends (-1 for permanent)
   */
  setOutage(startIndex, endIndex = -1) {
    this.outageStartIndex = startIndex;
    this.outageEndIndex = endIndex;
    this.simulatedOutage = startIndex >= 0;
  }

  /**
   * Process a single data point
   * @param {Object} current - current data point
   * @param {Object} prev - previous data point (null for first)
   * @param {number} index - current index in dataset
   * @returns {FusionResult}
   */
  processStep(current, prev, index) {
    const result = new FusionResult();
    result.timestamp = current.time;
    result.gt_lat = current.gt_lat;
    result.gt_lng = current.gt_lng;

    // Compute time delta
    const dt = prev ? (current.time - prev.time) : 0.1;
    const dtClamped = Math.max(0.01, Math.min(dt, 10)); // clamp to reasonable range

    // Determine GNSS availability
    const gnssAvailable = this._isGNSSAvailable(index);
    result.gnss_available = gnssAvailable;

    // Extract sensor data
    const speed_kmh = current.vehicle_speed || current.gt_speed || 0;
    const speed_ms = speed_kmh / 3.6;
    const yawRate_dps = current.yaw_rate || 0;
    const yawRate_rads = yawRate_dps * DEG2RAD;
    const heading_deg = current.gt_heading || 0;

    result.speed = speed_kmh;
    result.heading = heading_deg;
    result.accel_x = current.accel_x || 0;
    result.accel_y = current.accel_y || 0;
    result.accel_z = current.accel_z || 0;
    result.gyro_yaw = current.gyro_yaw || 0;
    result.gyro_pitch = current.gyro_pitch || 0;
    result.gyro_roll = current.gyro_roll || 0;

    // === 1. Pure INS Dead Reckoning (always runs) ===
    if (this.insState && prev) {
      this.insState = deadReckonSpeedHeading(
        this.insState, speed_ms, yawRate_rads, dtClamped
      );

      // Add noise to simulate real INS drift
      if (!gnssAvailable) {
        // Accumulate drift error
        this.insState.lat += (Math.random() - 0.5) * 0.000002;
        this.insState.lng += (Math.random() - 0.5) * 0.000002;
      }
    }

    if (this.insState) {
      result.ins_lat = this.insState.lat;
      result.ins_lng = this.insState.lng;
    }

    // === 2. GNSS+INS Fusion (EKF) ===
    if (prev) {
      // EKF prediction
      this.ekf.predict(dtClamped, speed_ms, heading_deg * DEG2RAD);

      if (gnssAvailable) {
        // EKF GNSS update
        const { north: dN, east: dE } = latlngToMeters(
          current.gt_lat - (result.ins_lat || current.gt_lat),
          current.gt_lng - (result.ins_lng || current.gt_lng),
          current.gt_lat
        );
        this.ekf.updateGNSS(dN, dE);

        // Speed update with NHC
        if (this.config.useNHC) {
          this.ekf.updateSpeed(0, 0);
        }

        this.mode = NavMode.GNSS_INS;
        this.lastGNSSTime = current.time;
      } else {
        this.mode = NavMode.DEAD_RECKONING;
      }
    }

    // Compute fused position
    const ekfState = this.ekf.getPositionUncertainty();
    if (gnssAvailable) {
      result.fused_lat = current.gt_lat;
      result.fused_lng = current.gt_lng;
      result.gnss_lat = current.gt_lat;
      result.gnss_lng = current.gt_lng;

      // Reset INS to GNSS position when available
      if (this.insState) {
        this.insState.lat = current.gt_lat;
        this.insState.lng = current.gt_lng;
        this.insState.heading = heading_deg * DEG2RAD;
      }
    } else {
      result.fused_lat = result.ins_lat;
      result.fused_lng = result.ins_lng;
      result.gnss_lat = null;
      result.gnss_lng = null;
    }

    // === 3. Particle Filter (RBPF) ===
    if (this.config.useParticleFilter && prev) {
      // PF prediction
      this.pf.predict(speed_ms, yawRate_rads, dtClamped);

      if (gnssAvailable) {
        // PF GNSS update
        this.pf.updateGNSS(current.gt_lat, current.gt_lng);
      }

      // PF speed update
      this.pf.updateSpeed(speed_ms);

      // Map matching correction
      if (this.config.useMapMatching && !gnssAvailable) {
        this.pf.correctWithRoadmap((lat, lng) =>
          this.roadNetwork.distanceToNearestRoad(lat, lng)
        );
      }

      const pfEstimate = this.pf.getEstimate();
      result.pf_lat = pfEstimate.lat;
      result.pf_lng = pfEstimate.lng;
      result.uncertainty = pfEstimate.uncertainty;
      result.neff = this.pf.getEffectiveParticleCount();

      // When in DR mode, use PF estimate as fused position
      if (!gnssAvailable) {
        result.fused_lat = pfEstimate.lat;
        result.fused_lng = pfEstimate.lng;
      }

      // Get particle positions for visualization
      if (!gnssAvailable) {
        result.particles = this.pf.getParticlePositions();
      }
    }

    // === 4. Compute errors ===
    result.ins_error = haversineDistance(
      result.gt_lat, result.gt_lng,
      result.ins_lat || result.gt_lat, result.ins_lng || result.gt_lng
    );
    result.fused_error = haversineDistance(
      result.gt_lat, result.gt_lng,
      result.fused_lat, result.fused_lng
    );
    result.pf_error = haversineDistance(
      result.gt_lat, result.gt_lng,
      result.pf_lat || result.gt_lat, result.pf_lng || result.gt_lng
    );

    result.mode = this.mode;
    this.currentIndex = index;

    // Track distances
    if (prev) {
      const segDist = haversineDistance(prev.gt_lat, prev.gt_lng, current.gt_lat, current.gt_lng);
      this.totalDistance += segDist;
      if (!gnssAvailable) this.drDistance += segDist;
    }

    return result;
  }

  /**
   * Check if GNSS is available at the given index
   */
  _isGNSSAvailable(index) {
    if (!this.simulatedOutage) return true;
    if (this.outageEndIndex < 0) {
      return index < this.outageStartIndex;
    }
    return index < this.outageStartIndex || index >= this.outageEndIndex;
  }

  /**
   * Process the entire dataset
   * @param {Array} data - dataset array
   * @returns {Array<FusionResult>}
   */
  processAll(data) {
    this.initialize(data);
    this.results = [];

    for (let i = 0; i < data.length; i++) {
      const prev = i > 0 ? data[i - 1] : null;
      const result = this.processStep(data[i], prev, i);
      this.results.push(result);
    }

    return this.results;
  }

  /**
   * Get summary statistics
   */
  getStats() {
    const drResults = this.results.filter(r => !r.gnss_available);
    const gnssResults = this.results.filter(r => r.gnss_available);

    const insErrors = drResults.map(r => r.ins_error);
    const fusedErrors = drResults.map(r => r.fused_error);
    const pfErrors = drResults.map(r => r.pf_error);

    const mean = arr => arr.length ? arr.reduce((a, b) => a + b, 0) / arr.length : 0;
    const max = arr => arr.length ? Math.max(...arr) : 0;
    const p95 = arr => {
      if (!arr.length) return 0;
      const sorted = [...arr].sort((a, b) => a - b);
      return sorted[Math.floor(sorted.length * 0.95)];
    };

    return {
      totalPoints: this.results.length,
      gnssPoints: gnssResults.length,
      drPoints: drResults.length,
      totalDistance: this.totalDistance,
      drDistance: this.drDistance,
      insError: {
        mean: mean(insErrors),
        max: max(insErrors),
        p95: p95(insErrors),
      },
      fusedError: {
        mean: mean(fusedErrors),
        max: max(fusedErrors),
        p95: p95(fusedErrors),
      },
      pfError: {
        mean: mean(pfErrors),
        max: max(pfErrors),
        p95: p95(pfErrors),
      },
      driftRate: this.drDistance > 0
        ? (mean(insErrors) / this.drDistance) * 100
        : 0,
      pfDriftRate: this.drDistance > 0
        ? (mean(pfErrors) / this.drDistance) * 100
        : 0,
    };
  }
}
