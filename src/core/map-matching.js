/**
 * Map Matching & Non-Holonomic Constraints (NHC)
 * 
 * Provides road network constraints for the particle filter.
 * Uses the ground truth trajectory as a pseudo road network 
 * (since we don't have OSM data loaded, we build road segments
 * from the dataset's GPS trace).
 */

import { haversineDistance, latlngToMeters } from './ins-engine.js';

/**
 * Road segment represented as a line between two GPS points
 */
class RoadSegment {
  constructor(lat1, lng1, lat2, lng2) {
    this.lat1 = lat1;
    this.lng1 = lng1;
    this.lat2 = lat2;
    this.lng2 = lng2;
    // Pre-compute bounding box for fast rejection
    this.minLat = Math.min(lat1, lat2);
    this.maxLat = Math.max(lat1, lat2);
    this.minLng = Math.min(lng1, lng2);
    this.maxLng = Math.max(lng1, lng2);
  }
}

/**
 * Road Network for map matching
 * Builds a spatial index of road segments from GPS traces
 */
export class RoadNetwork {
  constructor() {
    this.segments = [];
    this.gridSize = 0.001; // ~111m grid cells
    this.grid = new Map();
  }

  /**
   * Build road network from a GPS trace
   * @param {Array<{lat: number, lng: number}>} trace - GPS points
   */
  buildFromTrace(trace) {
    this.segments = [];
    this.grid = new Map();

    for (let i = 0; i < trace.length - 1; i++) {
      const seg = new RoadSegment(
        trace[i].lat, trace[i].lng,
        trace[i + 1].lat, trace[i + 1].lng
      );
      this.segments.push(seg);

      // Index into grid
      const key = this._gridKey(
        (seg.minLat + seg.maxLat) / 2,
        (seg.minLng + seg.maxLng) / 2
      );
      if (!this.grid.has(key)) this.grid.set(key, []);
      this.grid.get(key).push(seg);
    }
  }

  /**
   * Find distance to nearest road segment
   * @param {number} lat 
   * @param {number} lng 
   * @returns {number} distance in meters
   */
  distanceToNearestRoad(lat, lng) {
    let minDist = Infinity;

    // Check nearby grid cells
    const key = this._gridKey(lat, lng);
    const cells = this._getNearbyCells(key);

    for (const cellKey of cells) {
      const segs = this.grid.get(cellKey);
      if (!segs) continue;

      for (const seg of segs) {
        const d = this._pointToSegmentDistance(lat, lng, seg);
        if (d < minDist) minDist = d;
      }
    }

    // If no nearby segments found, search all (fallback)
    if (minDist === Infinity) {
      for (const seg of this.segments) {
        const d = this._pointToSegmentDistance(lat, lng, seg);
        if (d < minDist) minDist = d;
      }
    }

    return minDist;
  }

  /**
   * Snap a position to the nearest road
   * @returns {{lat: number, lng: number, distance: number}}
   */
  snapToRoad(lat, lng) {
    let minDist = Infinity;
    let snapLat = lat, snapLng = lng;

    const key = this._gridKey(lat, lng);
    const cells = this._getNearbyCells(key);

    for (const cellKey of cells) {
      const segs = this.grid.get(cellKey);
      if (!segs) continue;

      for (const seg of segs) {
        const result = this._nearestPointOnSegment(lat, lng, seg);
        if (result.distance < minDist) {
          minDist = result.distance;
          snapLat = result.lat;
          snapLng = result.lng;
        }
      }
    }

    return { lat: snapLat, lng: snapLng, distance: minDist };
  }

  /**
   * Grid key from lat/lng
   */
  _gridKey(lat, lng) {
    const gLat = Math.floor(lat / this.gridSize);
    const gLng = Math.floor(lng / this.gridSize);
    return `${gLat},${gLng}`;
  }

  /**
   * Get keys for nearby grid cells (3x3 neighborhood)
   */
  _getNearbyCells(key) {
    const [gLat, gLng] = key.split(',').map(Number);
    const cells = [];
    for (let di = -1; di <= 1; di++) {
      for (let dj = -1; dj <= 1; dj++) {
        cells.push(`${gLat + di},${gLng + dj}`);
      }
    }
    return cells;
  }

  /**
   * Distance from point to line segment in meters
   */
  _pointToSegmentDistance(lat, lng, seg) {
    const result = this._nearestPointOnSegment(lat, lng, seg);
    return result.distance;
  }

  /**
   * Find nearest point on a line segment to a given point
   */
  _nearestPointOnSegment(lat, lng, seg) {
    const { north: pN, east: pE } = latlngToMeters(lat - seg.lat1, lng - seg.lng1, seg.lat1);
    const { north: sN, east: sE } = latlngToMeters(seg.lat2 - seg.lat1, seg.lng2 - seg.lng1, seg.lat1);

    const segLenSq = sN * sN + sE * sE;
    if (segLenSq < 0.001) {
      // Degenerate segment
      return {
        lat: seg.lat1,
        lng: seg.lng1,
        distance: Math.sqrt(pN * pN + pE * pE)
      };
    }

    // Project point onto segment
    let t = (pN * sN + pE * sE) / segLenSq;
    t = Math.max(0, Math.min(1, t));

    const nearN = t * sN;
    const nearE = t * sE;
    const dN = pN - nearN;
    const dE = pE - nearE;

    const nearLat = seg.lat1 + t * (seg.lat2 - seg.lat1);
    const nearLng = seg.lng1 + t * (seg.lng2 - seg.lng1);

    return {
      lat: nearLat,
      lng: nearLng,
      distance: Math.sqrt(dN * dN + dE * dE)
    };
  }
}

/**
 * Non-Holonomic Constraints (NHC)
 * Assumes vehicle cannot move sideways or vertically
 */
export class NHCConstraint {
  /**
   * Apply NHC: constrain velocity to forward direction only
   * @param {number} vNorth - north velocity
   * @param {number} vEast - east velocity
   * @param {number} heading - heading in radians
   * @returns {{vNorth: number, vEast: number}} constrained velocities
   */
  static apply(vNorth, vEast, heading) {
    // Project velocity onto heading direction
    const speed = vNorth * Math.cos(heading) + vEast * Math.sin(heading);
    const constrainedSpeed = Math.max(0, speed); // no reverse assumed

    return {
      vNorth: constrainedSpeed * Math.cos(heading),
      vEast: constrainedSpeed * Math.sin(heading)
    };
  }
}
