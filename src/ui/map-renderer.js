/**
 * Map Renderer Module
 * Handles Leaflet map initialization, trajectory rendering, and particle visualization
 */

let map = null;
let groundTruthLayer = null;
let insLayer = null;
let fusedLayer = null;
let particleLayer = null;
let vehicleMarker = null;
let gtMarker = null;
let outageOverlay = null;

const COLORS = {
  groundTruth: '#1b7a2b',
  ins: '#c62828',
  fused: '#1a73e8',
  particle: 'rgba(26, 115, 232, 0.35)',
  vehicle: '#1a73e8',
  outageZone: 'rgba(198, 40, 40, 0.15)',
};

/**
 * Initialize the Leaflet map
 */
export function initMap(centerLat, centerLng, zoom = 14) {
  if (map) {
    map.remove();
  }

  map = L.map('map', {
    center: [centerLat, centerLng],
    zoom: zoom,
    zoomControl: true,
    attributionControl: true,
  });

  // Use a clean dark-styled tile layer
  L.tileLayer('https://{s}.basemaps.cartocdn.com/dark_all/{z}/{x}/{y}{r}.png', {
    attribution: 'CartoDB | OpenStreetMap',
    subdomains: 'abcd',
    maxZoom: 19
  }).addTo(map);

  // Initialize layers
  groundTruthLayer = L.layerGroup().addTo(map);
  insLayer = L.layerGroup().addTo(map);
  fusedLayer = L.layerGroup().addTo(map);
  particleLayer = L.layerGroup().addTo(map);

  // Vehicle position marker
  vehicleMarker = L.circleMarker([centerLat, centerLng], {
    radius: 7,
    fillColor: COLORS.vehicle,
    fillOpacity: 1,
    color: '#ffffff',
    weight: 2,
    pane: 'markerPane',
  }).addTo(map);

  // Ground truth marker
  gtMarker = L.circleMarker([centerLat, centerLng], {
    radius: 5,
    fillColor: COLORS.groundTruth,
    fillOpacity: 0.9,
    color: '#ffffff',
    weight: 1.5,
    pane: 'markerPane',
  }).addTo(map);

  return map;
}

/**
 * Draw the full ground truth path
 */
export function drawGroundTruth(points) {
  if (!map || !groundTruthLayer) return;
  groundTruthLayer.clearLayers();

  const latlngs = points.map(p => [p.gt_lat, p.gt_lng]);
  L.polyline(latlngs, {
    color: COLORS.groundTruth,
    weight: 3,
    opacity: 0.8,
    smoothFactor: 1,
  }).addTo(groundTruthLayer);
}

/**
 * Update the INS trajectory incrementally
 */
let insPoints = [];
let insPolyline = null;

export function resetINSPath() {
  insPoints = [];
  if (insLayer) insLayer.clearLayers();
  insPolyline = null;
}

export function addINSPoint(lat, lng) {
  insPoints.push([lat, lng]);
  if (insLayer) {
    insLayer.clearLayers();
    insPolyline = L.polyline(insPoints, {
      color: COLORS.ins,
      weight: 2.5,
      opacity: 0.8,
      dashArray: '8, 6',
      smoothFactor: 1,
    }).addTo(insLayer);
  }
}

/**
 * Update the fused trajectory incrementally
 */
let fusedPoints = [];
let fusedPolyline = null;

export function resetFusedPath() {
  fusedPoints = [];
  if (fusedLayer) fusedLayer.clearLayers();
  fusedPolyline = null;
}

export function addFusedPoint(lat, lng) {
  fusedPoints.push([lat, lng]);
  if (fusedLayer) {
    fusedLayer.clearLayers();
    fusedPolyline = L.polyline(fusedPoints, {
      color: COLORS.fused,
      weight: 3,
      opacity: 0.9,
      smoothFactor: 1,
    }).addTo(fusedLayer);
  }
}

/**
 * Draw particle cloud
 */
export function drawParticles(particles) {
  if (!particleLayer) return;
  particleLayer.clearLayers();

  if (!particles || particles.length === 0) return;

  // Draw particles as small circles
  const maxWeight = Math.max(...particles.map(p => p.weight));
  for (const p of particles) {
    const opacity = Math.min(1, (p.weight / maxWeight) * 2);
    L.circleMarker([p.lat, p.lng], {
      radius: 3,
      fillColor: COLORS.fused,
      fillOpacity: opacity * 0.5,
      color: COLORS.fused,
      weight: 0.5,
      opacity: opacity * 0.7,
    }).addTo(particleLayer);
  }
}

/**
 * Update vehicle marker position
 */
export function updateVehiclePosition(lat, lng, heading) {
  if (vehicleMarker) {
    vehicleMarker.setLatLng([lat, lng]);
  }
}

/**
 * Update ground truth marker position
 */
export function updateGTPosition(lat, lng) {
  if (gtMarker) {
    gtMarker.setLatLng([lat, lng]);
  }
}

/**
 * Set map view to follow vehicle
 */
export function panToVehicle(lat, lng) {
  if (map) {
    map.panTo([lat, lng], { animate: true, duration: 0.3 });
  }
}

/**
 * Fit map to bounds
 */
export function fitBounds(bounds) {
  if (map) {
    map.fitBounds([
      [bounds.min_lat, bounds.min_lng],
      [bounds.max_lat, bounds.max_lng]
    ], { padding: [30, 30] });
  }
}

/**
 * Highlight outage zone on the ground truth path
 */
export function highlightOutageZone(points, startIdx, endIdx) {
  if (outageOverlay) {
    map.removeLayer(outageOverlay);
    outageOverlay = null;
  }
  
  if (startIdx < 0 || startIdx >= points.length) return;
  const end = endIdx < 0 ? points.length : Math.min(endIdx, points.length);
  
  const latlngs = [];
  for (let i = startIdx; i < end; i++) {
    latlngs.push([points[i].gt_lat, points[i].gt_lng]);
  }
  
  if (latlngs.length > 1) {
    outageOverlay = L.polyline(latlngs, {
      color: '#c62828',
      weight: 8,
      opacity: 0.25,
      smoothFactor: 1,
    }).addTo(map);
  }
}

/**
 * Clear outage zone highlight
 */
export function clearOutageHighlight() {
  if (outageOverlay && map) {
    map.removeLayer(outageOverlay);
    outageOverlay = null;
  }
}

/**
 * Update vehicle marker style based on mode
 */
export function setVehicleMode(isGNSS) {
  if (vehicleMarker) {
    vehicleMarker.setStyle({
      fillColor: isGNSS ? COLORS.groundTruth : COLORS.fused,
      color: '#ffffff',
    });
  }
}

export function getMap() {
  return map;
}
