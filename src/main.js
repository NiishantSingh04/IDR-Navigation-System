/**
 * IDR System Prototype - Main Application
 * 
 * Orchestrates the GNSS+INS fusion engine, map visualization,
 * charts, and user controls for the Intelligent Dead Reckoning prototype.
 */

import './styles/main.css';
import { loadDataset } from './data/dataset-loader.js';
import { FusionEngine, NavMode } from './core/gnss-ins-fusion.js';
import {
  initMap, drawGroundTruth, resetINSPath, addINSPoint,
  resetFusedPath, addFusedPoint, drawParticles,
  updateVehiclePosition, updateGTPosition, panToVehicle,
  fitBounds, highlightOutageZone, clearOutageHighlight,
  setVehicleMode,
} from './ui/map-renderer.js';
import { initCharts, updateCharts, resetCharts } from './ui/charts.js';

// === Application State ===
let dataset = null;
let fusionEngine = null;
let results = [];
let isPlaying = false;
let playbackIndex = 0;
let playbackSpeed = 5;
let animationFrameId = null;
let lastFrameTime = 0;

// === DOM References ===
const $ = id => document.getElementById(id);

// === Initialization ===
async function init() {
  try {
    // Load dataset
    const json = await loadDataset('/data/sample-route.json');
    dataset = json.data;
    const metadata = json.metadata;

    console.log(`Dataset loaded: ${dataset.length} points`);
    console.log(`Bounds:`, metadata.bounds);

    // Initialize map
    initMap(metadata.bounds.center_lat, metadata.bounds.center_lng, 14);

    // Draw ground truth trajectory
    drawGroundTruth(dataset);

    // Fit map to data bounds
    fitBounds(metadata.bounds);

    // Initialize charts
    initCharts();

    // Initialize fusion engine
    fusionEngine = new FusionEngine({
      numParticles: 100,
      dMin: 25,
      useMapMatching: true,
      useNHC: true,
      useParticleFilter: true,
    });

    // Setup controls
    setupControls();

    // Process full dataset initially
    runProcessing();

    // Hide loading overlay
    setTimeout(() => {
      $('loadingOverlay').classList.add('hidden');
    }, 800);

  } catch (err) {
    console.error('Failed to initialize:', err);
    $('loadingOverlay').querySelector('.loading-text').textContent = 'Failed to load dataset';
    $('loadingOverlay').querySelector('.loading-subtitle').textContent = err.message;
  }
}

// === Data Processing ===
function runProcessing() {
  if (!dataset || !fusionEngine) return;

  // Get current config
  const numParticles = parseInt($('particleCount').value);
  const dMin = parseInt($('dMinSlider').value);
  const useMapMatch = $('toggleMapMatch').checked;
  const useNHC = $('toggleNHC').checked;
  const usePF = $('togglePF').checked;

  // Create new engine with current config
  fusionEngine = new FusionEngine({
    numParticles,
    dMin,
    useMapMatching: useMapMatch,
    useNHC,
    useParticleFilter: usePF,
  });

  // Apply outage if configured
  const outageStart = parseInt($('outageStart').value);
  const outageDuration = parseInt($('outageDuration').value);
  const startIdx = Math.floor(dataset.length * outageStart / 100);
  const durationPoints = Math.floor(dataset.length * outageDuration / 100);
  
  if ($('outageInfo').style.display !== 'none') {
    fusionEngine.setOutage(startIdx, startIdx + durationPoints);
    highlightOutageZone(dataset, startIdx, startIdx + durationPoints);
  }

  // Process all data
  results = fusionEngine.processAll(dataset);

  // Update stats
  updateStats();

  // Reset visualization
  resetPlayback();
}

// === Playback ===
function resetPlayback() {
  stopPlayback();
  playbackIndex = 0;
  resetINSPath();
  resetFusedPath();
  resetCharts();
  drawParticles([]);
  updateProgressUI();
  updateMetricsUI(results[0] || null);
}

function startPlayback() {
  if (playbackIndex >= results.length) {
    playbackIndex = 0;
    resetINSPath();
    resetFusedPath();
    resetCharts();
  }
  isPlaying = true;
  $('iconPlay').style.display = 'none';
  $('iconPause').style.display = 'block';
  lastFrameTime = performance.now();
  animationLoop();
}

function stopPlayback() {
  isPlaying = false;
  $('iconPlay').style.display = 'block';
  $('iconPause').style.display = 'none';
  if (animationFrameId) {
    cancelAnimationFrame(animationFrameId);
    animationFrameId = null;
  }
}

function animationLoop(timestamp) {
  if (!isPlaying) return;

  const elapsed = timestamp - lastFrameTime;
  const interval = 1000 / (10 * playbackSpeed); // base rate is 10 Hz

  if (elapsed >= interval) {
    lastFrameTime = timestamp;

    // Advance multiple steps per frame at high speeds
    const stepsPerFrame = Math.max(1, Math.floor(playbackSpeed / 10));
    
    for (let s = 0; s < stepsPerFrame && playbackIndex < results.length; s++) {
      const r = results[playbackIndex];

      // Update map
      if (r.ins_lat && r.ins_lng) {
        addINSPoint(r.ins_lat, r.ins_lng);
      }

      if (r.fused_lat && r.fused_lng) {
        addFusedPoint(r.fused_lat, r.fused_lng);
      }

      // Draw particles during DR mode
      if (r.particles && r.particles.length > 0) {
        drawParticles(r.particles);
      } else if (r.gnss_available) {
        drawParticles([]);
      }

      // Update vehicle marker
      updateVehiclePosition(r.fused_lat, r.fused_lng, r.heading);
      updateGTPosition(r.gt_lat, r.gt_lng);

      // Update mode badge
      setVehicleMode(r.gnss_available);

      // Update charts (throttled)
      if (playbackIndex % 2 === 0) {
        updateCharts(r, playbackIndex);
      }

      playbackIndex++;
    }

    // Update UI
    if (playbackIndex < results.length) {
      const r = results[playbackIndex - 1];
      updateMetricsUI(r);
      updateProgressUI();

      // Auto-pan map every N steps
      if (playbackIndex % 20 === 0) {
        panToVehicle(r.fused_lat || r.gt_lat, r.fused_lng || r.gt_lng);
      }
    }

    if (playbackIndex >= results.length) {
      stopPlayback();
      updateStats();
      return;
    }
  }

  animationFrameId = requestAnimationFrame(animationLoop);
}

// === UI Updates ===
function updateMetricsUI(result) {
  if (!result) return;

  // Mode badge
  const badge = $('modeBadge');
  const modeText = $('modeText');
  if (result.gnss_available) {
    badge.className = 'mode-badge gnss';
    modeText.textContent = 'GNSS+INS';
  } else {
    badge.className = 'mode-badge dr';
    modeText.textContent = 'DEAD RECKONING';
  }

  // Metrics
  const posErr = result.fused_error || 0;
  const errEl = $('metricPosError');
  errEl.innerHTML = `${posErr.toFixed(1)}<span class="metric-unit">m</span>`;
  errEl.className = 'metric-value ' + (posErr < 5 ? 'success' : posErr < 20 ? 'warning' : 'danger');

  $('metricSpeed').innerHTML = `${(result.speed || 0).toFixed(1)}<span class="metric-unit">km/h</span>`;
  $('metricHeading').innerHTML = `${(result.heading || 0).toFixed(1)}<span class="metric-unit">deg</span>`;
  $('metricUncertainty').innerHTML = `${(result.uncertainty || 0).toFixed(1)}<span class="metric-unit">m</span>`;

  // Error bars
  const maxError = 100;
  const insErr = result.ins_error || 0;
  const pfErr = result.pf_error || 0;
  const fusedErr = result.fused_error || 0;

  $('insErrorVal').textContent = `${insErr.toFixed(1)} m`;
  $('pfErrorVal').textContent = `${pfErr.toFixed(1)} m`;
  $('fusedErrorVal').textContent = `${fusedErr.toFixed(1)} m`;

  $('insErrorBar').style.width = `${Math.min(100, (insErr / maxError) * 100)}%`;
  $('pfErrorBar').style.width = `${Math.min(100, (pfErr / maxError) * 100)}%`;
  $('fusedErrorBar').style.width = `${Math.min(100, (fusedErr / maxError) * 100)}%`;
}

function updateProgressUI() {
  const total = results.length;
  const pct = total > 0 ? (playbackIndex / total) * 100 : 0;
  $('progressFill').style.width = `${pct}%`;
  $('progressHandle').style.left = `${pct}%`;
  $('playbackTime').textContent = `${playbackIndex} / ${total}`;
}

function updateStats() {
  if (!fusionEngine) return;
  const stats = fusionEngine.getStats();

  $('statTotalDist').textContent = `${(stats.totalDistance / 1000).toFixed(2)} km`;
  $('statDRDist').textContent = `${(stats.drDistance / 1000).toFixed(2)} km`;
  $('statMeanINS').textContent = `${stats.insError.mean.toFixed(1)} m`;
  $('statMeanPF').textContent = `${stats.pfError.mean.toFixed(1)} m`;
  $('statDriftRate').textContent = `${stats.driftRate.toFixed(1)}%`;
  $('statPFDrift').textContent = `${stats.pfDriftRate.toFixed(1)}%`;
  $('statP95INS').textContent = `${stats.insError.p95.toFixed(1)} m`;
  $('statP95PF').textContent = `${stats.pfError.p95.toFixed(1)} m`;
  $('statNeff').textContent = results.length > 0 
    ? `${(results[results.length - 1].neff || 0).toFixed(0)}`
    : '--';
}

// === Controls Setup ===
function setupControls() {
  // Play/Pause
  $('btnPlayPause').addEventListener('click', () => {
    if (isPlaying) {
      stopPlayback();
    } else {
      startPlayback();
    }
  });

  // Restart
  $('btnRestart').addEventListener('click', () => {
    resetPlayback();
  });

  // Progress bar click
  $('progressBar').addEventListener('click', (e) => {
    const rect = e.currentTarget.getBoundingClientRect();
    const pct = (e.clientX - rect.left) / rect.width;
    playbackIndex = Math.floor(pct * results.length);

    // Rebuild paths up to this point
    resetINSPath();
    resetFusedPath();
    resetCharts();
    for (let i = 0; i <= playbackIndex && i < results.length; i++) {
      const r = results[i];
      if (r.ins_lat && r.ins_lng) addINSPoint(r.ins_lat, r.ins_lng);
      if (r.fused_lat && r.fused_lng) addFusedPoint(r.fused_lat, r.fused_lng);
    }
    updateProgressUI();
    if (playbackIndex < results.length) {
      updateMetricsUI(results[playbackIndex]);
      panToVehicle(results[playbackIndex].fused_lat || results[playbackIndex].gt_lat,
                    results[playbackIndex].fused_lng || results[playbackIndex].gt_lng);
    }
  });

  // Speed selector
  document.querySelectorAll('.speed-btn').forEach(btn => {
    btn.addEventListener('click', () => {
      document.querySelectorAll('.speed-btn').forEach(b => b.classList.remove('active'));
      btn.classList.add('active');
      playbackSpeed = parseInt(btn.dataset.speed);
    });
  });

  // Outage controls
  $('outageStart').addEventListener('input', (e) => {
    $('outageStartLabel').textContent = `${e.target.value}%`;
  });

  $('outageDuration').addEventListener('input', (e) => {
    $('outageDurationLabel').textContent = `${e.target.value}%`;
  });

  $('btnApplyOutage').addEventListener('click', () => {
    $('outageInfo').style.display = 'flex';
    const startPct = parseInt($('outageStart').value);
    const durPct = parseInt($('outageDuration').value);
    $('outageInfoText').textContent = `GNSS outage: ${startPct}% - ${startPct + durPct}% of route`;
    runProcessing();
    startPlayback();
  });

  $('btnClearOutage').addEventListener('click', () => {
    $('outageInfo').style.display = 'none';
    clearOutageHighlight();
    fusionEngine.setOutage(-1);
    runProcessing();
  });

  // Algorithm parameter controls
  $('particleCount').addEventListener('input', (e) => {
    $('particleCountLabel').textContent = e.target.value;
  });

  $('dMinSlider').addEventListener('input', (e) => {
    $('dMinLabel').textContent = `${e.target.value} m`;
  });

  // Reprocess on parameter change (debounced)
  let reprocessTimeout = null;
  const reprocessOnChange = () => {
    clearTimeout(reprocessTimeout);
    reprocessTimeout = setTimeout(() => {
      runProcessing();
    }, 300);
  };

  $('particleCount').addEventListener('change', reprocessOnChange);
  $('dMinSlider').addEventListener('change', reprocessOnChange);
  $('toggleMapMatch').addEventListener('change', reprocessOnChange);
  $('toggleNHC').addEventListener('change', reprocessOnChange);
  $('togglePF').addEventListener('change', reprocessOnChange);

  // Keyboard shortcuts
  document.addEventListener('keydown', (e) => {
    if (e.code === 'Space') {
      e.preventDefault();
      if (isPlaying) stopPlayback();
      else startPlayback();
    }
    if (e.code === 'KeyR') {
      resetPlayback();
    }
  });
}

// === Start ===
document.addEventListener('DOMContentLoaded', init);
