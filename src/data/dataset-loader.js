/**
 * Dataset Loader
 * Loads and parses the pre-processed IO-VNBD JSON dataset
 */

/**
 * Load the sample route dataset
 * @returns {Promise<{metadata: Object, data: Array}>}
 */
export async function loadDataset(url = '/data/sample-route.json') {
  const response = await fetch(url);
  if (!response.ok) {
    throw new Error(`Failed to load dataset: ${response.statusText}`);
  }
  const json = await response.json();
  return json;
}

/**
 * Get a subset of the data between two indices
 */
export function getSubset(data, startIndex, endIndex) {
  return data.slice(startIndex, endIndex);
}

/**
 * Compute summary statistics for a dataset
 */
export function computeDatasetStats(data) {
  const lats = data.map(d => d.gt_lat);
  const lngs = data.map(d => d.gt_lng);
  const speeds = data.map(d => d.gt_speed || d.vehicle_speed || 0);

  const mean = arr => arr.reduce((a, b) => a + b, 0) / arr.length;
  const min = arr => Math.min(...arr);
  const max = arr => Math.max(...arr);

  return {
    numPoints: data.length,
    duration: data.length > 0 ? (data[data.length - 1].time - data[0].time) : 0,
    bounds: {
      minLat: min(lats),
      maxLat: max(lats),
      minLng: min(lngs),
      maxLng: max(lngs),
      centerLat: mean(lats),
      centerLng: mean(lngs),
    },
    speed: {
      mean: mean(speeds),
      max: max(speeds),
    }
  };
}
