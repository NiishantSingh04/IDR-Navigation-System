/**
 * Charts Module
 * Manages Chart.js instances for sensor data and error visualization
 */

import Chart from 'chart.js/auto';

let errorChart = null;
let accelChart = null;
let speedChart = null;

const MAX_CHART_POINTS = 200;

const CHART_COLORS = {
  insError: '#c62828',
  pfError: '#1a73e8',
  fusedError: '#e65100',
  accelX: '#ef5350',
  accelY: '#66bb6a',
  accelZ: '#42a5f5',
  speed: '#1a73e8',
  heading: '#ffa726',
};

const CHART_DEFAULTS = {
  responsive: true,
  maintainAspectRatio: false,
  animation: false,
  plugins: {
    legend: {
      display: true,
      position: 'top',
      align: 'end',
      labels: {
        color: '#9e9e9e',
        font: { family: 'Inter', size: 10, weight: '500' },
        boxWidth: 12,
        boxHeight: 2,
        padding: 8,
        usePointStyle: false,
      }
    },
    tooltip: {
      enabled: true,
      backgroundColor: 'rgba(17, 24, 39, 0.95)',
      titleFont: { family: 'Inter', size: 11 },
      bodyFont: { family: 'JetBrains Mono', size: 10 },
      borderColor: '#2d3a4e',
      borderWidth: 1,
      padding: 8,
      cornerRadius: 6,
    }
  },
  scales: {
    x: {
      display: true,
      grid: {
        color: 'rgba(45, 58, 78, 0.5)',
        drawBorder: false,
      },
      ticks: {
        color: '#757575',
        font: { family: 'JetBrains Mono', size: 9 },
        maxTicksLimit: 6,
      }
    },
    y: {
      display: true,
      grid: {
        color: 'rgba(45, 58, 78, 0.5)',
        drawBorder: false,
      },
      ticks: {
        color: '#757575',
        font: { family: 'JetBrains Mono', size: 9 },
        maxTicksLimit: 5,
      }
    }
  }
};

/**
 * Initialize all charts
 */
export function initCharts() {
  // Error chart
  const errorCtx = document.getElementById('chartError');
  if (errorCtx) {
    errorChart = new Chart(errorCtx, {
      type: 'line',
      data: {
        labels: [],
        datasets: [
          {
            label: 'INS Error',
            data: [],
            borderColor: CHART_COLORS.insError,
            backgroundColor: 'rgba(198, 40, 40, 0.1)',
            borderWidth: 1.5,
            pointRadius: 0,
            fill: true,
            tension: 0.3,
          },
          {
            label: 'RBPF Error',
            data: [],
            borderColor: CHART_COLORS.pfError,
            backgroundColor: 'rgba(26, 115, 232, 0.1)',
            borderWidth: 1.5,
            pointRadius: 0,
            fill: true,
            tension: 0.3,
          },
        ]
      },
      options: {
        ...CHART_DEFAULTS,
        scales: {
          ...CHART_DEFAULTS.scales,
          y: {
            ...CHART_DEFAULTS.scales.y,
            title: {
              display: true,
              text: 'Error (m)',
              color: '#757575',
              font: { family: 'Inter', size: 9 },
            },
            min: 0,
          }
        }
      }
    });
  }

  // Accelerometer chart
  const accelCtx = document.getElementById('chartAccel');
  if (accelCtx) {
    accelChart = new Chart(accelCtx, {
      type: 'line',
      data: {
        labels: [],
        datasets: [
          {
            label: 'X',
            data: [],
            borderColor: CHART_COLORS.accelX,
            borderWidth: 1.2,
            pointRadius: 0,
            tension: 0.2,
          },
          {
            label: 'Y',
            data: [],
            borderColor: CHART_COLORS.accelY,
            borderWidth: 1.2,
            pointRadius: 0,
            tension: 0.2,
          },
          {
            label: 'Z',
            data: [],
            borderColor: CHART_COLORS.accelZ,
            borderWidth: 1.2,
            pointRadius: 0,
            tension: 0.2,
          },
        ]
      },
      options: {
        ...CHART_DEFAULTS,
        scales: {
          ...CHART_DEFAULTS.scales,
          y: {
            ...CHART_DEFAULTS.scales.y,
            title: {
              display: true,
              text: 'm/s²',
              color: '#757575',
              font: { family: 'Inter', size: 9 },
            }
          }
        }
      }
    });
  }

  // Speed chart
  const speedCtx = document.getElementById('chartSpeed');
  if (speedCtx) {
    speedChart = new Chart(speedCtx, {
      type: 'line',
      data: {
        labels: [],
        datasets: [
          {
            label: 'Speed',
            data: [],
            borderColor: CHART_COLORS.speed,
            backgroundColor: 'rgba(26, 115, 232, 0.08)',
            borderWidth: 1.5,
            pointRadius: 0,
            fill: true,
            tension: 0.3,
            yAxisID: 'y',
          },
          {
            label: 'Heading',
            data: [],
            borderColor: CHART_COLORS.heading,
            borderWidth: 1.2,
            pointRadius: 0,
            tension: 0.2,
            yAxisID: 'y1',
          },
        ]
      },
      options: {
        ...CHART_DEFAULTS,
        scales: {
          x: CHART_DEFAULTS.scales.x,
          y: {
            ...CHART_DEFAULTS.scales.y,
            position: 'left',
            title: {
              display: true,
              text: 'km/h',
              color: '#757575',
              font: { family: 'Inter', size: 9 },
            },
            min: 0,
          },
          y1: {
            display: true,
            position: 'right',
            grid: { display: false },
            ticks: {
              color: '#757575',
              font: { family: 'JetBrains Mono', size: 9 },
              maxTicksLimit: 4,
            },
            title: {
              display: true,
              text: 'deg',
              color: '#757575',
              font: { family: 'Inter', size: 9 },
            }
          }
        }
      }
    });
  }
}

/**
 * Update charts with a new data point
 */
export function updateCharts(result, index) {
  const label = index.toString();

  if (errorChart) {
    const data = errorChart.data;
    data.labels.push(label);
    data.datasets[0].data.push(result.ins_error);
    data.datasets[1].data.push(result.pf_error);

    // Trim to max points
    if (data.labels.length > MAX_CHART_POINTS) {
      data.labels.shift();
      data.datasets.forEach(ds => ds.data.shift());
    }
    errorChart.update('none');
  }

  if (accelChart) {
    const data = accelChart.data;
    data.labels.push(label);
    data.datasets[0].data.push(result.accel_x);
    data.datasets[1].data.push(result.accel_y);
    data.datasets[2].data.push(result.accel_z);

    if (data.labels.length > MAX_CHART_POINTS) {
      data.labels.shift();
      data.datasets.forEach(ds => ds.data.shift());
    }
    accelChart.update('none');
  }

  if (speedChart) {
    const data = speedChart.data;
    data.labels.push(label);
    data.datasets[0].data.push(result.speed);
    data.datasets[1].data.push(result.heading);

    if (data.labels.length > MAX_CHART_POINTS) {
      data.labels.shift();
      data.datasets.forEach(ds => ds.data.shift());
    }
    speedChart.update('none');
  }
}

/**
 * Reset all charts
 */
export function resetCharts() {
  [errorChart, accelChart, speedChart].forEach(chart => {
    if (chart) {
      chart.data.labels = [];
      chart.data.datasets.forEach(ds => { ds.data = []; });
      chart.update('none');
    }
  });
}
