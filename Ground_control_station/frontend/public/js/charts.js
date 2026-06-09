/**
 * charts.js  –  Real-time telemetry history charts (Chart.js)
 * Three rolling-window line charts: velocity, altitude, roll/pitch.
 * Call Charts.push(telemetry) on each incoming telemetry packet.
 */

const Charts = (() => {
  const MAX_POINTS = 60;   // 30 s at 2 Hz (matches server history rate)
  const LABELS     = Array(MAX_POINTS).fill('');

  // Chart.js defaults matching the GCS colour theme
  Chart.defaults.color             = 'rgba(184,212,192,0.4)';
  Chart.defaults.borderColor       = 'rgba(0,232,122,0.1)';
  Chart.defaults.font.family       = "'JetBrains Mono', monospace";
  Chart.defaults.font.size         = 9;
  Chart.defaults.animation.duration = 0;  // no animation for real-time

  const baseDataset = {
    borderWidth:         1.5,
    pointRadius:         0,
    tension:             0.35,
    fill:                true,
  };

  let velChart, altChart, attChart;

  function _makeLabels() { return Array(MAX_POINTS).fill(''); }

  function init() {
    velChart = _buildVelChart();
    altChart = _buildAltChart();
    attChart = _buildAttChart();
  }

  function _ctx(id) {
    return document.getElementById(id).getContext('2d');
  }

  function _scaleOpts(yMin, yMax, label) {
    return {
      x: { display: false, grid: { display: false } },
      y: {
        min: yMin,
        max: yMax,
        ticks: { maxTicksLimit: 4, font: { size: 8 } },
        grid:  { color: 'rgba(0,232,122,0.07)' },
        title: { display: !!label, text: label, font: { size: 8 } },
      },
    };
  }

  function _buildVelChart() {
    return new Chart(_ctx('vel-chart'), {
      type: 'line',
      data: {
        labels: _makeLabels(),
        datasets: [
          {
            ...baseDataset,
            label:           'Horizontal m/s',
            data:            Array(MAX_POINTS).fill(0),
            borderColor:     '#00e87a',
            backgroundColor: 'rgba(0,232,122,0.06)',
          },
          {
            ...baseDataset,
            label:           'Vertical m/s',
            data:            Array(MAX_POINTS).fill(0),
            borderColor:     '#3b82f6',
            backgroundColor: 'rgba(59,130,246,0.06)',
          },
        ],
      },
      options: {
        responsive: true,
        maintainAspectRatio: false,
        plugins: { legend: { display: true, position: 'top', labels: { boxWidth: 8, font: { size: 8 } } } },
        scales:  _scaleOpts(-5, 20, ''),
      },
    });
  }

  function _buildAltChart() {
    return new Chart(_ctx('alt-chart'), {
      type: 'line',
      data: {
        labels: _makeLabels(),
        datasets: [{
          ...baseDataset,
          label:           'Altitude m',
          data:            Array(MAX_POINTS).fill(0),
          borderColor:     '#ffb300',
          backgroundColor: 'rgba(255,179,0,0.07)',
        }],
      },
      options: {
        responsive: true,
        maintainAspectRatio: false,
        plugins: { legend: { display: false } },
        scales:  _scaleOpts(0, 100, ''),
      },
    });
  }

  function _buildAttChart() {
    return new Chart(_ctx('att-chart'), {
      type: 'line',
      data: {
        labels: _makeLabels(),
        datasets: [
          {
            ...baseDataset,
            label:           'Roll °',
            data:            Array(MAX_POINTS).fill(0),
            borderColor:     '#ff6b6b',
            backgroundColor: 'rgba(255,107,107,0.05)',
          },
          {
            ...baseDataset,
            label:           'Pitch °',
            data:            Array(MAX_POINTS).fill(0),
            borderColor:     '#a78bfa',
            backgroundColor: 'rgba(167,139,250,0.05)',
          },
        ],
      },
      options: {
        responsive: true,
        maintainAspectRatio: false,
        plugins: { legend: { display: true, position: 'top', labels: { boxWidth: 8, font: { size: 8 } } } },
        scales:  _scaleOpts(-45, 45, ''),
      },
    });
  }

  function _push1(chart, ...values) {
    values.forEach((v, i) => {
      const d = chart.data.datasets[i].data;
      d.push(parseFloat(v.toFixed(2)));
      if (d.length > MAX_POINTS) d.shift();
    });
    chart.data.labels.push('');
    if (chart.data.labels.length > MAX_POINTS) chart.data.labels.shift();
    chart.update('none');
  }

  /**
   * Load a batch of history points (called once after connect).
   * @param {Array} historyArr - array of history objects from server
   */
  function loadHistory(historyArr) {
    if (!velChart || !historyArr.length) return;

    // Reset all charts
    [velChart, altChart, attChart].forEach(c => {
      c.data.datasets.forEach(ds => { ds.data = []; });
      c.data.labels = [];
    });

    historyArr.forEach(h => {
      velChart.data.datasets[0].data.push(parseFloat((h.vel_h || 0).toFixed(2)));
      velChart.data.datasets[1].data.push(parseFloat((h.vel_v || 0).toFixed(2)));
      velChart.data.labels.push('');

      altChart.data.datasets[0].data.push(parseFloat((h.alt || 0).toFixed(1)));
      altChart.data.labels.push('');

      attChart.data.datasets[0].data.push(parseFloat((h.roll || 0).toFixed(1)));
      attChart.data.datasets[1].data.push(parseFloat((h.pitch || 0).toFixed(1)));
      attChart.data.labels.push('');
    });

    velChart.update('none');
    altChart.update('none');
    attChart.update('none');
  }

  /**
   * Push one telemetry sample to all charts.
   */
  function push(t) {
    if (!velChart) return;
    _push1(velChart, t.velocity.horizontal, t.velocity.vertical);
    _push1(altChart, t.position.z);
    _push1(attChart, t.attitude.roll, t.attitude.pitch);

    // Dynamically expand altitude axis
    const allAlt = altChart.data.datasets[0].data;
    const maxAlt = Math.max(...allAlt, 10);
    altChart.options.scales.y.max = Math.ceil(maxAlt * 1.2);
  }

  return { init, push, loadHistory };
})();
