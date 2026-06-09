/**
 * Velocity History Chart using Chart.js
 */

const VelChart = (() => {
  let chart = null;
  const MAX_POINTS = 80;
  const labels = Array(MAX_POINTS).fill('');
  const vvData = Array(MAX_POINTS).fill(0);
  const vhData = Array(MAX_POINTS).fill(0);

  function init() {
    const ctx = document.getElementById('vel-chart').getContext('2d');
    chart = new Chart(ctx, {
      type: 'line',
      data: {
        labels,
        datasets: [
          {
            label: 'Vertical',
            data: vvData,
            borderColor: '#3b82f6',
            backgroundColor: 'rgba(59,130,246,0.1)',
            borderWidth: 1.5,
            pointRadius: 0,
            tension: 0.4,
            fill: true
          },
          {
            label: 'Horizontal',
            data: vhData,
            borderColor: '#00dc8a',
            backgroundColor: 'rgba(0,220,138,0.08)',
            borderWidth: 1.5,
            pointRadius: 0,
            tension: 0.4,
            fill: true
          }
        ]
      },
      options: {
        responsive: true,
        maintainAspectRatio: false,
        animation: false,
        plugins: {
          legend: {
            labels: {
              color: 'rgba(200,232,216,0.5)',
              font: { size: 9, family: 'JetBrains Mono' },
              boxWidth: 10,
              padding: 8
            }
          },
          tooltip: { enabled: false }
        },
        scales: {
          x: { display: false },
          y: {
            grid:  { color: 'rgba(255,255,255,0.04)' },
            ticks: { color: 'rgba(200,232,216,0.4)', font: { size: 9 }, maxTicksLimit: 4 },
            border:{ color: 'transparent' }
          }
        }
      }
    });
  }

  function push(vv, vh) {
    vvData.shift(); vvData.push(Math.abs(vv));
    vhData.shift(); vhData.push(vh);
    if (chart) chart.update('none');
  }

  return { init, push };
})();
