/**
 * Leaflet Map Module
 * Shows drone position, flight path, and home marker
 */

const DroneMap = (() => {
  let map = null;
  let droneMarker = null;
  let homeMaker = null;
  let flightPath = null;
  let homeSet = false;
  let pathPoints = [];

  const DRONE_SVG = `
    <svg xmlns="http://www.w3.org/2000/svg" width="32" height="32" viewBox="0 0 32 32">
      <circle cx="16" cy="16" r="4" fill="#00dc8a"/>
      <line x1="0" y1="0" x2="10" y2="10" stroke="#00dc8a" stroke-width="1.5" stroke-linecap="round"/>
      <line x1="32" y1="0" x2="22" y2="10" stroke="#00dc8a" stroke-width="1.5" stroke-linecap="round"/>
      <line x1="0" y1="32" x2="10" y2="22" stroke="#00dc8a" stroke-width="1.5" stroke-linecap="round"/>
      <line x1="32" y1="32" x2="22" y2="22" stroke="#00dc8a" stroke-width="1.5" stroke-linecap="round"/>
      <circle cx="4" cy="4" r="3" fill="none" stroke="#00dc8a" stroke-width="1.2"/>
      <circle cx="28" cy="4" r="3" fill="none" stroke="#00dc8a" stroke-width="1.2"/>
      <circle cx="4" cy="28" r="3" fill="none" stroke="#00dc8a" stroke-width="1.2"/>
      <circle cx="28" cy="28" r="3" fill="none" stroke="#00dc8a" stroke-width="1.2"/>
    </svg>`;

  function init() {
    map = L.map('map', {
      center: [18.5204, 73.8567],
      zoom: 15,
      zoomControl: false,
      attributionControl: false
    });

    // Dark tile layer
    L.tileLayer('https://{s}.basemaps.cartocdn.com/dark_all/{z}/{x}/{y}{r}.png', {
      maxZoom: 20
    }).addTo(map);

    L.control.zoom({ position: 'bottomright' }).addTo(map);

    // Flight path polyline
    flightPath = L.polyline([], {
      color: '#00dc8a',
      weight: 2,
      opacity: 0.7,
      dashArray: '4 4'
    }).addTo(map);

    // Drone icon
    const droneIcon = L.divIcon({
      html: DRONE_SVG,
      className: '',
      iconSize: [32, 32],
      iconAnchor: [16, 16]
    });

    droneMarker = L.marker([18.5204, 73.8567], { icon: droneIcon }).addTo(map);
    droneMarker.bindPopup('<b>Drone</b>');
  }

  function update(lat, lon, alt, yaw) {
    if (!map || lat === 0 && lon === 0) return;
    const latlng = [lat, lon];

    droneMarker.setLatLng(latlng);

    // Rotate drone icon via CSS
    const el = droneMarker.getElement();
    if (el) el.style.transform += ` rotate(${yaw}deg)`;

    // Set home on first valid GPS
    if (!homeSet && lat !== 0) {
      homeSet = true;
      const homeIcon = L.divIcon({
        html: `<div style="background:#f59e0b;width:10px;height:10px;border-radius:50%;border:2px solid #fff"></div>`,
        className: '',
        iconSize: [10, 10],
        iconAnchor: [5, 5]
      });
      homeMaker = L.marker(latlng, { icon: homeIcon }).addTo(map);
      homeMaker.bindPopup('<b>Home</b>');
    }

    // Add to path (deduplicate nearby points)
    if (pathPoints.length === 0 || distance(pathPoints[pathPoints.length - 1], latlng) > 0.00002) {
      pathPoints.push(latlng);
      if (pathPoints.length > 500) pathPoints.shift();
      flightPath.setLatLngs(pathPoints);
    }
  }

  function centerMap() {
    if (droneMarker) map.setView(droneMarker.getLatLng(), map.getZoom());
  }

  function clearPath() {
    pathPoints = [];
    flightPath.setLatLngs([]);
  }

  function distance(a, b) {
    return Math.hypot(a[0] - b[0], a[1] - b[1]);
  }

  return { init, update, centerMap, clearPath };
})();
