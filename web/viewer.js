/* Browser presentation only: recognition and distance calculations live in Python. */
const REFRESH_INTERVAL_MS = 250;
const PREVIEW_INTERVAL_MS = 60;
const STALE_AFTER_MS = 5000;
let previewEnabled = false;
const element = (id) => document.getElementById(id);

function setText(id, value) {
  element(id).textContent = value;
}

function createText(tag, text, className) {
  const node = document.createElement(tag);
  node.textContent = text;
  if (className) node.className = className;
  return node;
}

function showConnection(connected, message) {
  setText('connectionStatus', message);
  element('connectionStatus').classList.toggle('disconnected', !connected);
}

function showDetectionCards(detections, classes) {
  const counts = element('classCounts');
  counts.replaceChildren();
  counts.append(createText('span', `${classes.length} outdoor categories enabled`));
  const mobileCount = detections.filter((item) => item.motion_group === 'potentially_dynamic').length;
  const staticCount = detections.filter((item) => item.motion_group === 'usually_static').length;
  counts.append(createText('span', `Could move: ${mobileCount}`), createText('span', `Usually stationary: ${staticCount}`));
  for (const label of classes) {
    const count = detections.filter((item) => item.label === label).length;
    if (!count) continue; // Do not crowd the panel with dozens of zero counts.
    const displayName = {
      person: 'People',
      car: 'Cars',
      tree: 'Trees'
    } [label] || label;
    counts.append(createText('span', `${displayName}  ${count}`));
  }
  const container = element('detectionCards');
  // Preserve expanded explanations while new frames arrive.
  const expanded = new Set([...container.querySelectorAll('article')]
    .filter((card) => card.querySelector('details')?.open)
    .map((card) => card.dataset.key));
  container.replaceChildren();
  if (!detections.length) {
    const empty = createText('div', '', 'empty-state');
    empty.append(createText('span', '◎'), createText('h3', 'No target objects in view'),
      createText('p', 'Point the camera toward a target object. Recognized objects appear here.'));
    container.append(empty);
    return;
  }
  const groupContainers = {};
  for (const [key, title] of [['potentially_dynamic', 'Could move'], ['usually_static', 'Usually stationary']]) {
    const count = detections.filter((item) => item.motion_group === key).length;
    if (!count) continue;
    const group = createText('section', '', 'mobility-section');
    group.append(createText('h3', `${title} · ${count}`, 'mobility-heading'));
    container.append(group);
    groupContainers[key] = group;
  }
  detections.forEach((item, index) => {
    const card = createText('article', '', 'object-card');
    card.dataset.motionGroup = item.motion_group || 'unknown';
    card.dataset.key = `${item.label}-${index}`;
    const heading = document.createElement('header');
    heading.append(createText('h3', item.label),
      createText('span', item.observation_status === 'prediction_only' ? 'Predicted track · no fresh detection' : `${Math.round(item.confidence * 100)}% confidence`, 'confidence'));
    const distance = item.distance_m === null ? 'Distance unavailable' : `≈ ${item.distance_m.toFixed(1)} m`;
    const motion = item.motion || {};
    const speed = Number.isFinite(motion.object_speed_mps) ? `~${motion.object_speed_mps.toFixed(2)} m/s estimated` :
      Number.isFinite(motion.image_speed_px_s) ? `${motion.image_speed_px_s.toFixed(0)} px/s; metric speed unknown` : 'Collecting motion history';
    const arrival = Number.isFinite(motion.front_zone_entry_s) ? (motion.front_zone_entry_s === 0 ? 'Already in front zone' : `Front-zone arrival ~${motion.front_zone_entry_s.toFixed(1)} s`) : 'Front-zone arrival unknown';
    const notes = document.createElement('details');
    notes.open = expanded.has(card.dataset.key);
    notes.append(createText('summary', 'Why this distance?'), createText('p', item.distance_note));
    card.append(heading, createText('p', distance, 'object-distance'),
      createText('p', `${item.horizontal_position} of image · ${item.object_category || 'Outdoor object'}`, 'object-position'),
      createText('p', `Track #${motion.track_id ?? '?'} · ${speed}`, 'object-position'), createText('p', arrival, 'object-position'), notes);
    (groupContainers[item.motion_group] || container).append(card);
  });
}

function showDistanceGuide(frame) {
  const mode = frame.distance_mode || 'size';
  const settings = frame.distance_assumptions || {};
  const calibrated = Boolean(settings.focal_calibration?.focal_px);
  setText('distanceMode', {
    size: 'Object size',
    ground: 'Ground plane',
    off: 'Off'
  } [mode] || mode);
  setText('calibrationStatus', calibrated ? 'Focal length calibrated' : 'Camera calibration needed');
  setText('distanceModeTag', `${mode.toUpperCase()} MODE`);
  const descriptions = {
    size: ['A rough estimate from object size', 'The image height of a fully visible object is compared with its assumed real height. Partial bodies, unusual sizes and camera tilt can make this inaccurate. Trees have no default size.'],
    ground: ['An estimate from the ground plane', 'Uses camera height, downward angle and the visible base of the object. Level ground is assumed. Occluded feet, hidden tree bases and uneven ground can invalidate the estimate.'],
    off: ['Distance estimation is switched off', 'Objects are still recognized, but this session does not calculate distances.']
  };
  const [title, description] = descriptions[mode] || descriptions.size;
  setText('distanceTitle', title);
  setText('distanceExplanation', description);
  const heights = Object.entries(settings.assumed_object_heights_m || {}).map(([label, height]) => `${label}: ${height} m`);
  setText('sizeAssumptions', mode === 'size' ? `Assumed height → ${heights.join(' / ')}` :
    mode === 'ground' ? `Camera height: ${settings.camera_height_m ?? 'not set'} m / Tilt: ${settings.camera_downward_tilt_degrees ?? 'not set'}°` : '');
}

function clearStaleDisplay(message) {
  previewEnabled = false;
  showConnection(false, 'Camera offline');
  element('cameraPreview').hidden = true;
  element('cameraPreview').removeAttribute('src');
  element('cameraPlaceholder').hidden = false;
  setText('cameraPlaceholder', message);
  for (const id of ['objectCount', 'nearestDistance', 'analysisTime']) setText(id, '—');
  setText('objectBadge', '—');
  setText('rawOutput', 'No current observations.');
  setText('lastUpdate', 'Waiting for camera');
  element('classCounts').replaceChildren();
  const empty = createText('div', '', 'empty-state');
  empty.append(createText('span', '◎'), createText('h3', 'Waiting for the camera'), createText('p', message));
  element('detectionCards').replaceChildren(empty);
}

async function refreshDetections() {
  try {
    const response = await fetch('/api/detections', {
      cache: 'no-store'
    });
    if (!response.ok) throw new Error('Detector unavailable');
    const frame = await response.json();
    showDistanceGuide(frame);
    if (!frame.camera_active || Date.now() - Date.parse(frame.timestamp_utc) > STALE_AFTER_MS) {
      throw new Error('No fresh camera observations');
    }
    if (frame.camera_health?.alert) showConnection(false, 'Camera view unusable');
    else showConnection(true, 'Camera live');
    const detections = frame.detections || [];
    setText('objectCount', detections.length);
    setText('objectBadge', detections.length);
    const ranges = detections.map((item) => item.distance_m).filter(Number.isFinite);
    setText('nearestDistance', ranges.length ? `≈ ${Math.min(...ranges).toFixed(1)} m` : '—');
    setText('analysisTime', `${Math.round(frame.inference_ms)} ms`);
    setText('resolution', `${frame.image_width} × ${frame.image_height}`);
    setText('cameraDetails', `${frame.source?.startsWith('webots') ? 'Webots camera' : 'Webcam'} · ${frame.image_width} × ${frame.image_height}`);
    setText('classDetails', `${(frame.classes || []).length} outdoor categories`);
    setText('frameNumber', `Frame ${frame.frame_id.toLocaleString()}`);
    setText('lastUpdate', `Updated ${new Date(frame.timestamp_utc).toLocaleTimeString()}`);
    showDetectionCards(detections, frame.classes || []);
    setText('rawOutput', JSON.stringify(frame, null, 2));
    previewEnabled = true;
  } catch (error) {
    clearStaleDisplay('Turn on your webcam and start the detector when you are ready.');
  }
  // Wait for completion before scheduling another poll; never queue requests.
  setTimeout(refreshDetections, REFRESH_INTERVAL_MS);
}
refreshDetections();

// Preview updates independently of slower text/card updates. Never queue image
// requests: wait for each image to load before fetching another. This removes
// the old 2.5 FPS display cap without claiming extra model detections.
async function refreshPreview() {
  if (previewEnabled) {
    const next = new Image();
    const loaded = await new Promise((resolve) => {
      next.onload = () => resolve(true);
      next.onerror = () => resolve(false);
      next.src = `/frame.jpg?t=${Date.now()}`;
    });
    if (loaded && previewEnabled) {
      element('cameraPreview').src = next.src;
      element('cameraPreview').hidden = false;
      element('cameraPlaceholder').hidden = true;
    }
  }
  setTimeout(refreshPreview, PREVIEW_INTERVAL_MS);
}
refreshPreview();
