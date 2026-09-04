// robot_console HTML遠隔観測UI（閲覧専用）。
//
// /snapshot.json をポーリングして状態サマリ・GPS・センサ一覧・鮮度一覧・地図
// マーカーを更新し、/images/{panel_id} を別周期でポーリングして画像を更新する。
// 本ページは観測専用であり、書き込み系のfetch（POST/PUT/DELETE）は行わない。
// ページタイトル脇には常にsnapshot取得の最終更新時刻と経過秒数を端的に表示し、
// 古い値を現在値と誤認させないようにする（詳細メッセージや色分けによる強調はしない）。
// 地図はLeaflet + OpenStreetMapタイルを使うため、閲覧時にインターネット接続が
// 必要（robot_console_ui_renewal_input.md 8.4節 候補A）。waypoint列は
// snapshot.route.waypoints（緯度経度を持つもののみ）から描画し、
// snapshot.route.current_index を境に走行済み/未走行を色分けする。

const SNAPSHOT_POLL_MS = 1000;
const IMAGE_POLL_MS = 1500;

// 鮮度表示の再評価周期。fetchが長時間ハングしても表示が更新されるよう、
// ポーリングの成否とは独立したタイマーで判定する。
const FRESHNESS_INDICATOR_CHECK_MS = 500;
// 1回のsnapshot取得に許す最大時間。上限が無いと、経路の切断（VPNのNATタイムアウト等）で
// fetchが解決も棄却もしないまま滞留し、次回ポーリングが再スケジュールされずに
// 画面が無言で固まる。
const SNAPSHOT_FETCH_TIMEOUT_MS = 5000;

const DEFAULT_LATITUDE = 36.083;
const DEFAULT_LONGITUDE = 140.113;
const DEFAULT_ZOOM = 18;

const FRESHNESS_COLORS = {
  OK: '#2e7d32',
  STALE: '#f9a825',
  LOST: '#c62828',
  UNKNOWN: '#757575',
};

let lastSnapshotSuccessAt = null;
let snapshotFailureCount = 0;
let lastSnapshotErrorText = '';

let knownPanelIds = [];
let leafletMap = null;
let currentPositionMarker = null;
let targetPositionMarker = null;
let hasCenteredMap = false;
let hasFitRouteBounds = false;
let routeWaypointMarkers = [];
let routeTraveledPolyline = null;
let routeUntraveledPolyline = null;
let knownRouteWaypointCount = -1;

const ROUTE_TRAVELED_COLOR = '#757575';
const ROUTE_UNTRAVELED_COLOR = '#66bb6a';

// EventBanner.severity の表示色（ui_qt/widgets/color_rules.py と同じ配色）。
const SEVERITY_COLORS = {
  error: '#c62828',
  warn: '#f9a825',
  notice: '#1565c0',
  info: '#2e7d32',
};

function severityColor(severity) {
  return SEVERITY_COLORS[severity] || FRESHNESS_COLORS.UNKNOWN;
}

function freshnessColor(level) {
  return FRESHNESS_COLORS[level] || FRESHNESS_COLORS.UNKNOWN;
}

// GPS未測位時などサーバ側の値がnullになり得るため、そのままtoFixed()を呼ばない。
// null相手にTypeErrorを投げると描画全体が中断し、画面が無言で固まる。
function formatNumber(value, digits, unit) {
  if (typeof value !== 'number' || !Number.isFinite(value)) {
    return '-';
  }
  return `${value.toFixed(digits)}${unit}`;
}

function initMap() {
  leafletMap = L.map('leaflet-map').setView([DEFAULT_LATITUDE, DEFAULT_LONGITUDE], DEFAULT_ZOOM);
  L.tileLayer('https://{s}.tile.openstreetmap.org/{z}/{x}/{y}.png', {
    maxZoom: 19,
    attribution: '&copy; OpenStreetMap contributors',
  }).addTo(leafletMap);

  // グリッドレイアウト内では初期化時にコンテナの実寸がまだ確定していないことが
  // あり、その場合Leafletがタイルを1枚も要求しない。少し遅らせてサイズを
  // 再計算させることで確実にタイルを読み込ませる。
  setTimeout(() => leafletMap.invalidateSize(), 0);
  window.addEventListener('resize', () => leafletMap.invalidateSize());
}

function updateMapMarkers(snapshot) {
  if (leafletMap === null) {
    return;
  }

  const current = snapshot.localization;
  if (current.latitude !== null && current.longitude !== null) {
    const latlng = [current.latitude, current.longitude];
    if (currentPositionMarker === null) {
      currentPositionMarker = L.circleMarker(latlng, {
        radius: 5,
        color: '#4fc3f7',
        fillColor: '#4fc3f7',
        fillOpacity: 0.9,
      })
        .bindTooltip('現在位置')
        .addTo(leafletMap);
    } else {
      currentPositionMarker.setLatLng(latlng);
    }
    if (!hasCenteredMap) {
      leafletMap.setView(latlng, DEFAULT_ZOOM);
      hasCenteredMap = true;
    }
  }

  const target = snapshot.target;
  if (target.latitude !== null && target.longitude !== null) {
    const latlng = [target.latitude, target.longitude];
    if (targetPositionMarker === null) {
      targetPositionMarker = L.circleMarker(latlng, {
        radius: 4,
        color: '#f9a825',
        fillColor: '#f9a825',
        fillOpacity: 0.9,
      })
        .bindTooltip('目標waypoint')
        .addTo(leafletMap);
    } else {
      targetPositionMarker.setLatLng(latlng);
    }
  }
}

function updateRouteOverlay(snapshot) {
  if (leafletMap === null) {
    return;
  }

  const route = snapshot.route || {};
  // 走行済み点数はCore側が決める（完走時は current_index が最終waypointのindexで
  // 止まるため、indexだけで色分けすると最後の1点が未走行のまま残る）。
  const traveledCount =
    typeof route.traveled_waypoint_count === 'number'
      ? route.traveled_waypoint_count
      : route.current_index || 0;
  const validWaypoints = (route.waypoints || []).filter(
    (waypoint) => waypoint.latitude !== null && waypoint.longitude !== null,
  );

  // waypoint数が変わった場合のみマーカーを作り直す。緯度経度・走行状態の
  // 更新は既存マーカーのsetLatLng/setStyleで行い、DOM再生成を避ける。
  if (validWaypoints.length !== knownRouteWaypointCount) {
    for (const marker of routeWaypointMarkers) {
      leafletMap.removeLayer(marker);
    }
    routeWaypointMarkers = validWaypoints.map((waypoint) =>
      L.circleMarker([waypoint.latitude, waypoint.longitude], {
        radius: 3,
        weight: 1,
        fillOpacity: 0.9,
      }).addTo(leafletMap),
    );
    knownRouteWaypointCount = validWaypoints.length;

    // 自己位置（pose_enu）を未受信の間は地図がDEFAULT_LATITUDE/LONGITUDEの
    // 初期表示のまま動かず、waypointが描画されていても実際のroute位置が
    // 画面外になり続ける（manual_start前はrobot_simulatorが自己位置を出力
    // しないため、この状態が長時間続き得る）。初めてwaypointを受け取った
    // 時点で一度だけroute全体が収まるよう地図をfitさせ、以後は自己位置側の
    // 自動センタリング（updateMapMarkers）やユーザー操作を優先して上書きしない。
    if (validWaypoints.length > 0 && !hasFitRouteBounds && !hasCenteredMap) {
      const routeBounds = validWaypoints.map((waypoint) => [waypoint.latitude, waypoint.longitude]);
      leafletMap.fitBounds(routeBounds, { padding: [24, 24] });
      hasFitRouteBounds = true;
    }
  }

  for (let i = 0; i < validWaypoints.length; i += 1) {
    const waypoint = validWaypoints[i];
    const traveled = waypoint.index < traveledCount;
    const color = traveled ? ROUTE_TRAVELED_COLOR : ROUTE_UNTRAVELED_COLOR;
    routeWaypointMarkers[i].setLatLng([waypoint.latitude, waypoint.longitude]);
    routeWaypointMarkers[i].setStyle({ color, fillColor: color });
  }

  if (routeTraveledPolyline === null) {
    routeTraveledPolyline = L.polyline([], {
      color: ROUTE_TRAVELED_COLOR,
      weight: 3,
    }).addTo(leafletMap);
    routeUntraveledPolyline = L.polyline([], {
      color: ROUTE_UNTRAVELED_COLOR,
      weight: 3,
    }).addTo(leafletMap);
  }

  // 走行済み線と未走行線は現在追従中のwaypointで接続する（両方に含める）。
  // 完走時は traveledCount が総数になり、未走行線は空になる。
  const traveledLatLngs = validWaypoints
    .filter((waypoint) => waypoint.index <= traveledCount)
    .map((waypoint) => [waypoint.latitude, waypoint.longitude]);
  const untraveledLatLngs = validWaypoints
    .filter((waypoint) => waypoint.index >= traveledCount)
    .map((waypoint) => [waypoint.latitude, waypoint.longitude]);
  routeTraveledPolyline.setLatLngs(traveledLatLngs);
  routeUntraveledPolyline.setLatLngs(untraveledLatLngs);
}

function appendField(dl, label, value) {
  const dt = document.createElement('dt');
  dt.textContent = label;
  const dd = document.createElement('dd');
  dd.textContent = value;
  dl.appendChild(dt);
  dl.appendChild(dd);
}

function renderSummary(snapshot) {
  const fields = document.getElementById('summary-fields');
  fields.innerHTML = '';
  appendField(fields, '運行フェーズ', snapshot.operation.phase);
  appendField(
    fields,
    '業務モード',
    `${snapshot.operation.environment} / ${snapshot.operation.drive_mode}`,
  );
  const routeProgress = snapshot.operation.route_progress;
  appendField(
    fields,
    '進捗',
    typeof routeProgress === 'number' ? formatNumber(routeProgress * 100, 1, '%') : '-',
  );
  appendField(
    fields,
    'WP',
    `${snapshot.operation.current_waypoint || '-'} -> ${snapshot.operation.next_waypoint || '-'}`,
  );
  appendField(fields, 'manual_start', String(snapshot.operation.manual_start));
  if (snapshot.operation.pause_reason) {
    appendField(fields, '停止理由', snapshot.operation.pause_reason);
  }
  appendField(fields, 'route_follower', snapshot.follower.state);
  const drive = snapshot.drive;
  appendField(fields, 'drive mode', `${drive.mode} / ${drive.output_source || '-'}`);
  appendField(
    fields,
    'cmd_vel',
    `${formatNumber(drive.cmd_vel_linear_mps, 2, ' m/s')} / ` +
      `${formatNumber(drive.cmd_vel_angular_dps, 1, ' deg/s')} (${drive.cmd_vel_freshness})`,
  );
  appendField(fields, 'localization source', snapshot.localization.source);
}

function renderEvents(events) {
  const list = document.getElementById('event-list');
  list.innerHTML = '';
  if (!events || events.length === 0) {
    const li = document.createElement('li');
    li.className = 'event-empty';
    li.textContent = 'イベントなし';
    list.appendChild(li);
    return;
  }
  // 並び順はサーバ側で優先度順に整列済みのため、ここでは変更しない。
  for (const event of events) {
    const li = document.createElement('li');
    const message = document.createElement('span');
    message.textContent = event.message;
    message.style.color = severityColor(event.severity);
    li.appendChild(message);
    if (event.source) {
      const source = document.createElement('span');
      source.className = 'event-source';
      source.textContent = event.source;
      li.appendChild(source);
    }
    list.appendChild(li);
  }
}

function renderGpsSummary(snapshot) {
  const fields = document.getElementById('gps-fields');
  fields.innerHTML = '';
  appendField(fields, 'RTK', snapshot.gps.rtk_state);
  appendField(fields, 'Satellites', `${snapshot.gps.num_satellites} sat`);
  appendField(fields, 'HDOP', formatNumber(snapshot.gps.hdop, 2, ''));
  appendField(fields, 'Correction', formatNumber(snapshot.gps.correction_age_s, 2, ' s'));
  appendField(fields, 'Heading', formatNumber(snapshot.gps.heading_deg, 1, ' deg'));
  appendField(fields, 'Localization freshness', snapshot.localization.freshness);
}

function renderSensorGrid(allPanels) {
  // route_mapは専用のLeaflet地図で表示するため、センサ・画像パネルの
  // グリッドには含めない（PyQt5側のlocalization_sensor_tab.pyと同様の扱い）。
  const panels = allPanels.filter((panel) => panel.panel_id !== 'route_map');
  const grid = document.getElementById('sensor-grid');
  const panelIds = panels.map((panel) => panel.panel_id);
  const sameOrder =
    panelIds.length === knownPanelIds.length &&
    panelIds.every((id, index) => id === knownPanelIds[index]);

  if (!sameOrder) {
    grid.innerHTML = '';
    for (const panel of panels) {
      const card = document.createElement('figure');
      card.className = 'sensor-card';

      const img = document.createElement('img');
      img.id = `panel-image-${panel.panel_id}`;
      img.alt = panel.title;

      const caption = document.createElement('figcaption');
      caption.id = `panel-caption-${panel.panel_id}`;

      card.appendChild(img);
      card.appendChild(caption);
      grid.appendChild(card);
    }
    knownPanelIds = panelIds;
  }

  for (const panel of panels) {
    const caption = document.getElementById(`panel-caption-${panel.panel_id}`);
    if (caption === null) {
      continue;
    }
    caption.textContent = `${panel.title} / ${panel.topic || '-'} / ${panel.freshness}`;
    caption.style.color = freshnessColor(panel.freshness);
  }
}

function renderHealthTable(profiles) {
  const tbody = document.querySelector('#health-table tbody');
  tbody.innerHTML = '';
  for (const item of profiles) {
    const row = document.createElement('tr');

    const profileCell = document.createElement('td');
    profileCell.textContent = item.profile_id;
    const categoryCell = document.createElement('td');
    categoryCell.textContent = item.category;
    const statusCell = document.createElement('td');
    statusCell.textContent = item.status;
    const healthCell = document.createElement('td');
    healthCell.textContent = item.health;
    healthCell.style.color = freshnessColor(item.health);

    row.appendChild(profileCell);
    row.appendChild(categoryCell);
    row.appendChild(statusCell);
    row.appendChild(healthCell);
    tbody.appendChild(row);
  }
}

function renderMapCaption(snapshot) {
  const caption = document.getElementById('map-caption');
  const source = snapshot.localization.source;
  const freshness = snapshot.localization.freshness;
  caption.textContent = `source: ${source} / freshness: ${freshness}`;
  caption.style.color = freshnessColor(freshness);
}

// yyyy/mm/dd HH:MM:SS 形式の日時文字列を返す。画面幅が狭い場合は年月日を省略し、
// HH:MM:SS のみを返す（ヘッダー右端の限られた幅に収めるため）。
function formatTimestamp(date) {
  const pad = (value) => String(value).padStart(2, '0');
  const timePart = `${pad(date.getHours())}:${pad(date.getMinutes())}:${pad(date.getSeconds())}`;
  if (!window.matchMedia('(min-width: 768px)').matches) {
    return timePart;
  }
  const datePart = `${date.getFullYear()}/${pad(date.getMonth() + 1)}/${pad(date.getDate())}`;
  return `${datePart} ${timePart}`;
}

// ページタイトル脇に、最終更新時刻と経過秒数だけを端的に表示する。
// 詳細な文章や色分けによる強調はせず、閲覧者が一目で「いつの値か」を確認できれば十分とする。
function renderFreshnessIndicator() {
  const indicator = document.getElementById('freshness-indicator');
  if (indicator === null) {
    return;
  }

  if (lastSnapshotSuccessAt === null) {
    indicator.textContent = '未受信';
    return;
  }

  const elapsedSec = (Date.now() - lastSnapshotSuccessAt) / 1000;
  indicator.textContent =
    `${formatTimestamp(new Date(lastSnapshotSuccessAt))} (応答${elapsedSec.toFixed(1)}s)`;
}

async function fetchSnapshot() {
  // fetchはタイムアウトを持たないため、AbortControllerで上限を設ける。
  const controller = new AbortController();
  const timeoutId = setTimeout(() => controller.abort(), SNAPSHOT_FETCH_TIMEOUT_MS);
  try {
    const response = await fetch('/snapshot.json', {
      cache: 'no-store',
      signal: controller.signal,
    });
    if (!response.ok) {
      throw new Error(`HTTP ${response.status}`);
    }
    return await response.json();
  } finally {
    clearTimeout(timeoutId);
  }
}

async function pollSnapshot() {
  try {
    const snapshot = await fetchSnapshot();
    // 描画までを1つの成功単位として扱う。描画途中の例外を握り潰すと、
    // 通信は成功しているのに画面だけが古いまま固まる状態を検知できない。
    renderSummary(snapshot);
    renderEvents(snapshot.events);
    renderGpsSummary(snapshot);
    renderMapCaption(snapshot);
    updateRouteOverlay(snapshot);
    updateMapMarkers(snapshot);
    renderSensorGrid(snapshot.sensor_panels);
    renderHealthTable(snapshot.health);
    lastSnapshotSuccessAt = Date.now();
    snapshotFailureCount = 0;
    lastSnapshotErrorText = '';
  } catch (error) {
    snapshotFailureCount += 1;
    lastSnapshotErrorText = `${error.name}: ${error.message}`;
    console.error('snapshotの取得または描画に失敗しました', error);
  } finally {
    renderFreshnessIndicator();
    setTimeout(pollSnapshot, SNAPSHOT_POLL_MS);
  }
}

function pollFreshnessIndicator() {
  renderFreshnessIndicator();
  setTimeout(pollFreshnessIndicator, FRESHNESS_INDICATOR_CHECK_MS);
}

function pollImages() {
  const cacheBuster = Date.now();
  for (const panelId of knownPanelIds) {
    const img = document.getElementById(`panel-image-${panelId}`);
    if (img !== null) {
      img.src = `/images/${panelId}?t=${cacheBuster}`;
    }
  }
  setTimeout(pollImages, IMAGE_POLL_MS);
}

initMap();
pollSnapshot();
pollFreshnessIndicator();
pollImages();
