/**
 * Warehouse Robot Web Dashboard -- roslibjs subscriber logic.
 *
 * Connects to rosbridge_websocket and subscribes to:
 *   /battery_state    (sensor_msgs/BatteryState)
 *   /current_task     (warehouse_interfaces/DeliveryOrder)
 *   /task_queue       (warehouse_interfaces/TaskQueue)
 *   /tf               (for robot pose)
 *
 * Also renders a 2D warehouse map on canvas.
 */

// ---------- Configuration ---------------------------------------------------
const ROSBRIDGE_URL = `ws://${window.location.hostname || 'localhost'}:9090`;

// Warehouse dimensions (meters) - matches warehouse.world
const WORLD = {
  xMin: -10, xMax: 10,
  yMin: -8, yMax: 8,
};

// Station positions (from stations.yaml)
const STATIONS = {
  pickup_A:    { x: -8.0, y: -3.0, type: 'pickup' },
  pickup_B:    { x: -8.0, y:  3.0, type: 'pickup' },
  delivery_D1: { x:  8.0, y: -3.0, type: 'delivery' },
  delivery_D2: { x:  8.0, y:  3.0, type: 'delivery' },
  charging:    { x:  8.5, y:  6.0, type: 'charging' },
};

// Shelf positions (from warehouse.world)
const SHELVES = [
  { x: -4, y: -4 }, { x: 0, y: -4 }, { x: 4, y: -4 },
  { x: -4, y: -1.5 }, { x: 0, y: -1.5 }, { x: 4, y: -1.5 },
  { x: -4, y: 1.5 }, { x: 0, y: 1.5 }, { x: 4, y: 1.5 },
  { x: -4, y: 4 }, { x: 0, y: 4 }, { x: 4, y: 4 },
];

// ---------- DOM references --------------------------------------------------
const elConnStatus  = document.getElementById('connection-status');
const elPoseX       = document.getElementById('pose-x');
const elPoseY       = document.getElementById('pose-y');
const elPoseYaw     = document.getElementById('pose-yaw');
const elBatteryBar  = document.getElementById('battery-bar');
const elBatteryPct  = document.getElementById('battery-pct');
const elBatteryVolt = document.getElementById('battery-volt');
const elBatStatus   = document.getElementById('battery-status');
const elTaskContent = document.getElementById('current-task-content');
const elQueueBody   = document.getElementById('queue-body');
const canvas        = document.getElementById('warehouse-map');
const ctx           = canvas.getContext('2d');

// ---------- Robot state -----------------------------------------------------
let robotPose = { x: 0, y: -6, yaw: 1.5708 }; // spawn position
let currentTask = null;

// ---------- Canvas coordinate transform ------------------------------------
function worldToCanvas(wx, wy) {
  const px = ((wx - WORLD.xMin) / (WORLD.xMax - WORLD.xMin)) * canvas.width;
  const py = ((WORLD.yMax - wy) / (WORLD.yMax - WORLD.yMin)) * canvas.height;
  return { x: px, y: py };
}

function metersToPixels(m) {
  return (m / (WORLD.xMax - WORLD.xMin)) * canvas.width;
}

// ---------- Draw warehouse map ---------------------------------------------
function drawMap() {
  // Background
  ctx.fillStyle = '#1a1a1a';
  ctx.fillRect(0, 0, canvas.width, canvas.height);

  // Grid
  ctx.strokeStyle = '#2a2a3a';
  ctx.lineWidth = 0.5;
  for (let x = WORLD.xMin; x <= WORLD.xMax; x += 2) {
    const p = worldToCanvas(x, 0);
    ctx.beginPath();
    ctx.moveTo(p.x, 0);
    ctx.lineTo(p.x, canvas.height);
    ctx.stroke();
  }
  for (let y = WORLD.yMin; y <= WORLD.yMax; y += 2) {
    const p = worldToCanvas(0, y);
    ctx.beginPath();
    ctx.moveTo(0, p.y);
    ctx.lineTo(canvas.width, p.y);
    ctx.stroke();
  }

  // Shelves
  const shelfW = metersToPixels(2.5);
  const shelfH = metersToPixels(0.8);
  ctx.fillStyle = '#5c4033';
  ctx.strokeStyle = '#8b6914';
  ctx.lineWidth = 1;
  for (const shelf of SHELVES) {
    const p = worldToCanvas(shelf.x, shelf.y);
    ctx.fillRect(p.x - shelfW/2, p.y - shelfH/2, shelfW, shelfH);
    ctx.strokeRect(p.x - shelfW/2, p.y - shelfH/2, shelfW, shelfH);
  }

  // Stations
  for (const [name, st] of Object.entries(STATIONS)) {
    const p = worldToCanvas(st.x, st.y);
    const r = metersToPixels(0.6);

    ctx.beginPath();
    ctx.arc(p.x, p.y, r, 0, Math.PI * 2);

    if (st.type === 'pickup') {
      ctx.fillStyle = 'rgba(76, 175, 80, 0.3)';
      ctx.strokeStyle = '#4caf50';
    } else if (st.type === 'delivery') {
      ctx.fillStyle = 'rgba(33, 150, 243, 0.3)';
      ctx.strokeStyle = '#2196f3';
    } else {
      ctx.fillStyle = 'rgba(255, 202, 40, 0.3)';
      ctx.strokeStyle = '#ffca28';
    }
    ctx.lineWidth = 2;
    ctx.fill();
    ctx.stroke();

    // Label
    ctx.fillStyle = '#eee';
    ctx.font = '11px sans-serif';
    ctx.textAlign = 'center';
    const label = name.replace('pickup_', 'P-').replace('delivery_', 'D-').replace('charging', 'CHG');
    ctx.fillText(label, p.x, p.y + r + 14);
  }

  // Robot
  const rp = worldToCanvas(robotPose.x, robotPose.y);
  const robotR = metersToPixels(0.4);

  // Robot body
  ctx.beginPath();
  ctx.arc(rp.x, rp.y, robotR, 0, Math.PI * 2);
  ctx.fillStyle = '#e94560';
  ctx.fill();
  ctx.strokeStyle = '#ff6b81';
  ctx.lineWidth = 2;
  ctx.stroke();

  // Robot heading arrow
  const arrowLen = robotR * 1.6;
  // Canvas Y is inverted, and yaw in ROS is CCW from X-axis
  const canvasAngle = -robotPose.yaw;
  const ax = rp.x + Math.cos(canvasAngle) * arrowLen;
  const ay = rp.y + Math.sin(canvasAngle) * arrowLen;
  ctx.beginPath();
  ctx.moveTo(rp.x, rp.y);
  ctx.lineTo(ax, ay);
  ctx.strokeStyle = '#fff';
  ctx.lineWidth = 2.5;
  ctx.stroke();

  // Arrowhead
  const headLen = 6;
  ctx.beginPath();
  ctx.moveTo(ax, ay);
  ctx.lineTo(ax - headLen * Math.cos(canvasAngle - 0.5), ay - headLen * Math.sin(canvasAngle - 0.5));
  ctx.moveTo(ax, ay);
  ctx.lineTo(ax - headLen * Math.cos(canvasAngle + 0.5), ay - headLen * Math.sin(canvasAngle + 0.5));
  ctx.stroke();

  // Robot label
  ctx.fillStyle = '#fff';
  ctx.font = 'bold 11px sans-serif';
  ctx.textAlign = 'center';
  ctx.fillText('ROBOT', rp.x, rp.y - robotR - 6);

  // Legend
  ctx.font = '11px sans-serif';
  ctx.textAlign = 'left';
  const legendX = 10, legendY = canvas.height - 50;

  ctx.fillStyle = '#4caf50';
  ctx.fillRect(legendX, legendY, 12, 12);
  ctx.fillStyle = '#eee';
  ctx.fillText('Pickup', legendX + 18, legendY + 10);

  ctx.fillStyle = '#2196f3';
  ctx.fillRect(legendX + 80, legendY, 12, 12);
  ctx.fillStyle = '#eee';
  ctx.fillText('Delivery', legendX + 98, legendY + 10);

  ctx.fillStyle = '#ffca28';
  ctx.fillRect(legendX + 170, legendY, 12, 12);
  ctx.fillStyle = '#eee';
  ctx.fillText('Charging', legendX + 188, legendY + 10);

  ctx.fillStyle = '#e94560';
  ctx.fillRect(legendX + 270, legendY, 12, 12);
  ctx.fillStyle = '#eee';
  ctx.fillText('Robot', legendX + 288, legendY + 10);

  // Coordinates overlay
  ctx.fillStyle = 'rgba(0,0,0,0.5)';
  ctx.fillRect(canvas.width - 160, 5, 155, 22);
  ctx.fillStyle = '#aaa';
  ctx.font = '11px monospace';
  ctx.textAlign = 'right';
  ctx.fillText(`x:${robotPose.x.toFixed(1)} y:${robotPose.y.toFixed(1)} yaw:${(robotPose.yaw*180/Math.PI).toFixed(0)}`, canvas.width - 10, 20);
}

// ---------- Animation loop --------------------------------------------------
function animate() {
  drawMap();
  requestAnimationFrame(animate);
}
animate();

// ---------- Connect to rosbridge --------------------------------------------
const ros = new ROSLIB.Ros({ url: ROSBRIDGE_URL });

ros.on('connection', () => {
  console.log('Connected to rosbridge at', ROSBRIDGE_URL);
  elConnStatus.textContent = 'Connected';
  elConnStatus.className = 'status-connected';
});

ros.on('error', (error) => {
  console.error('rosbridge error:', error);
  elConnStatus.textContent = 'Error';
  elConnStatus.className = 'status-disconnected';
});

ros.on('close', () => {
  console.log('Disconnected from rosbridge');
  elConnStatus.textContent = 'Disconnected';
  elConnStatus.className = 'status-disconnected';
  setTimeout(() => { ros.connect(ROSBRIDGE_URL); }, 3000);
});

// ---------- Battery subscriber ----------------------------------------------
const batteryTopic = new ROSLIB.Topic({
  ros: ros,
  name: '/battery_state',
  messageType: 'sensor_msgs/BatteryState',
});

const SUPPLY_STATUS = {
  0: 'UNKNOWN',
  1: 'CHARGING',
  2: 'DISCHARGING',
  3: 'NOT CHARGING',
  4: 'FULL',
};

batteryTopic.subscribe((msg) => {
  const pct = (msg.percentage * 100).toFixed(1);
  elBatteryPct.textContent = pct;
  elBatteryVolt.textContent = msg.voltage.toFixed(1);
  elBatStatus.textContent = SUPPLY_STATUS[msg.power_supply_status] || 'UNKNOWN';

  elBatteryBar.style.width = pct + '%';
  elBatteryBar.classList.remove('low', 'mid');
  if (pct < 20) {
    elBatteryBar.classList.add('low');
  } else if (pct < 50) {
    elBatteryBar.classList.add('mid');
  }
});

// ---------- Current task subscriber -----------------------------------------
const currentTaskTopic = new ROSLIB.Topic({
  ros: ros,
  name: '/current_task',
  messageType: 'warehouse_interfaces/DeliveryOrder',
});

currentTaskTopic.subscribe((msg) => {
  currentTask = msg;
  if (!msg.order_id) {
    elTaskContent.innerHTML = '<p class="dim">No active task -- robot idle</p>';
    return;
  }
  elTaskContent.innerHTML = `
    <div class="grid-2col">
      <div><span class="label">Order:</span> ${msg.order_id}</div>
      <div><span class="label">Status:</span> ${msg.status}</div>
      <div><span class="label">Pickup:</span> ${msg.pickup_station}</div>
      <div><span class="label">Delivery:</span> ${msg.delivery_station}</div>
      <div><span class="label">Priority:</span> ${msg.priority}</div>
    </div>
  `;
});

// ---------- Task queue subscriber -------------------------------------------
const taskQueueTopic = new ROSLIB.Topic({
  ros: ros,
  name: '/task_queue',
  messageType: 'warehouse_interfaces/TaskQueue',
});

taskQueueTopic.subscribe((msg) => {
  if (!msg.orders || msg.orders.length === 0) {
    elQueueBody.innerHTML = '<tr><td colspan="6" class="dim">Queue empty</td></tr>';
    return;
  }
  let html = '';
  msg.orders.forEach((order, i) => {
    html += `<tr>
      <td>${i + 1}</td>
      <td>${order.order_id}</td>
      <td>${order.pickup_station}</td>
      <td>${order.delivery_station}</td>
      <td>${order.priority}</td>
      <td>${order.status}</td>
    </tr>`;
  });
  elQueueBody.innerHTML = html;
});

// ---------- TF subscriber (robot pose) --------------------------------------
const tfTopic = new ROSLIB.Topic({
  ros: ros,
  name: '/tf',
  messageType: 'tf2_msgs/TFMessage',
});

tfTopic.subscribe((msg) => {
  for (const transform of msg.transforms) {
    if (transform.child_frame_id === 'base_footprint' &&
        transform.header.frame_id === 'odom') {
      const t = transform.transform.translation;
      const q = transform.transform.rotation;
      const siny = 2.0 * (q.w * q.z + q.x * q.y);
      const cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z);
      const yaw = Math.atan2(siny, cosy);

      robotPose.x = t.x;
      robotPose.y = t.y;
      robotPose.yaw = yaw;

      elPoseX.textContent = t.x.toFixed(2);
      elPoseY.textContent = t.y.toFixed(2);
      elPoseYaw.textContent = (yaw * 180.0 / Math.PI).toFixed(1);
    }
  }
});

// Also use TFClient for proper map->base_footprint lookup
const tfClient = new ROSLIB.TFClient({
  ros: ros,
  fixedFrame: 'map',
  angularThres: 0.01,
  transThres: 0.01,
});

tfClient.subscribe('base_footprint', (tf) => {
  robotPose.x = tf.translation.x;
  robotPose.y = tf.translation.y;

  const q = tf.rotation;
  const siny = 2.0 * (q.w * q.z + q.x * q.y);
  const cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z);
  robotPose.yaw = Math.atan2(siny, cosy);

  elPoseX.textContent = robotPose.x.toFixed(2);
  elPoseY.textContent = robotPose.y.toFixed(2);
  elPoseYaw.textContent = (robotPose.yaw * 180.0 / Math.PI).toFixed(1);
});

// ---------- Submit order via service ----------------------------------------
document.getElementById('submit-order-btn').addEventListener('click', () => {
  const pickup = document.getElementById('pickup-select').value;
  const delivery = document.getElementById('delivery-select').value;
  const priority = parseInt(document.getElementById('priority-select').value);
  const resultEl = document.getElementById('order-result');
  const submitBtn = document.getElementById('submit-order-btn');

  const client = new ROSLIB.Service({
    ros: ros,
    name: '/submit_order',
    serviceType: 'warehouse_interfaces/srv/SubmitOrder',
  });

  const request = new ROSLIB.ServiceRequest({
    pickup_station: pickup,
    delivery_station: delivery,
    priority: priority,
  });

  if (!ros.isConnected) {
    resultEl.textContent = 'Error: ROS bridge disconnected.';
    resultEl.style.color = '#e53935';
    return;
  }

  resultEl.textContent = 'Submitting...';
  resultEl.style.color = '#ffca28';
  submitBtn.disabled = true;

  // Prevent indefinite "Submitting..." if service is not ready or drops.
  let finished = false;
  const finish = (message, color) => {
    if (finished) {
      return;
    }
    finished = true;
    clearTimeout(timeoutId);
    resultEl.textContent = message;
    resultEl.style.color = color;
    submitBtn.disabled = false;
  };

  const timeoutId = setTimeout(() => {
    finish('Error: /submit_order unavailable. Wait for task manager to start.', '#e53935');
  }, 8000);

  // Check service availability first so users get a quick, actionable message.
  ros.getServices((services) => {
    if (!services.includes('/submit_order')) {
      finish('Error: /submit_order not ready yet. Try again in a few seconds.', '#e53935');
      return;
    }

    client.callService(request, (result) => {
      if (result.accepted) {
        finish(`Order ${result.order_id} accepted!`, '#4caf50');
      } else {
        finish(`Rejected: ${result.message}`, '#e53935');
      }
    }, (error) => {
      finish(`Error: ${error}`, '#e53935');
    });
  }, () => {
    finish('Error: failed to query ROS services.', '#e53935');
  });
});
