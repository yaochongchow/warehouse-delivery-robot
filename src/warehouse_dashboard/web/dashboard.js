/**
 * Warehouse Robot Web Dashboard -- roslibjs subscriber logic.
 *
 * Connects to rosbridge_websocket and subscribes to:
 *   /battery_state    (sensor_msgs/BatteryState)
 *   /current_task     (warehouse_interfaces/DeliveryOrder)
 *   /task_queue       (warehouse_interfaces/TaskQueue)
 *   /tf               (for robot pose)
 */

// ---------- Configuration ---------------------------------------------------
const ROSBRIDGE_URL = `ws://${window.location.hostname || 'localhost'}:9090`;

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
  // Auto-reconnect after 3 seconds
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
      // Note: in a full system we would compose odom->base_footprint with
      // map->odom.  For simplicity we display odom-frame pose here; a
      // TFClient could be used for the full lookup.
      const t = transform.transform.translation;
      const q = transform.transform.rotation;

      // Yaw from quaternion
      const siny = 2.0 * (q.w * q.z + q.x * q.y);
      const cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z);
      const yaw = Math.atan2(siny, cosy);

      elPoseX.textContent = t.x.toFixed(2);
      elPoseY.textContent = t.y.toFixed(2);
      elPoseYaw.textContent = (yaw * 180.0 / Math.PI).toFixed(1);
    }
  }
});

// Also try using a TFClient for the proper map->base_footprint lookup
const tfClient = new ROSLIB.TFClient({
  ros: ros,
  fixedFrame: 'map',
  angularThres: 0.01,
  transThres: 0.01,
});

tfClient.subscribe('base_footprint', (tf) => {
  elPoseX.textContent = tf.translation.x.toFixed(2);
  elPoseY.textContent = tf.translation.y.toFixed(2);

  const q = tf.rotation;
  const siny = 2.0 * (q.w * q.z + q.x * q.y);
  const cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z);
  const yaw = Math.atan2(siny, cosy);
  elPoseYaw.textContent = (yaw * 180.0 / Math.PI).toFixed(1);
});
