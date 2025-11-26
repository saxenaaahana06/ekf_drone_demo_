// EKF demo script (client-only). State: [x y z vx vy vz] (6D)
// Simplified motion model: x += vx*dt; vx += ax*dt (ax from accelerometer in body->NED assumed already in NED)
// This demo assumes inputs are already in local NED frame for simplicity.

let state = math.zeros(6,1); // column vector
let P = math.multiply(0.1, math.identity(6));
let Q = math.multiply(0.01, math.identity(6));
let R_gps = math.multiply(1.0, math.identity(3)); // GPS position noise
let R_baro = math.matrix([[0.5]]); // baro noise
let traj = [];

function readInputs() {
  return {
    ax: parseFloat(document.getElementById('ax').value) || 0,
    ay: parseFloat(document.getElementById('ay').value) || 0,
    az: parseFloat(document.getElementById('az').value) || 0,
    gx: parseFloat(document.getElementById('gx').value) || 0,
    gy: parseFloat(document.getElementById('gy').value) || 0,
    gz: parseFloat(document.getElementById('gz').value) || 0,
    gpsx: parseFloat(document.getElementById('gpsx').value),
    gpsy: parseFloat(document.getElementById('gpsy').value),
    gpsz: parseFloat(document.getElementById('gpsz').value),
    baro: parseFloat(document.getElementById('baro').value),
    dt: parseFloat(document.getElementById('dt').value) || 0.1
  };
}

function predict(u) {
  // Continuous acceleration integrated over dt to update velocity, then position
  const dt = u.dt;
  // State vector: [x,y,z,vx,vy,vz]
  // State transition F
  let F = math.identity(6)._data;
  F[0][3] = dt; F[1][4] = dt; F[2][5] = dt;
  F = math.matrix(F);

  // Control: acceleration affects velocities directly (simple)
  let B = math.zeros(6,3);
  B.subset(math.index(3,0), dt);
  B.subset(math.index(4,1), dt);
  B.subset(math.index(5,2), dt);

  let uvec = math.matrix([[u.ax],[u.ay],[u.az]]);

  // Predict state
  state = math.add(math.multiply(F, state), math.multiply(B, uvec));
  // Predict covariance
  P = math.add(math.multiply(F, math.multiply(P, math.transpose(F))), Q);
}

function updateGPS(u) {
  // Measurement z = [x y z]^T
  let z = math.matrix([[u.gpsx],[u.gpsy],[u.gpsz]]);
  // H matrix: pick position from state
  let H = math.zeros(3,6);
  H.subset(math.index(0,0),1);
  H.subset(math.index(1,1),1);
  H.subset(math.index(2,2),1);
  let S = math.add(math.multiply(H, math.multiply(P, math.transpose(H))), R_gps);
  let K = math.multiply(P, math.multiply(math.transpose(H), math.inv(S)));
  let y = math.subtract(z, math.multiply(H, state));
  state = math.add(state, math.multiply(K, y));
  P = math.multiply(math.subtract(math.identity(6), math.multiply(K,H)), P);
}

function updateBaro(u) {
  // barometer measures altitude (z)
  let z = math.matrix([[u.baro]]);
  let H = math.zeros(1,6);
  H.subset(math.index(0,2),1);
  let S = math.add(math.multiply(H, math.multiply(P, math.transpose(H))), R_baro);
  let K = math.multiply(P, math.multiply(math.transpose(H), math.inv(S)));
  let y = math.subtract(z, math.multiply(H, state));
  state = math.add(state, math.multiply(K, y));
  P = math.multiply(math.subtract(math.identity(6), math.multiply(K,H)), P);
}

function stepOnce() {
  const u = readInputs();
  predict(u);
  // If GPS is not NaN, update
  if (!isNaN(u.gpsx) && !isNaN(u.gpsy) && !isNaN(u.gpsz)) updateGPS(u);
  if (!isNaN(u.baro)) updateBaro(u);
  traj.push([state.get([0,0]), state.get([1,0])]);
  renderState();
}

function renderState() {
  document.getElementById('sx').innerText = state.get([0,0]).toFixed(3);
  document.getElementById('sy').innerText = state.get([1,0]).toFixed(3);
  document.getElementById('sz').innerText = state.get([2,0]).toFixed(3);
  document.getElementById('svx').innerText = state.get([3,0]).toFixed(3);
  document.getElementById('svy').innerText = state.get([4,0]).toFixed(3);
  document.getElementById('svz').innerText = state.get([5,0]).toFixed(3);

  // Show position covariance block
  const Ppos = [
    [P.get([0,0]), P.get([0,1]), P.get([0,2])],
    [P.get([1,0]), P.get([1,1]), P.get([1,2])],
    [P.get([2,0]), P.get([2,1]), P.get([2,2])]
  ];
  document.getElementById('pStr').innerText = JSON.stringify(Ppos, null, 2);

  // Draw trajectory
  const canvas = document.getElementById('traj');
  const ctx = canvas.getContext('2d');
  ctx.clearRect(0,0,canvas.width,canvas.height);
  // draw axes
  ctx.strokeStyle = '#ddd';
  ctx.beginPath(); ctx.moveTo(40,10); ctx.lineTo(40,canvas.height-30); ctx.lineTo(canvas.width-10, canvas.height-30); ctx.stroke();

  // compute scale
  let xs = traj.map(t => t[0]);
  let ys = traj.map(t => t[1]);
  let minx = Math.min(...xs, -5), maxx = Math.max(...xs,5);
  let miny = Math.min(...ys, -5), maxy = Math.max(...ys,5);
  let w = canvas.width - 60, h = canvas.height - 50;
  function sx(x){ return 40 + (x - minx)/(maxx-minx || 1) * w; }
  function sy(y){ return (canvas.height-30) - (y - miny)/(maxy-miny || 1) * h; }

  // draw path
  ctx.strokeStyle = '#0b74d1'; ctx.beginPath();
  for (let i=0;i<traj.length;i++){
    const [x,y] = traj[i];
    if (i===0) ctx.moveTo(sx(x), sy(y)); else ctx.lineTo(sx(x), sy(y));
  }
  ctx.stroke();

  // draw current
  if (traj.length){
    const [x,y] = traj[traj.length-1];
    ctx.fillStyle = '#d12b2b';
    ctx.beginPath(); ctx.arc(sx(x), sy(y), 4, 0, Math.PI*2); ctx.fill();
  }
}

// simple logging CSV
function downloadLog() {
  let csv = 'time,x,y,z,vx,vy,vz\\n';
  // for demo: create rows from traj and state not full history stored; we'll store minimal entries
  // Here we will create a single-row snapshot (last)
  csv += '0,' + state.get([0,0]) + ',' + state.get([1,0]) + ',' + state.get([2,0]) + ',' + state.get([3,0]) + ',' + state.get([4,0]) + ',' + state.get([5,0]) + '\\n';
  const blob = new Blob([csv], {type: 'text/csv'});
  const url = URL.createObjectURL(blob);
  const a = document.createElement('a');
  a.href = url; a.download = 'ekf_log.csv'; document.body.appendChild(a); a.click(); a.remove();
}

// wire UI
document.getElementById('stepBtn').addEventListener('click', stepOnce);
document.getElementById('resetBtn').addEventListener('click', ()=>{ state = math.zeros(6,1); P = math.multiply(0.1, math.identity(6)); traj = []; renderState(); });
document.getElementById('runBtn').addEventListener('click', ()=>{
  // run for simulated duration (e.g., 10 seconds)
  const dt = parseFloat(document.getElementById('dt').value) || 0.1;
  const steps = Math.round(10 / dt);
  let i=0;
  function runtick(){
    stepOnce();
    i++;
    if (i<steps) setTimeout(runtick, dt*1000);
  }
  runtick();
});
document.getElementById('downloadLogBtn').addEventListener('click', downloadLog);

// initial render
renderState();
