EKF UAV Demo Website
=====================

This is a simple client-only demo that demonstrates an Extended Kalman Filter (EKF)
for a 6-state UAV model: position (x,y,z) and velocity (vx,vy,vz).

Features:
- Enter IMU accelerations (assumed in local NED), GPS position, and barometer altitude.
- Single-step prediction + updates for GPS and barometer.
- Trajectory canvas visualization.
- Downloadable CSV snapshot.

How to use:
1. Unzip the folder and open index.html in a modern browser (no server required).
2. Change inputs and press "Step" to run one EKF cycle, or "Run (10s)" to simulate 10 seconds.
3. Reset to clear state.

Notes & next steps:
- This demo assumes sensor inputs are pre-processed into a local NED frame.
- The EKF here is intentionally simplified for clarity:
  - State vector: [x y z vx vy vz]
  - Motion model: constant-velocity with acceleration control input.
  - No orientation states or coordinate transforms are implemented.
- For a production system:
  - Add orientation (roll/pitch/yaw) and transform accelerations from body to NED.
  - Use a higher-fidelity process model and tune Q/R matrices.
  - Consider server-side logging, real-time plotting, and file upload for logs.

You can host this on GitHub Pages or Netlify by uploading the three files:
index.html, style.css, script.js
