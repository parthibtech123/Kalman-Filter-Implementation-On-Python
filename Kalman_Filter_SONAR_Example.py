"""
Kalman Filter: estimating position AND velocity from noisy position-only
measurements (sonar altitude example, matching the MATLAB tutorial).
 
State:      x = [position, velocity]^T      (n = 2)
Measurement: z = position only               (m = 1)
"""
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt

np.random.seed(3)

## 1. System Model ()  n = 2 state, m = 1 measurement

dt = 0.02 # Sonar Sample time (sec)

A = np.array([[1,dt],   #### (2x2) state transition: p_k+1 = p_k + dt*v_k ; v_k+1 = v_k
              [0,1]])

H = np.array([[1,0]])  # (1x2) measurement matrix: we only see position

Q = np.array([[1.0, 0.0],        # (2x2) process noise -- how much we distrust "constant velocity"
              [0.0, 3.0]])       #   larger on velocity since that's the shakier assumption


R = np.array([[10.0]])           # (1x1) measurement noise -- sonar sensor error

I = np.eye(2)

x = np.array([[200.0],           # initial altitude guess (m)
              [0.0]])            # initial velocity guess (m/s) -- no info, so 0
P = np.array([[50.0, 0.0],       # large initial uncertainty -- "we don't really know"
              [0.0, 20.0]])


print("=== MODEL ===")
print(f"dt = {dt}")
print("A =\n", A)
print("H =\n", H)
print("Q =\n", Q)
print("R =\n", R)
print("Initial x =\n", x)
print("Initial P =\n", P)
print()


# 2. SIMULATE TRUE ALTITUDE + NOISY SONAR MEASUREMENTS

N = 400                       # number of samples (400 * 0.02s = 8 seconds)
true_v = -5.0                 # descending at 5 m/s (e.g. a drone landing)
true_p0 = 200.0
 
time = np.arange(N) * dt
true_positions = true_p0 + true_v * time                 # constant-velocity descent
true_velocities = np.full(N, true_v)
 
sonar_std = np.sqrt(R[0, 0])                              # sqrt(10) ~= 3.16 m
measurements = true_positions + np.random.normal(0, sonar_std, N)
 
# ============================================================
# 3. RUN THE KALMAN FILTER
# ============================================================
est_pos, est_vel = [], []
P_pos_var, P_vel_var = [], []
gains_pos, gains_vel = [], []
 
for z_val in measurements:
    z = np.array([[z_val]])
 
    # ---- PREDICT ----
    x_pred = A @ x
    P_pred = A @ P @ A.T + Q
 
    # ---- ESTIMATE / UPDATE ----
    innovation = z - H @ x_pred                    # y_k = z_k - H x_k^-
    S = H @ P_pred @ H.T + R                        # innovation covariance
    K = P_pred @ H.T @ np.linalg.inv(S)             # Kalman gain (2x1)
 
    x = x_pred + K @ innovation
    P = (I - K @ H) @ P_pred
 
    est_pos.append(x[0, 0])
    est_vel.append(x[1, 0])
    P_pos_var.append(P[0, 0])
    P_vel_var.append(P[1, 1])
    gains_pos.append(K[0, 0])
    gains_vel.append(K[1, 0])
 
est_pos = np.array(est_pos)
est_vel = np.array(est_vel)
P_pos_var = np.array(P_pos_var)
P_vel_var = np.array(P_vel_var)
 
# ============================================================
# 4. METRICS
# ============================================================
rmse_raw_pos = np.sqrt(np.mean((measurements - true_positions) ** 2))
rmse_kf_pos = np.sqrt(np.mean((est_pos - true_positions) ** 2))
rmse_kf_vel = np.sqrt(np.mean((est_vel - true_velocities) ** 2))
 
print("=== RESULTS ===")
print(f"RMSE raw sonar position:    {rmse_raw_pos:.3f} m")
print(f"RMSE Kalman position:       {rmse_kf_pos:.3f} m")
print(f"RMSE Kalman velocity:       {rmse_kf_vel:.3f} m/s  (true velocity: {true_v} m/s)")
print(f"Final position estimate:    {est_pos[-1]:.2f} m")
print(f"Final velocity estimate:    {est_vel[-1]:.2f} m/s")
 
# ============================================================
# 5. VISUALIZATION
# ============================================================
fig, axes = plt.subplots(2, 2, figsize=(13, 9))
 
ax = axes[0, 0]
ax.plot(time, true_positions, color="black", linewidth=2, label="True altitude")
ax.scatter(time, measurements, color="red", s=8, alpha=0.4, label="Noisy sonar")
ax.plot(time, est_pos, color="green", linewidth=2, label="Kalman estimate")
ax.set_title("Altitude: True vs Sonar Measurement vs Estimate")
ax.set_xlabel("Time (s)")
ax.set_ylabel("Altitude (m)")
ax.legend()
ax.grid(alpha=0.3)
 
ax = axes[0, 1]
ax.axhline(true_v, color="black", linewidth=2, label="True velocity")
ax.plot(time, est_vel, color="purple", linewidth=2, label="Kalman estimate (never measured directly!)")
ax.set_title("Velocity: Inferred purely from noisy position")
ax.set_xlabel("Time (s)")
ax.set_ylabel("Velocity (m/s)")
ax.legend()
ax.grid(alpha=0.3)
 
ax = axes[1, 0]
ax.plot(time, P_pos_var, label="Position variance (P[0,0])", color="green")
ax.plot(time, P_vel_var, label="Velocity variance (P[1,1])", color="purple")
ax.set_title("Error Covariance P Shrinking (Confidence Growing)")
ax.set_xlabel("Time (s)")
ax.set_ylabel("Variance")
ax.legend()
ax.grid(alpha=0.3)
 
ax = axes[1, 1]
ax.plot(time, gains_pos, label="Gain: position row", color="green")
ax.plot(time, gains_vel, label="Gain: velocity row", color="purple")
ax.set_title("Kalman Gain K Converging")
ax.set_xlabel("Time (s)")
ax.set_ylabel("Gain value")
ax.legend()
ax.grid(alpha=0.3)
 
plt.tight_layout()
plt.savefig("G:\Kalma_Filters\Kalman-Filter-Implementation-On-Python\kalman_position_velocity_sonar.png", dpi=150)
print("\nSaved: kalman_position_velocity_sonar.png")
