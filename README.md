# Kalman Filter: Position and Velocity Estimation from Noisy Position Measurements

This repository implements a **2-State Linear Kalman Filter** in Python to estimate both **position** and **velocity** using **only noisy position measurements**.

The example is based on the classic **sonar altitude estimation** problem where the sensor measures altitude (position) but does **not** directly measure velocity.

---

# Problem Statement

Suppose a drone is descending vertically.

We can measure its altitude using a noisy sonar sensor, but we cannot measure its velocity directly.

Instead of differentiating noisy measurements, we use a **Kalman Filter** to simultaneously estimate

- Position
- Velocity

from noisy position measurements.

---

# State Model

The system state is

\[
x_k=
\begin{bmatrix}
p_k\\
v_k
\end{bmatrix}
\]

where

- \(p_k\) = Position
- \(v_k\) = Velocity

---

# Motion Model

We assume **constant velocity motion**

\[
x_{k+1}=Ax_k+w_k
\]

where

\[
A=
\begin{bmatrix}
1 & \Delta t\\
0 & 1
\end{bmatrix}
\]

with

- \(\Delta t=0.02\) sec

The state transition equations become

\[
p_{k+1}=p_k+\Delta t\,v_k
\]

\[
v_{k+1}=v_k
\]

---

# Measurement Model

The sonar only measures position.

\[
z_k=Hx_k+v_k
\]

where

\[
H=
\begin{bmatrix}
1&0
\end{bmatrix}
\]

Only the first state (position) is observed.

---

# Noise Models

## Process Noise

The implementation uses

\[
Q=
\begin{bmatrix}
1&0\\
0&3
\end{bmatrix}
\]

This models uncertainty in the constant velocity assumption.

---

## Measurement Noise

The sonar measurement noise is

\[
R=
\begin{bmatrix}
10
\end{bmatrix}
\]

corresponding to a standard deviation of

\[
\sigma=\sqrt{10}\approx3.16\;m
\]

---

# Initial Conditions

Initial state estimate

\[
\hat{x}_0=
\begin{bmatrix}
200\\
0
\end{bmatrix}
\]

Initial covariance

\[
P_0=
\begin{bmatrix}
50&0\\
0&20
\end{bmatrix}
\]

The filter initially assumes

- altitude ≈ 200 m
- velocity unknown (set to zero)
- large uncertainty

---

# Kalman Filter Algorithm

Each iteration consists of two stages.

---

## 1. Prediction

### State Prediction

\[
\hat{x}_k^-=A\hat{x}_{k-1}
\]

### Covariance Prediction

\[
P_k^-=AP_{k-1}A^T+Q
\]

---

## 2. Innovation

Innovation

\[
y_k=z_k-H\hat{x}_k^-
\]

Innovation covariance

\[
S_k=HP_k^-H^T+R
\]

---

## 3. Kalman Gain

\[
K_k=P_k^-H^TS_k^{-1}
\]

The Kalman gain determines how much the prediction should trust the new measurement.

---

## 4. Update

State update

\[
\hat{x}_k=\hat{x}_k^-+K_ky_k
\]

Covariance update

\[
P_k=(I-K_kH)P_k^-
\]

---

# Numerical Example (First Iteration)

Suppose the first sonar measurement is

\[
z_1=199.2\,m
\]

---

## Prediction

Predicted state

\[
\hat{x}_1^-=
\begin{bmatrix}
200\\
0
\end{bmatrix}
\]

Predicted covariance

\[
P_1^-=
\begin{bmatrix}
51.008&0.4\\
0.4&23
\end{bmatrix}
\]

Notice that the off-diagonal terms become non-zero, indicating correlation between position and velocity uncertainty.

---

## Innovation

Predicted measurement

\[
H\hat{x}_1^-=200
\]

Innovation

\[
y_1=199.2-200=-0.8
\]

Innovation covariance

\[
S_1=61.008
\]

---

## Kalman Gain

\[
K_1=
\begin{bmatrix}
0.8362\\
0.00656
\end{bmatrix}
\]

Interpretation:

- Position gain is large → strongly trust the new measurement.
- Velocity gain is small → velocity is only indirectly corrected.

---

## Updated State

\[
\hat{x}_1=
\begin{bmatrix}
199.331\\
-0.00525
\end{bmatrix}
\]

The position moves significantly toward the measurement, while the velocity changes only slightly.

---

## Updated Covariance

\[
P_1\approx
\begin{bmatrix}
8.355&0.065\\
0.065&22.997
\end{bmatrix}
\]

Position uncertainty decreases substantially after the first measurement, whereas velocity uncertainty reduces more gradually over successive iterations.

---

# Output Metrics

The implementation computes

- Raw sonar position RMSE
- Kalman-filtered position RMSE
- Velocity estimation RMSE

Example:

```
RMSE raw sonar position : 3.17 m
RMSE Kalman position    : 1.02 m
RMSE Kalman velocity    : 0.35 m/s
```

*(Exact values vary because measurement noise is randomly generated.)*

---

# Generated Plots

The program produces four plots:

### 1. Position Estimation

- True position
- Noisy sonar measurements
- Kalman filter estimate

---

### 2. Velocity Estimation

Shows how the filter estimates velocity even though it is never directly measured.

---

### 3. Error Covariance

Displays

- Position variance
- Velocity variance

Both gradually decrease as confidence in the estimates improves.

---

### 4. Kalman Gain

Illustrates the convergence of

- Position gain
- Velocity gain

toward steady-state values.

---

# Project Structure

```
.
├── Kalman_Filter_SONAR_Example.py
├── kalman_position_velocity_sonar.png
└── README.md
```

---

# Requirements

Install the required Python packages:

```bash
pip install numpy pandas matplotlib
```

---

# Run

```bash
python kalman_position_velocity.py
```

The script prints the estimation metrics and saves the visualization figure.

---

# References

- R. E. Kalman, "A New Approach to Linear Filtering and Prediction Problems", 1960.
- Dan Simon, *Optimal State Estimation*.
- Greg Welch & Gary Bishop, *An Introduction to the Kalman Filter*.
- Brown & Hwang, *Introduction to Random Signals and Applied Kalman Filtering*.

---

# License

This project is intended for educational and research purposes and demonstrates the implementation of a **Linear Kalman Filter for Position-Velocity Estimation** using noisy position-only measurements.