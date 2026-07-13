# Kalman Filter: Position and Velocity Estimation from Noisy Position Measurements

A Python implementation of a **Linear Kalman Filter** for estimating **position** and **velocity** using **only noisy position measurements**.

This project demonstrates the complete implementation of a 2-state Kalman Filter using a simulated **sonar altitude estimation** example.

---

## Features

- Linear Kalman Filter implementation from scratch
- Position and velocity estimation
- Noisy sonar measurement simulation
- RMSE evaluation
- Error covariance visualization
- Kalman Gain visualization
- Well-commented Python code for beginners

---

## Problem Statement

Suppose a drone is descending vertically.

A sonar sensor measures only the altitude (position), but the velocity is **not measured directly**.

Instead of estimating velocity by differentiating noisy measurements, the Kalman Filter estimates both states simultaneously.

---

# System Model

## State Vector

```text
        | Position |
x(k) =  |          |
        | Velocity |
```

where

- Position = Altitude of the drone
- Velocity = Vertical speed of the drone

---

## State Transition Matrix

```text
      | 1   dt |
A =   |        |
      | 0    1 |
```

where

```
dt = 0.02 seconds
```

This assumes a **constant velocity model**.

The motion equations are

```text
Position(k+1) = Position(k) + dt × Velocity(k)

Velocity(k+1) = Velocity(k)
```

---

## Measurement Matrix

The sonar measures only position.

```text
H = [1  0]
```

---

# Noise Models

## Process Noise

```text
      | 1  0 |
Q =   |      |
      | 0  3 |
```

This models uncertainty in the constant velocity assumption.

---

## Measurement Noise

```text
R = [10]
```

The measurement noise standard deviation is

```text
σ = √10 ≈ 3.16 m
```

---

# Initial Conditions

Initial state estimate

```text
        |200|
x₀  =   |   |
        | 0 |
```

Initial covariance

```text
      |50   0|
P₀ =  |      |
      | 0  20|
```

Initially we assume

- Position ≈ 200 m
- Velocity = 0 m/s
- Large uncertainty

---

# Kalman Filter Algorithm

Each iteration consists of two stages.

---

## 1. Prediction Step

### Predict State

```text
x̂⁻ = A x̂
```

### Predict Covariance

```text
P⁻ = A P Aᵀ + Q
```

---

## 2. Innovation Step

Innovation

```text
y = z − Hx̂⁻
```

Innovation Covariance

```text
S = H P⁻ Hᵀ + R
```

---

## 3. Kalman Gain

```text
K = P⁻ Hᵀ S⁻¹
```

The Kalman Gain determines how much the prediction should trust the new measurement.

---

## 4. Update Step

Update State

```text
x̂ = x̂⁻ + Ky
```

Update Covariance

```text
P = (I − KH)P⁻
```

---

# Numerical Example

Suppose the first sonar measurement is

```text
z₁ = 199.2 m
```

---

## Prediction

Predicted State

```text
        |200|
x̂⁻ =   |   |
        | 0 |
```

Predicted Covariance

```text
      |51.008   0.4|
P⁻ =  |            |
      |0.4      23 |
```

Notice that the off-diagonal elements become non-zero.

This creates a correlation between position and velocity uncertainty, allowing the Kalman Filter to estimate velocity even though it is never measured directly.

---

## Innovation

Predicted measurement

```text
200 m
```

Innovation

```text
199.2 − 200 = -0.8 m
```

Innovation covariance

```text
61.008
```

---

## Kalman Gain

```text
      |0.8362 |
K =   |       |
      |0.00656|
```

Interpretation

- Position gain is large
- Velocity gain is small

Therefore,

- Position changes significantly
- Velocity changes only slightly

---

## Updated State

```text
        |199.331 |
x̂ =    |        |
        |-0.00525|
```

The filter moves the position estimate toward the measurement while making only a small correction to velocity.

---

## Updated Covariance

```text
      |8.355   0.065|
P =   |             |
      |0.065 22.997 |
```

Notice

- Position uncertainty decreases dramatically.
- Velocity uncertainty decreases slowly over multiple iterations.

---

# Project Structure

```text
Kalman-Filter-Implementation-On-Python
│
├── Kalman_Filter_SONAR_Example.py
├── kalman_position_velocity_sonar.png
├── README.md
└── LICENSE
```

---

# Installation

Clone the repository

```bash
git clone https://github.com/yourusername/Kalman-Filter-Implementation-On-Python.git
```

Move into the project directory

```bash
cd Kalman-Filter-Implementation-On-Python
```

Install dependencies

```bash
pip install numpy matplotlib pandas
```

---

# Running the Program

```bash
python kalman_position_velocity.py
```

---

# Output

The program prints

```text
RMSE raw sonar position

RMSE Kalman position

RMSE Kalman velocity

Final position estimate

Final velocity estimate
```

and generates the following figure.

```
Altitude
├── True Position
├── Noisy Measurements
└── Kalman Estimate

Velocity
├── True Velocity
└── Estimated Velocity

Error Covariance
├── Position Variance
└── Velocity Variance

Kalman Gain
├── Position Gain
└── Velocity Gain
```

---

# Example Output

```text
=== RESULTS ===

RMSE raw sonar position : 3.12 m

RMSE Kalman position : 1.05 m

RMSE Kalman velocity : 0.34 m/s

Final position estimate : 160.08 m

Final velocity estimate : -4.98 m/s
```

*(Exact values vary because the measurement noise is randomly generated.)*

---

# Applications

Kalman Filters are widely used in

- Robotics
- Autonomous Vehicles
- GPS Navigation
- UAV Navigation
- Radar Tracking
- Sonar Tracking
- Sensor Fusion
- Computer Vision
- Object Tracking
- Aerospace Systems

---

# References

1. R. E. Kalman, "A New Approach to Linear Filtering and Prediction Problems", 1960.
2. Greg Welch and Gary Bishop, *An Introduction to the Kalman Filter*.
3. Dan Simon, *Optimal State Estimation*.
4. Brown & Hwang, *Introduction to Random Signals and Applied Kalman Filtering*.

---

# License

This project is released under the **MIT License**.

---

# Author

**Parthib Kumar Dey**

M.Tech Smart Manufacturing  
Department of Design and Manufacturing  
Indian Institute of Science (IISc), Bengaluru

GitHub: https://github.com/parthibtech123