# Rocketry Sensor Fusion: MEKF State Estimation Engine

**A high-fidelity, production-grade Extended Kalman Filter for rocket flight state estimation.**

This project implements a complete sensor fusion pipeline for liquid bipropellant rockets, producing a single, drift-corrected estimate of the vehicle's position, velocity, and orientation by intelligently combining noisy data from GPS, barometers, magnetometers, and inertial measurement units (IMUs).

---

## Table of Contents

- [Motivation & Problem Statement](#motivation--problem-statement)
- [What This Project Does](#what-this-project-does)
- [Mathematical Foundation](#mathematical-foundation)
  - [The State Vector](#the-state-vector)
  - [The MEKF Formulation](#the-mekf-formulation)
  - [Prediction Step](#prediction-step)
  - [Update Step](#update-step)
  - [Flight-Phase-Adaptive Process Noise](#flight-phase-adaptive-process-noise)
  - [Measurement Noise Tuning](#measurement-noise-tuning)
- [Software Architecture](#software-architecture)
  - [Module 1: Data Ingestion & Pre-processing](#module-1-data-ingestion--pre-processing)
  - [Module 2: State Estimation Engine (EKF Core)](#module-2-state-estimation-engine-ekf-core)
  - [Module 3: Flight Phase State Machine](#module-3-flight-phase-state-machine)
  - [Module 4: Logging & Output](#module-4-logging--output)
  - [Module 5: Analysis & Visualization (Python)](#module-5-analysis--visualization-python)
- [Technology Stack](#technology-stack)
- [Directory Structure](#directory-structure)
- [Building the Project](#building-the-project)
  - [Prerequisites](#prerequisites)
  - [Build Steps](#build-steps)
- [Running the Project](#running-the-project)
  - [Quick Start (Synthetic Data)](#quick-start-synthetic-data)
  - [Workflow Explanation](#workflow-explanation)
- [Test Suite](#test-suite)
  - [Test Organization](#test-organization)
  - [Running Tests](#running-tests)
  - [Test Coverage Summary](#test-coverage-summary)
- [Current Status (Phase 1 Complete)](#current-status-phase-1-complete)
- [Roadmap: Open Work](#roadmap-open-work)
  - [Phase 2: Infrastructure](#phase-2-infrastructure)
  - [Phase 3: Filter Validation & Diagnostics](#phase-3-filter-validation--diagnostics)
  - [Phase 4: Real Hardware Integration](#phase-4-real-hardware-integration)
  - [Phase 5: Advanced Features](#phase-5-advanced-features)
- [Author](#author)

---

## Motivation & Problem Statement

Rocket flight relies on accurate knowledge of the vehicle's state: where it is, how fast it's moving, and which direction it's pointing. No single sensor provides perfect information:

- **GPS** gives absolute position but updates slowly (~5 Hz) and has 2–5 meter noise.
- **Barometers** measure altitude via air pressure, affected by weather and temperature drift.
- **IMUs** (accelerometers + gyroscopes) deliver high-frequency data (100 Hz+) but accumulate **integration drift** — errors compound exponentially over time.
- **Magnetometers** provide heading reference but are disturbed by magnetic interference.

A naive approach of "just use GPS for position and IMU for everything else" fails within seconds. The IMU drifts meters off-course in under 10 seconds without correction. Conversely, using only GPS means you have no state estimate between 200ms updates, and velocity/acceleration are unavailable.

**Sensor fusion** solves this by combining the strengths of each sensor while compensating for their weaknesses. An **Extended Kalman Filter (EKF)** is the industry-standard algorithm for this task. This project implements a **Multiplicative Extended Kalman Filter (MEKF)**, a specialized variant that keeps attitude (orientation) representations numerically stable by maintaining quaternions on the unit sphere.

---

## What This Project Does

This software:

1. **Ingests** raw sensor telemetry from binary log files (or real-time streams).
2. **Fuses** the data using a 16-state MEKF with bias tracking.
3. **Produces** a high-frequency (100 Hz), drift-corrected state estimate including:
   - 3D position (x, y, z in meters)
   - 3D velocity (vₓ, vᵧ, vᵧ in m/s)
   - Orientation as a unit quaternion (q)
   - IMU biases (gyroscope and accelerometer offsets)
   - Flight phase (PRE_LAUNCH → BOOST → COAST → DESCENT → LANDED)
4. **Logs** all data to a CSV file for post-flight analysis.
5. **Visualizes** the results in Python with multi-panel diagnostic plots.

**Key features:**

- **Bias tracking:** Estimates and corrects for IMU sensor drift in real-time.
- **Flight-phase adaptation:** Process noise automatically adjusts based on whether the rocket is launching, coasting, or descending.
- **GPS origin averaging:** Averages the first 10 GPS packets to eliminate systematic position offsets that would otherwise contaminate bias estimates.
- **Numerically stable:** MEKF formulation prevents quaternion denormalization and covariance indefiniteness.
- **Extensively tested:** 55+ unit tests covering initialization, prediction, updates, numerical stability, and end-to-end validation.

---

## Mathematical Foundation

This section explains the filter's mathematics in detail. We start from first principles and build up to the full MEKF algorithm.

### The State Vector

The filter tracks a **16-dimensional nominal state vector** $\mathbf{x}$:

$$
\mathbf{x} = \begin{bmatrix}
\mathbf{p} \\
\mathbf{v} \\
\mathbf{q} \\
\mathbf{b}_g \\
\mathbf{b}_a
\end{bmatrix}
=
\begin{bmatrix}
p_x, p_y, p_z \\
v_x, v_y, v_z \\
q_w, q_x, q_y, q_z \\
b_{gx}, b_{gy}, b_{gz} \\
b_{ax}, b_{ay}, b_{az}
\end{bmatrix}
$$

Where:

- $\mathbf{p}$ ∈ ℝ³: **Position** in a local East-North-Up (ENU) coordinate frame, with origin set by the first GPS lock. Units: meters.
- $\mathbf{v}$ ∈ ℝ³: **Velocity** in the same ENU frame. Units: m/s.
- $\mathbf{q}$ ∈ ℍ: **Orientation** as a unit quaternion representing the rotation from the ENU frame to the vehicle body frame. The quaternion has the constraint $\|\mathbf{q}\| = 1$.
- $\mathbf{b}_g$ ∈ ℝ³: **Gyroscope bias** — the constant offset error in the gyroscope measurements. Units: rad/s.
- $\mathbf{b}_a$ ∈ ℝ³: **Accelerometer bias** — the constant offset error in the accelerometer measurements. Units: m/s².

**Why track biases?** IMU sensors have manufacturing imperfections that cause them to report a non-zero value even when stationary. For example, a gyroscope might read +0.02 rad/s when the rocket is perfectly still. If we integrate this directly, we accumulate 0.02 rad/s × 100 seconds = 2 radians = 114° of false rotation. By estimating and subtracting the bias, we eliminate this drift.

### The MEKF Formulation

A standard EKF would track all 16 states directly and propagate a 16×16 covariance matrix. However, the quaternion constraint $\|\mathbf{q}\| = 1$ creates a problem: the covariance matrix must respect this constraint, requiring a complex normalization Jacobian at every step.

The **Multiplicative EKF (MEKF)** solves this elegantly:

- The **nominal state** $\mathbf{x}$ remains 16-dimensional and includes the full 4-component quaternion.
- The **error state** (used for covariance propagation) is **15-dimensional**, replacing the 4-component quaternion with a **3-component attitude error vector** $\delta\boldsymbol{\theta}$.

The error-state vector is:

$$
\delta\mathbf{x} = \begin{bmatrix}
\delta\mathbf{p} \\
\delta\mathbf{v} \\
\delta\boldsymbol{\theta} \\
\delta\mathbf{b}_g \\
\delta\mathbf{b}_a
\end{bmatrix} \in \mathbb{R}^{15}
$$

The **covariance matrix** $\mathbf{P}$ is 15×15, not 16×16. This removes the quaternion constraint from the covariance space entirely.

**How does this work?**

1. **Prediction:** We propagate the full 16-state nominal vector using the nonlinear dynamics. Separately, we propagate the 15×15 covariance using a linearized error-state model.

2. **Correction:** When a measurement arrives, we compute a 15-dimensional error-state correction $\delta\mathbf{x}$. For the attitude component, we apply this correction **multiplicatively**:

$$
\mathbf{q}_{\text{new}} = \mathbf{q}_{\text{old}} \otimes \delta\mathbf{q}
$$

where $\delta\mathbf{q}$ is constructed from $\delta\boldsymbol{\theta}$ via the small-angle approximation:

$$
\delta\mathbf{q} \approx \begin{bmatrix} 1 \\ \delta\boldsymbol{\theta}/2 \end{bmatrix}
$$

This is then explicitly normalized: $\mathbf{q}_{\text{new}} \leftarrow \mathbf{q}_{\text{new}} / \|\mathbf{q}_{\text{new}}\|$.

The multiplicative correction keeps the quaternion **exactly on the unit sphere** without requiring constraint handling in the covariance propagation. This is why MEKF is the gold standard for spacecraft and aircraft attitude estimation.

### Prediction Step

The prediction step runs at the IMU rate (100 Hz in this implementation). At each timestep $k$, we receive a new IMU measurement:

$$
\mathbf{u}_k = \begin{bmatrix} \boldsymbol{\omega}_{\text{raw}} \\ \mathbf{a}_{\text{raw}} \end{bmatrix}
$$

where $\boldsymbol{\omega}_{\text{raw}}$ is the raw gyroscope reading (rad/s) and $\mathbf{a}_{\text{raw}}$ is the raw accelerometer reading (m/s²).

#### Step 1: Bias Correction

We subtract the estimated biases from the raw measurements:

$$
\boldsymbol{\omega} = \boldsymbol{\omega}_{\text{raw}} - \mathbf{b}_g
$$

$$
\mathbf{a}_b = \mathbf{a}_{\text{raw}} - \mathbf{b}_a
$$

where $\mathbf{a}_b$ is the corrected acceleration in the **body frame**.

#### Step 2: Orientation Update

We integrate the angular velocity to update the quaternion. Using the quaternion derivative formula:

$$
\dot{\mathbf{q}} = \frac{1}{2} \mathbf{q} \otimes \begin{bmatrix} 0 \\ \boldsymbol{\omega} \end{bmatrix}
$$

Discretized with Euler integration (timestep $\Delta t$):

$$
\mathbf{q}_{k+1} = \mathbf{q}_k + \frac{\Delta t}{2} \left( \mathbf{q}_k \otimes \begin{bmatrix} 0 \\ \boldsymbol{\omega} \end{bmatrix} \right)
$$

Then normalize: $\mathbf{q}_{k+1} \leftarrow \mathbf{q}_{k+1} / \|\mathbf{q}_{k+1}\|$.

#### Step 3: Acceleration Transformation

The corrected acceleration $\mathbf{a}_b$ is in the body frame. We rotate it to the world frame using the rotation matrix $\mathbf{R}(\mathbf{q})$:

$$
\mathbf{a}_{\text{world}} = \mathbf{R}(\mathbf{q}_{k+1}) \mathbf{a}_b
$$

Then subtract gravity (which acts in the world frame):

$$
\mathbf{a}_{\text{net}} = \mathbf{a}_{\text{world}} - \begin{bmatrix} 0 \\ 0 \\ 9.81 \end{bmatrix}
$$

#### Step 4: Velocity and Position Update

Using Euler integration:

$$
\mathbf{v}_{k+1} = \mathbf{v}_k + \mathbf{a}_{\text{net}} \Delta t
$$

$$
\mathbf{p}_{k+1} = \mathbf{p}_k + \mathbf{v}_k \Delta t + \frac{1}{2} \mathbf{a}_{\text{net}} \Delta t^2
$$

#### Step 5: Bias Propagation

The biases are modeled as **random walk** processes. Their dynamics are:

$$
\dot{\mathbf{b}}_g = \mathbf{w}_g, \quad \dot{\mathbf{b}}_a = \mathbf{w}_a
$$

where $\mathbf{w}_g$ and $\mathbf{w}_a$ are white noise processes (captured in the process noise matrix $\mathbf{Q}$). In discrete time, this means the biases don't change during prediction:

$$
\mathbf{b}_{g,k+1} = \mathbf{b}_{g,k}, \quad \mathbf{b}_{a,k+1} = \mathbf{b}_{a,k}
$$

#### Step 6: Error-State Covariance Propagation

The error-state covariance is propagated using the **linearized error dynamics**:

$$
\mathbf{P}_{k+1} = \mathbf{F}_k \mathbf{P}_k \mathbf{F}_k^T + \mathbf{Q}_k
$$

where $\mathbf{F}_k$ is the **state transition Jacobian** and $\mathbf{Q}_k$ is the **process noise matrix**.

The state transition Jacobian $\mathbf{F}$ (15×15) has the following block structure:

$$
\mathbf{F} = \begin{bmatrix}
\mathbf{I}_3 & \mathbf{I}_3 \Delta t & \frac{1}{2}\Delta t^2 \mathbf{J}_{\text{att}} & \mathbf{0} & -\frac{1}{2}\Delta t^2 \mathbf{R} \\
\mathbf{0} & \mathbf{I}_3 & \Delta t \mathbf{J}_{\text{att}} & \mathbf{0} & -\Delta t \mathbf{R} \\
\mathbf{0} & \mathbf{0} & \mathbf{I}_3 - [\boldsymbol{\omega}]_\times \Delta t & -\mathbf{I}_3 \Delta t & \mathbf{0} \\
\mathbf{0} & \mathbf{0} & \mathbf{0} & \mathbf{I}_3 & \mathbf{0} \\
\mathbf{0} & \mathbf{0} & \mathbf{0} & \mathbf{0} & \mathbf{I}_3
\end{bmatrix}
$$

where:

- $\mathbf{R} = \mathbf{R}(\mathbf{q}_{k+1})$ is the rotation matrix from body to world.
- $[\boldsymbol{\omega}]_\times$ is the **skew-symmetric matrix** (cross-product matrix) of $\boldsymbol{\omega}$:

$$
[\boldsymbol{\omega}]_\times = \begin{bmatrix}
0 & -\omega_z & \omega_y \\
\omega_z & 0 & -\omega_x \\
-\omega_y & \omega_x & 0
\end{bmatrix}
$$

- $\mathbf{J}_{\text{att}}$ is the Jacobian of acceleration with respect to attitude error:

$$
\mathbf{J}_{\text{att}} = -\mathbf{R} [\mathbf{a}_b]_\times
$$

This captures the fact that a small rotation error $\delta\boldsymbol{\theta}$ causes the rotated acceleration vector to point in a slightly wrong direction.

**Key insight:** The off-diagonal blocks couple position/velocity errors to attitude and bias errors. This is how the filter learns biases: if the accelerometer has a bias, the integrated position will drift. When GPS corrects the position, the Kalman gain routes part of that correction into the bias estimate via $\mathbf{P}[\delta\mathbf{p}, \delta\mathbf{b}_a]$.

### Update Step

The update step runs whenever a slower sensor provides a measurement:

- **Barometer:** ~10 Hz, measures altitude via air pressure.
- **GPS:** ~5 Hz, measures 3D position (latitude, longitude, altitude).
- **Magnetometer:** ~20 Hz, measures heading (yaw angle).

#### Measurement Model

Each sensor has a measurement model of the form:

$$
\mathbf{z} = \mathbf{h}(\mathbf{x}) + \mathbf{v}
$$

where $\mathbf{z}$ is the measurement, $\mathbf{h}(\mathbf{x})$ is the predicted measurement from the state, and $\mathbf{v}$ is measurement noise with covariance $\mathbf{R}$.

#### Examples

**Barometer:**

The barometer measures altitude. The measurement model is simply:

$$
z_{\text{baro}} = p_z + v_{\text{baro}}
$$

The measurement Jacobian $\mathbf{H}_{\text{baro}}$ (1×15) has a single `1` at index 2 (the $p_z$ slot):

$$
\mathbf{H}_{\text{baro}} = \begin{bmatrix} 0 & 0 & 1 & 0 & \cdots & 0 \end{bmatrix}
$$

**GPS:**

GPS measures all three position components:

$$
\mathbf{z}_{\text{GPS}} = \begin{bmatrix} p_x \\ p_y \\ p_z \end{bmatrix} + \mathbf{v}_{\text{GPS}}
$$

$$
\mathbf{H}_{\text{GPS}} = \begin{bmatrix}
1 & 0 & 0 & 0 & \cdots & 0 \\
0 & 1 & 0 & 0 & \cdots & 0 \\
0 & 0 & 1 & 0 & \cdots & 0
\end{bmatrix}
$$

**Magnetometer:**

The magnetometer measures heading (yaw angle $\psi$). This is a nonlinear function of the quaternion:

$$
\psi = \text{atan2}(2(q_w q_z + q_x q_y), \, 1 - 2(q_y^2 + q_z^2))
$$

The Jacobian $\mathbf{H}_{\text{mag}}$ is derived by taking the derivative of $\psi$ with respect to the attitude error $\delta\boldsymbol{\theta}$. This requires the chain rule through the quaternion update formula. The result is a 1×15 row vector with non-zero entries only in the attitude block (indices 6–8).

#### Kalman Gain and State Correction

The **innovation** (measurement residual) is:

$$
\mathbf{y} = \mathbf{z} - \mathbf{h}(\mathbf{x})
$$

The **innovation covariance** is:

$$
\mathbf{S} = \mathbf{H} \mathbf{P} \mathbf{H}^T + \mathbf{R}
$$

The **Kalman gain** is:

$$
\mathbf{K} = \mathbf{P} \mathbf{H}^T \mathbf{S}^{-1}
$$

The **error-state correction** is:

$$
\delta\mathbf{x} = \mathbf{K} \mathbf{y}
$$

#### Applying the Correction

For position, velocity, and biases (indices 0–5, 9–14), the correction is **additive**:

$$
\mathbf{p} \leftarrow \mathbf{p} + \delta\mathbf{p}
$$

For attitude (indices 6–8), the correction is **multiplicative**:

$$
\delta\mathbf{q} = \begin{bmatrix} 1 \\ \delta\boldsymbol{\theta} / 2 \end{bmatrix}, \quad \mathbf{q} \leftarrow \text{normalize}(\mathbf{q} \otimes \delta\mathbf{q})
$$

#### Covariance Update

The covariance is updated using the **Joseph form**, which is numerically stable and guarantees positive-definiteness:

$$
\mathbf{P} \leftarrow (\mathbf{I} - \mathbf{K}\mathbf{H}) \mathbf{P} (\mathbf{I} - \mathbf{K}\mathbf{H})^T + \mathbf{K} \mathbf{R} \mathbf{K}^T
$$

This is more expensive than the simplified form $(\mathbf{I} - \mathbf{K}\mathbf{H})\mathbf{P}$, but it prevents the covariance from becoming indefinite due to floating-point roundoff errors.

### Flight-Phase-Adaptive Process Noise

The process noise matrix $\mathbf{Q}$ determines how much the filter "trusts" its own prediction versus new measurements. It should reflect the true uncertainty in the dynamics.

A rocket's dynamics change drastically between flight phases:

- **PRE_LAUNCH:** Stationary on the pad. Position/velocity shouldn't change at all.
- **BOOST:** High acceleration (~20 m/s²), violent vibrations, thrust model uncertainty.
- **COAST:** Ballistic flight under gravity only. Smooth, predictable dynamics.
- **DESCENT:** Parachute deployed, drag-dominated descent.
- **LANDED:** Stationary on the ground.

Using a single $\mathbf{Q}$ for all phases would either over-trust the model during boost (causing the filter to ignore measurements) or under-trust it during coast (causing excessive measurement noise to be incorporated).

**Solution:** Adaptive process noise. The filter tracks the current flight phase via a finite state machine (see [Flight Phase State Machine](#module-3-flight-phase-state-machine)) and selects $\mathbf{Q}$ accordingly.

The process noise is **diagonal** (states are uncorrelated):

$$
\mathbf{Q} = \text{diag}(q_{\mathbf{p}}, q_{\mathbf{v}}, q_{\boldsymbol{\theta}}, q_{\mathbf{b}_g}, q_{\mathbf{b}_a})
$$

where each $q$ is a 3×3 block with the same scalar value on the diagonal. The tuned values are:

| Phase       | $q_{\mathbf{p}}$ | $q_{\mathbf{v}}$ | $q_{\boldsymbol{\theta}}$ | $q_{\mathbf{b}_g}$ | $q_{\mathbf{b}_a}$ |
|-------------|-----------|-----------|----------------|-----------|-----------|
| PRE_LAUNCH  | 1×10⁻⁶   | 1×10⁻⁴   | 1×10⁻⁵        | 1×10⁻⁸   | 1×10⁻⁷   |
| BOOST       | 1×10⁻⁴   | 0.1       | 5×10⁻⁴        | 1×10⁻⁷   | 1×10⁻⁷   |
| COAST       | 1×10⁻⁵   | 0.01      | 5×10⁻⁴        | 1×10⁻⁸   | 1×10⁻⁷   |
| DESCENT     | 1×10⁻⁵   | 1×10⁻³   | 5×10⁻⁴        | 1×10⁻⁸   | 1×10⁻⁷   |
| LANDED      | 1×10⁻⁸   | 1×10⁻⁶   | 1×10⁻⁶        | 1×10⁻⁹   | 1×10⁻⁹   |

**Key observations:**

- $q_{\mathbf{v}}$ increases by 1000× from PRE_LAUNCH to BOOST, reflecting genuine velocity uncertainty from thrust.
- $q_{\mathbf{b}_a}$ stays constant at 1×10⁻⁷ across PRE_LAUNCH/BOOST/COAST. This is deliberate: the accelerometer bias is a **hardware property** (sensor offset), not a flight-phase-dependent quantity. Increasing it during boost would allow noise-driven baro residuals to produce large spurious bias updates.
- $q_{\boldsymbol{\theta}}$ is reduced from 1×10⁻³ → 5×10⁻⁴ during boost. This limits the growth of $\mathbf{P}[\delta\boldsymbol{\theta}, \delta\mathbf{p}]$ (attitude-position cross-covariance), preventing large attitude corrections when the first GPS/baro measurement arrives at coast entry.

### Measurement Noise Tuning

The measurement noise covariance $\mathbf{R}$ represents the **sensor noise** for each measurement type. Ideally, this matches the actual sensor specifications. In practice, $\mathbf{R}$ is often **inflated** beyond the true sensor noise to account for unmodeled effects (e.g., multi-path GPS errors, magnetic disturbances) and to stabilize the filter.

#### Barometer: $R_{\text{baro}} = 1.0 \text{ m}^2$

The barometer in this system adds Gaussian noise with $\sigma = 2 \text{ Pa}$. Using the International Standard Atmosphere (ISA) model:

$$
h = 44330 \left(1 - \left(\frac{P}{P_0}\right)^{1/5.255}\right)
$$

the derivative $\frac{dh}{dP}$ at sea level is approximately $-0.0833 \text{ m/Pa}$. Thus:

$$
\sigma_h = 0.0833 \times 2 = 0.167 \text{ m} \quad \Rightarrow \quad R_{\text{true}} = 0.028 \text{ m}^2
$$

However, **$R_{\text{baro}} = 1.0 \text{ m}^2$ is used instead** (a 36× inflation). Here's why:

During boost, the position covariance $P_{zz}$ grows rapidly due to $q_{\mathbf{v}} = 0.1$. If $R_{\text{baro}} \ll P_{zz}$, the Kalman gain approaches:

$$
K \approx \frac{P[:, z]}{P_{zz}}
$$

This means the innovation $y = z_{\text{baro}} - \hat{z}$ is applied at **near-full strength** to every state via the cross-covariance $P[:, z]$. In particular, attitude corrections occur via $P[\delta\boldsymbol{\theta}, \delta p_z]$. With a 50 m baro innovation at coast entry and even 1% cross-correlation, this produces:

$$
\delta\boldsymbol{\theta} = 0.01 \times 50 = 0.5 \text{ rad} = 28°
$$

This is the "40° yaw spike" observed before tuning. Setting $R_{\text{baro}} = 1.0$ keeps $K$ well-behaved even when $P_{zz}$ is large.

**Tradeoff:** The filter now under-weights baro by 6× relative to its true accuracy. This is acceptable because GPS provides complementary altitude corrections at 5 Hz.

#### GPS: $R_{\text{GPS}} = 6.25 \text{ m}^2 \mathbf{I}_3$

The GPS simulator adds $\sigma = 2.5 \text{ m}$ noise in each axis:

$$
R_{\text{GPS}} = \begin{bmatrix}
6.25 & 0 & 0 \\
0 & 6.25 & 0 \\
0 & 0 & 6.25
\end{bmatrix}
$$

This matches the true sensor noise with no inflation.

**GPS origin averaging:** The GPS origin (the reference point for the local ENU frame) is set by averaging the first 10 GPS packets. A single packet has $\sigma = 2.5$ m noise per axis. Averaging 10 reduces this to:

$$
\sigma_{\text{origin}} = \frac{2.5}{\sqrt{10}} = 0.79 \text{ m}
$$

Without averaging, the origin noise creates a **fixed per-flight systematic offset** in every subsequent GPS measurement. Over hundreds of corrections, this contaminates the bias estimates via $P[\delta\mathbf{b}_a, \delta\mathbf{p}]$. Averaging eliminates this.

#### Magnetometer: $R_{\text{mag}} = 0.1 \text{ rad}^2$

The magnetometer measures heading with $\sigma \approx 0.32 \text{ rad} \approx 18°$. This is a rough empirical value based on typical magnetometer performance in the presence of magnetic disturbances.

---

## Software Architecture

The software is split into **5 modules** organized across C++ (high-performance core) and Python (analysis and visualization).

### Module 1: Data Ingestion & Pre-processing

**Files:** `src/DataIngestion.cpp`, `src/DataIngestion.hpp`, `src/SensorData.hpp`

**Purpose:** Read raw sensor telemetry from binary log files and parse it into structured C++ objects.

**Responsibilities:**

1. **Binary parsing:** Read the log file byte-by-byte. Each packet has a 1-byte type ID followed by a fixed-size struct:
   - `0x10`: IMU packet (8 doubles: timestamp, acc_x/y/z, gyro_x/y/z)
   - `0x20`: Barometer packet (3 doubles: timestamp, pressure_Pa, temperature_C)
   - `0x30`: GPS packet (6 doubles + metadata: timestamp, lat/lon/alt, num_satellites, fix_valid)
   - `0x40`: Magnetometer packet (4 doubles: timestamp, mag_x/y/z)

2. **Validation:** Skip malformed packets, log parse errors.

3. **Output:** A `ParsedData` struct containing four `std::vector`s (one per sensor type), sorted by timestamp.

**Data flow:**

```
Binary log file (flight_data.bin)
  ↓
DataIngestion::loadFromFile()
  ↓
ParsedData {imu_data, baro_data, gps_data, mag_data}
```

**Design note:** This module is sensor-format-agnostic. To add a new sensor (e.g., LiDAR), you only need to:

1. Define a new struct in `SensorData.hpp` (e.g., `LidarMeasurement`).
2. Add a case to the parser switch statement.
3. Add a `std::vector<LidarMeasurement>` to `ParsedData`.

### Module 2: State Estimation Engine (EKF Core)

**Files:** `src/Ekf.cpp`, `src/Ekf.hpp`

**Purpose:** The heart of the system. Implements the 16-state MEKF with bias tracking.

**Public interface:**

```cpp
class Ekf {
public:
  Ekf();  // Initialize state to zero, P to initial covariance

  void predict(const ImuMeasurement& imu);  // High-rate prediction (100 Hz)

  void updateBaro(const BaroMeasurement& baro);  // Altitude correction
  void updateGps(const GpsMeasurement& gps);     // Position correction
  void updateMag(const MagMeasurement& mag);     // Heading correction

  Eigen::Vector3d getPosition() const;
  Eigen::Vector3d getVelocity() const;
  Eigen::Quaterniond getOrientation() const;
  Eigen::Vector3d getGyroBias() const;
  Eigen::Vector3d getAccelBias() const;
  FlightPhase getPhase() const;
};
```

**Internal state:**

- `Eigen::VectorXd x_` (16×1): Nominal state vector.
- `Eigen::MatrixXd P_` (15×15): Error-state covariance.
- `bool initialized_`: Set to `true` after the first IMU packet (to initialize timestamp).
- `bool origin_set_`: Set to `true` after 10 GPS packets (origin averaging complete).
- `FlightPhaseEstimator phase_estimator_`: Tracks current flight phase.

**Key methods:**

- `predict()`: Implements the full prediction step (see [Prediction Step](#prediction-step)). Updates `x_` and `P_`.
- `updateWithMeasurement()`: Generic update logic used by all sensors. Computes Kalman gain, applies correction, updates covariance via Joseph form.
- `applyCorrection()`: Applies the 15-dimensional error-state correction to the 16-dimensional nominal state, handling the multiplicative quaternion update.
- `getProcessNoise()`: Returns $\mathbf{Q}$ based on current flight phase.

**Numerical stability features:**

- Quaternion normalization after every prediction and update.
- Joseph-form covariance update to guarantee positive-definiteness.
- Eigenvalue checks in test suite to ensure $\mathbf{P}$ remains positive semi-definite.

### Module 3: Flight Phase State Machine

**Files:** `src/FlightPhase.cpp`, `src/FlightPhase.hpp`

**Purpose:** Detect the current flight phase based on the estimated state (altitude, velocity, IMU acceleration).

**State diagram:**

```
PRE_LAUNCH ──[|acc| > 15 m/s² for 5 samples]──> BOOST
                                                   │
                                                   │ [|acc| < 12 m/s² OR t > 30s]
                                                   ↓
LANDED <──[alt < 2m, |vel_z| < 1 m/s, 20 samples]── DESCENT <─── COAST
                                                                    ↑
                                                                    │
                                        [vel_z < 0, alt > 10m] ─────┘
```

**Public interface:**

```cpp
enum class FlightPhase { PRE_LAUNCH, BOOST, COAST, DESCENT, LANDED };

class FlightPhaseEstimator {
public:
  FlightPhaseEstimator(Config cfg = Config{});

  void update(double alt_m, double vel_z_ms, double imu_accel_norm_ms2, double dt_s);

  FlightPhase getPhase() const;
  double getTimeInPhase() const;
  std::string getPhaseString() const;
};
```

**Configuration parameters:**

- `launch_accel_threshold_ms2 = 15.0`: Acceleration threshold to trigger BOOST.
- `launch_confirm_samples = 5`: Consecutive samples above threshold to confirm launch (prevents false triggers from vibration).
- `burnout_accel_threshold_ms2 = 12.0`: Acceleration below this indicates motor burnout.
- `max_burn_time_s = 30.0`: Timeout for BOOST phase (safety fallback).
- `min_altitude_for_apogee_m = 10.0`: Minimum altitude before detecting apogee (prevents false detection during vertical drop tests).
- `landed_alt_threshold_m = 2.0`, `landed_vel_threshold_ms = 1.0`, `landed_confirm_samples = 20`: Landing detection criteria.

**Design rationale:** Transition conditions use **hysteresis** (e.g., requiring 5 consecutive samples) to prevent chattering between states due to noisy measurements. The `time_in_phase_s_` counter is reset on every transition, allowing phase-dependent behavior (e.g., "if in BOOST for > 5 seconds, deploy parachute early").

### Module 4: Logging & Output

**Files:** `src/main.cpp`

**Purpose:** Orchestrate the full pipeline: load data → fuse → log results.

**Workflow:**

1. Load binary log file via `DataIngestion`.
2. Create a unified timeline by merging all sensor measurements, sorted by timestamp.
3. Initialize the EKF.
4. Loop over the timeline:
   - If event is IMU → call `predict()`.
   - If event is baro/GPS/mag → call the corresponding `update()` method.
   - After every event, log the current state to `trajectory.csv`.
5. Close the CSV file.

**CSV format:**

```
time,pos_x,pos_y,pos_z,vel_x,vel_y,vel_z,quat_w,quat_x,quat_y,quat_z,bg_x,bg_y,bg_z,ba_x,ba_y,ba_z,phase
0.000000,0.000000,0.000000,0.000000,0.000000,0.000000,0.000000,1.000000,0.000000,0.000000,0.000000,0.000000,0.000000,0.000000,0.000000,0.000000,0.000000,PRE_LAUNCH
0.010000,0.000001,0.000002,0.000512,...
```

Each row is a snapshot of the full 16-state vector plus the flight phase string.

**Why CSV?** CSV is human-readable and trivially imported into Python/MATLAB. The Infrastructure phase will replace this with HDF5 or Apache Feather for faster I/O and metadata embedding.

### Module 5: Analysis & Visualization (Python)

**Files:** `plot_results.py`

**Purpose:** Generate diagnostic plots to validate filter performance.

**Workflow:**

1. Load `trajectory.csv` into a Pandas DataFrame.
2. Generate vertical ground truth by re-running the flight physics (hardcoded to match `DataGenerator.cpp`).
3. Create a 7-panel figure:
   - **Altitude:** EKF vs truth.
   - **Velocity:** 3D velocity (vel_x, vel_y, vel_z) vs vertical truth.
   - **Horizontal drift:** pos_x and pos_y (ideal = 0 for vertical flight).
   - **Yaw:** Heading derived from quaternion.
   - **Gyro biases:** bg_x, bg_y, bg_z over time.
   - **Accel biases:** ba_x, ba_y, ba_z over time.
   - **Flight phase:** Step plot showing transitions.
4. Overlay **phase bands** (translucent background colors) on all subplots for temporal context.
5. Save to `flight_analysis.png`.

**Libraries used:**

- `pandas`: DataFrame for time-series data.
- `numpy`: Vectorized math (e.g., quaternion-to-yaw conversion).
- `matplotlib`: Plotting.

**Key function:**

```python
def quat_to_yaw_deg(qw, qx, qy, qz):
    """Extract yaw (heading) from quaternion."""
    siny = 2.0 * (qw * qz + qx * qy)
    cosy = 1.0 - 2.0 * (qy * qy + qz * qz)
    return np.degrees(np.arctan2(siny, cosy))
```

---

## Technology Stack

### C++ Core (Modules 1-4)

- **Language:** C++17
- **Build system:** CMake 3.14+
- **Linear algebra:** [Eigen 3.4.0](https://eigen.tuxfamily.org/) — header-only, highly optimized, ideal for Kalman filters.
- **Testing:** [Google Test 1.12.1](https://github.com/google/googletest) — fetched via CMake FetchContent.
- **I/O:** Standard library (`<fstream>`, `<iostream>`).

### Python Analysis (Module 5)

- **Language:** Python 3.8+
- **Libraries:**
  - `pandas`: DataFrame-based CSV loading and time-series manipulation.
  - `numpy`: Numerical operations.
  - `matplotlib`: Static 2D plots.

### Future Stack (Infrastructure Phase)

- **Config:** `yaml-cpp` for YAML configuration files.
- **Logging:** HDF5 or Apache Feather for high-performance binary logs with metadata.
- **Real-time:** ZeroMQ or UDP sockets for telemetry streaming.

---

## Directory Structure

```
.
├── CMakeLists.txt               # Top-level CMake configuration
├── README.md                    # This file
├── .gitignore                   # Ignore build/, binaries, IDE files
│
├── src/                         # C++ source code
│   ├── CMakeLists.txt           # Build config for src/
│   ├── main.cpp                 # Main executable (sensor_fusion)
│   ├── mainGenerator.cpp        # Synthetic data generator (generate_data)
│   ├── Ekf.cpp / Ekf.hpp        # EKF core
│   ├── FlightPhase.cpp / .hpp   # Flight phase state machine
│   ├── DataIngestion.cpp / .hpp # Binary log parser
│   ├── DataGenerator.cpp / .hpp # Synthetic data generator
│   └── SensorData.hpp           # Sensor data structures
│
├── test/                        # Unit tests (Google Test)
│   ├── CMakeLists.txt           # Build config for tests
│   ├── main_test.cpp            # Test runner main()
│   ├── test_Eigen.cpp           # Eigen library sanity check
│   ├── test_SensorData.cpp      # Data structure tests
│   ├── test_DataGenerator.cpp   # Synthetic data validation
│   ├── test_DataIngestion.cpp   # Binary parsing tests
│   ├── test_FlightPhase.cpp     # State machine tests
│   ├── test_EkfInit.cpp         # Initialization tests
│   ├── test_EkfPredict.cpp      # Prediction step tests
│   ├── test_EkfUpdate.cpp       # Update step tests (baro/GPS/mag)
│   ├── test_EkfMekf.cpp         # MEKF-specific properties
│   ├── test_EkfBias.cpp         # Bias tracking tests
│   └── test_EkfNumerical.cpp    # End-to-end numerical stability
│
├── plot_results.py              # Python analysis script
├── build/                       # CMake build directory (gitignored)
│   ├── src/sensor_fusion        # Main executable
│   ├── src/generate_data        # Data generator
│   ├── test/run_tests           # Test suite
│   └── trajectory.csv           # Output CSV (after running sensor_fusion)
│
└── flight_analysis.png          # Output diagnostic plot
```

---

## Building the Project

### Prerequisites

**Required:**

- **CMake 3.14+** — Install via package manager or download from [cmake.org](https://cmake.org/).
- **C++17 compiler:**
  - Linux: GCC 7+ or Clang 5+
  - macOS: Apple Clang 10+ (Xcode Command Line Tools)
  - Windows: MSVC 2017+ or MinGW-w64
- **Python 3.8+** with `pip`.

**Python dependencies:**

```bash
pip install pandas numpy matplotlib
```

### Build Steps

1. **Clone the repository:**

```bash
git clone https://github.com/your-username/rocketry-sensor-fusion.git
cd rocketry-sensor-fusion
```

2. **Create build directory:**

```bash
mkdir build
cd build
```

3. **Run CMake:**

```bash
cmake ..
```

This will:
- Download Eigen 3.4.0 and Google Test 1.12.1 via FetchContent.
- Configure the build for your platform.

4. **Compile:**

```bash
make -j$(nproc)  # Linux/macOS (use all CPU cores)
cmake --build . --config Release  # Windows
```

**Outputs:**

- `build/src/sensor_fusion` — Main executable.
- `build/src/generate_data` — Synthetic data generator.
- `build/test/run_tests` — Test suite.

---

## Running the Project

### Quick Start (Synthetic Data)

This demonstrates the full pipeline on synthetic flight data.

#### Step 1: Generate synthetic flight data

```bash
cd build
./src/generate_data
```

**Output:** `flight_data.bin` — A 300-second simulated vertical flight (10s boost @ 20 m/s², ballistic coast, parachute descent).

**What it simulates:**

- IMU: 100 Hz, adds Gaussian noise to acceleration/gyro (σ = 0.2 m/s², σ = 0.01 rad/s).
- Barometer: 10 Hz, adds pressure noise (σ = 2 Pa → ~0.17 m altitude noise).
- GPS: 5 Hz, adds position noise (σ = 2.5 m per axis).
- Magnetometer: 20 Hz, adds magnetic field noise (σ = 0.5 μT).

#### Step 2: Run the sensor fusion filter

```bash
./src/sensor_fusion
```

**Output:** `trajectory.csv` — Contains the full 16-state estimate at 100 Hz (30,000 rows for 300 seconds).

**Console output:**

```
Loading flight_data.bin...
Parsing file: flight_data.bin...
Parsing Complete.
Loaded 30000 IMU packets.
Loaded 3000 Baro packets.
Loaded 1500 GPS packets.
Loaded 6000 Mag packets.
Processing 40500 events...
EKF Initialized (MEKF, 15-state error covariance).
GPS Origin Set (averaged over 10 packets).
Done. Results written to 'trajectory.csv'.
```

#### Step 3: Generate diagnostic plots

```bash
cd ..  # Return to project root
python3 plot_results.py
```

**Output:** `flight_analysis.png` — 7-panel diagnostic plot.

**What to look for:**

- **Altitude:** Blue line (EKF) should overlap the black dashed line (truth) almost perfectly.
- **Velocity:** vel_z should peak at ~200 m/s during boost, then decay. vel_x/vel_y should hover near zero.
- **Horizontal drift:** ±5 m, symmetric, GPS-noise limited.
- **Yaw:** Small transient at coast entry (~5°), then ±1° noise.
- **Biases:** Gyro biases converge near zero. Accel biases settle at ~±0.2 m/s² (within filter's 1-sigma uncertainty).
- **Flight phase:** Clean transitions PRE_LAUNCH → BOOST → COAST → DESCENT (no LANDED, flight time exceeds 300s).

### Workflow Explanation

The three executables form a pipeline:

```
generate_data → flight_data.bin → sensor_fusion → trajectory.csv → plot_results.py → flight_analysis.png
```

**Typical use case for real flights:**

1. Rocket logs raw sensor data to `flight_data.bin` during flight.
2. After landing, transfer the file to your laptop.
3. Run `sensor_fusion` to produce `trajectory.csv`.
4. Run `plot_results.py` to visualize.
5. Share `flight_analysis.png` with your team for performance review.

---

## Test Suite

The project has **55 unit tests** organized into 10 test files, covering initialization, dynamics, measurements, numerical stability, and end-to-end validation.

### Test Organization

Tests are grouped by subsystem:

| File                       | Tests | Focus                                                |
|----------------------------|-------|------------------------------------------------------|
| `test_Eigen.cpp`           | 1     | Eigen library sanity check                           |
| `test_SensorData.cpp`      | 9     | Data structures: default values, size stability      |
| `test_DataGenerator.cpp`   | 14    | Synthetic data: packet counts, timestamps, physics   |
| `test_DataIngestion.cpp`   | 1     | Binary parsing correctness                           |
| `test_FlightPhase.cpp`     | 11    | State machine transitions, hysteresis, counters      |
| `test_EkfInit.cpp`         | 11    | Initialization: zero state, unit quaternion, P PSD   |
| `test_EkfPredict.cpp`      | 13    | Prediction: gravity cancellation, integration, Q     |
| `test_EkfUpdate.cpp`       | 19    | Updates: baro/GPS/mag innovation, NaN rejection      |
| `test_EkfMekf.cpp`         | 9     | MEKF properties: 15×15 P, quaternion normalization   |
| `test_EkfBias.cpp`         | 8     | Bias tracking: observability, drift reduction        |
| `test_EkfNumerical.cpp`    | 4     | End-to-end: symmetry, PSD, convergence, NaN/Inf      |

### Running Tests

```bash
cd build
./test/run_tests
```

**Expected output:**

```
[==========] Running 55 tests from 11 test suites.
...
[  PASSED  ] 55 tests.
```

**To run a specific test file:**

```bash
./test/run_tests --gtest_filter=EkfPredictTest.*
```

**To see detailed test output:**

```bash
./test/run_tests --gtest_verbose
```

### Test Coverage Summary

**Coverage highlights:**

- **Initialization:** All 16 states initialized correctly, P is positive-definite.
- **Prediction:** Gravity-only input produces zero net acceleration. Vertical thrust integrates correctly. Quaternion remains unit norm after 500 steps.
- **Covariance dynamics:** P grows after predict (process noise), shrinks after update (measurement info). Remains symmetric and PSD throughout.
- **Measurements:** Baro/GPS/mag all drive state toward measurement. NaN/invalid data rejected. Convergence validated (e.g., altitude settles within 15m of target after 800 baro updates).
- **MEKF:** Quaternion norm exactly 1.0 (to machine precision) after every predict/update. P is 15×15, not 16×16.
- **Bias tracking:** Injecting +1 m/s² bias on acc_z, then correcting with baro, drives ba_z estimate positive (directionally correct). Bias-aware filter drifts less than naive integration.
- **Numerical stability:** Full 300-second flight with 40,500 events produces no NaN/Inf. P remains symmetric and PSD. Altitude error < 50m (validates against ground truth).

**What's NOT tested (yet):**

- Multi-threaded predict/update (single-threaded only for now).
- Real hardware edge cases (GPS dropouts, magnetometer spikes, accelerometer saturation).
- Long-duration flights (> 1 hour) where floating-point roundoff might accumulate.

These will be addressed in Phase 3 (Filter Validation & Diagnostics).

---

## Current Status (Phase 1 Complete)

**Implemented and validated:**

✅ 16-state MEKF with bias tracking (position, velocity, orientation, gyro bias, accel bias)  
✅ Flight-phase-adaptive process noise (PRE_LAUNCH/BOOST/COAST/DESCENT/LANDED)  
✅ GPS origin averaging (10 packets → σ = 0.79 m per axis)  
✅ Barometer, GPS, magnetometer update steps  
✅ Joseph-form covariance update for numerical stability  
✅ Quaternion normalization enforcement  
✅ Binary log ingestion (IMU/baro/GPS/mag)  
✅ CSV output logging (16 states + phase)  
✅ Python diagnostic plots (7-panel flight analysis)  
✅ Synthetic data generator (300s vertical flight)  
✅ 55 unit tests (initialization, dynamics, measurements, end-to-end)  
✅ CMake build system with Eigen/GTest auto-fetch  

**Validated performance:**

- **Altitude tracking:** EKF overlaps ground truth within plotting resolution (<1 m error).
- **Velocity:** Clean estimate throughout flight, horizontal components near zero.
- **Attitude:** Yaw transients < 5°, recovers within seconds, ±1° steady-state.
- **Biases:** Gyro biases converge near zero. Accel biases settle within 1-sigma initial uncertainty (±0.3 m/s²).
- **Numerical health:** No NaN/Inf after 300-second flight. Covariance remains PSD.

**Known minor issues (deferred to Phase 3):**

- Gyro bias bg_x transiently dips to -0.06 rad/s during boost, recovers by t=50s. Does not affect position/altitude estimates. Root cause: q_bg 10× larger during boost than coast. Will be addressed via NIS/NEES diagnostic tuning.

---

## Roadmap: Open Work

### Phase 2: Infrastructure

**Goal:** Production-ready data pipeline, configuration management, and hardware interfacing foundations.

#### 2.1 YAML Configuration System

**Status:** Not started.

**Objective:** Externalize all tunable parameters (Q, R, phase thresholds, GPS averaging count) to a single `config.yaml` file, eliminating hardcoded constants in C++.

**Design:**

- Use `yaml-cpp` library for parsing.
- Define schema:
  ```yaml
  ekf:
    initial_covariance:
      position: 0.1
      velocity: 0.1
      attitude: 0.1
      gyro_bias: 0.01
      accel_bias: 0.1
    
    process_noise:
      pre_launch: {pos: 1e-6, vel: 1e-4, att: 1e-5, bg: 1e-8, ba: 1e-7}
      boost:      {pos: 1e-4, vel: 0.1,  att: 5e-4, bg: 1e-7, ba: 1e-7}
      coast:      {pos: 1e-5, vel: 0.01, att: 5e-4, bg: 1e-8, ba: 1e-7}
      descent:    {pos: 1e-5, vel: 1e-3, att: 5e-4, bg: 1e-8, ba: 1e-7}
      landed:     {pos: 1e-8, vel: 1e-6, att: 1e-6, bg: 1e-9, ba: 1e-9}
    
    measurement_noise:
      baro: 1.0
      gps: 6.25
      mag: 0.1
    
    gps_origin_samples: 10
  
  flight_phase:
    launch_accel_threshold: 15.0
    launch_confirm_samples: 5
    burnout_accel_threshold: 12.0
    # ... (all FlightPhaseEstimator::Config parameters)
  
  simulation:
    duration_sec: 300.0
    dt_sec: 0.01
    burn_time_sec: 10.0
    burn_acceleration: 20.0
  ```

- Load config in `main()`, pass to `Ekf` and `DataGenerator` constructors.
- Python script also reads `config.yaml` to regenerate ground truth (eliminates hardcoded BURN_TIME/BURN_ACC duplication).

**Validation:** All existing tests must pass with default config. Add test for config schema parsing.

**Acceptance criteria:** Filter behavior unchanged when config matches current hardcoded values. Zero code changes required to re-tune filter for different vehicle.

#### 2.2 HDF5 or Feather Logging

**Status:** Not started.

**Objective:** Replace CSV with a high-performance binary format that supports metadata, fast random access, and >10× faster I/O.

**Design:**

- **Option A: HDF5**
  - Pro: Industry standard, widely supported, self-describing.
  - Con: Large library dependency, complex API.
  
- **Option B: Apache Feather (IPC)**
  - Pro: Lightweight, zero-copy reads in Python/Pandas, fast.
  - Con: Less mature, no built-in compression.

**Recommendation:** Start with Feather for speed, migrate to HDF5 if metadata/compression becomes critical.

**Schema:**

```
/trajectory
  ├── time          (float64 array)
  ├── position      (Nx3 float64 array)
  ├── velocity      (Nx3 float64 array)
  ├── quaternion    (Nx4 float64 array)
  ├── gyro_bias     (Nx3 float64 array)
  ├── accel_bias    (Nx3 float64 array)
  ├── covariance    (Nx15x15 float64 array, optional)
  └── phase         (N int8 array, enum)

/metadata
  ├── vehicle_name  (string)
  ├── flight_date   (string)
  ├── config_yaml   (embedded YAML text)
  └── git_commit    (string, for reproducibility)
```

**Validation:** Benchmark I/O speed. Target: <100ms to write 300s of data (vs. ~500ms for CSV).

**Acceptance criteria:** Python analysis loads Feather file in <50ms. All plots unchanged vs CSV baseline.

#### 2.3 Real-Time Telemetry Stream Parser

**Status:** Not started.

**Objective:** Ingest live telemetry from a radio link or serial port, feeding data directly to the EKF without intermediate file writes.

**Design:**

- Implement `TelemetryStream` class:
  ```cpp
  class TelemetryStream {
  public:
    void connect(const std::string& port);  // "/dev/ttyUSB0" or "udp://0.0.0.0:5000"
    bool poll(SensorMeasurement& out, int timeout_ms);
  };
  ```

- Modify `main.cpp` to support two modes:
  - **Post-flight mode:** `./sensor_fusion flight_data.bin` (current behavior).
  - **Real-time mode:** `./sensor_fusion --stream /dev/ttyUSB0` (new).

- In real-time mode, loop:
  ```cpp
  while (stream.poll(measurement, 100)) {
    if (measurement.type == IMU) filter.predict(...);
    // ...
    logger.append(filter.getState());
  }
  ```

**Protocols to support:**

- Serial (UART): Most common for radios.
- UDP sockets: For WiFi/Ethernet.
- (Future) ZeroMQ: For distributed systems.

**Validation:** Replay a recorded binary log through a virtual serial port (e.g., `socat`). Verify filter output matches post-flight processing.

**Acceptance criteria:** Real-time mode produces identical trajectory to post-flight mode when given the same input data (bit-for-bit determinism).

---

### Phase 3: Filter Validation & Diagnostics

**Goal:** Systematic filter health monitoring, performance characterization, and tuning automation.

#### 3.1 Normalized Innovation Squared (NIS) Monitoring

**Status:** Not started.

**Objective:** Detect when measurement residuals are inconsistent with the filter's reported uncertainty, indicating Q/R mistuning or model mismatch.

**Theory:** The **innovation** $\mathbf{y} = \mathbf{z} - \mathbf{h}(\mathbf{x})$ should be zero-mean Gaussian with covariance $\mathbf{S} = \mathbf{H}\mathbf{P}\mathbf{H}^T + \mathbf{R}$. The **Normalized Innovation Squared** is:

$$
\epsilon_k = \mathbf{y}_k^T \mathbf{S}_k^{-1} \mathbf{y}_k
$$

Under correct tuning, $\epsilon_k \sim \chi^2(m)$ where $m$ is the measurement dimension. Over a batch of N measurements:

$$
\bar{\epsilon} = \frac{1}{N} \sum_{k=1}^N \epsilon_k \approx m
$$

**Implementation:**

- Log $\epsilon_k$ for every baro/GPS/mag update.
- Compute moving average $\bar{\epsilon}$ over 100-sample windows.
- Flag warnings if $\bar{\epsilon} > 2m$ (under-confident) or $\bar{\epsilon} < 0.5m$ (over-confident).

**Validation:** With correct R, $\bar{\epsilon}_{\text{GPS}} \approx 3$, $\bar{\epsilon}_{\text{baro}} \approx 1$. Plot time-series to visualize consistency.

**Acceptance criteria:** NIS stays within 2σ bounds for 95% of the flight. Outliers correlate with known events (e.g., GPS multipath during low altitude).

#### 3.2 Normalized Estimation Error Squared (NEES)

**Status:** Not started.

**Objective:** Validate that the filter's covariance $\mathbf{P}$ accurately reflects the true estimation error, using ground truth from simulation.

**Theory:** The **estimation error** $\mathbf{e} = \mathbf{x}_{\text{true}} - \mathbf{x}_{\text{est}}$ should satisfy:

$$
\mathbf{e}^T \mathbf{P}^{-1} \mathbf{e} \sim \chi^2(n)
$$

where $n = 15$ (error-state dimension). Average over N timesteps:

$$
\bar{\epsilon}_{\text{NEES}} = \frac{1}{N} \sum_{k=1}^N \mathbf{e}_k^T \mathbf{P}_k^{-1} \mathbf{e}_k \approx 15
$$

**Implementation:**

- Modify `DataGenerator` to output ground truth state alongside noisy measurements.
- Compute NEES at each timestep.
- Plot NEES vs time, overlay $\chi^2(15)$ confidence bounds.

**Validation:** NEES should hover around 15. Values consistently >> 15 indicate over-confidence (P too small). Values << 15 indicate under-confidence (P too large).

**Acceptance criteria:** 90% of NEES values fall within 95% $\chi^2$ bounds. Outliers confined to phase transitions (brief model mismatch).

#### 3.3 Monte Carlo Tuning Automation

**Status:** Not started.

**Objective:** Automate Q/R parameter search to minimize NEES deviation, replacing manual trial-and-error.

**Design:**

1. Define search space: e.g., $q_{\mathbf{v}} \in [0.01, 1.0]$, $R_{\text{baro}} \in [0.1, 10.0]$.
2. Run 100 Monte Carlo simulations per parameter set (different noise seeds).
3. Compute mean NEES and NIS for each run.
4. Objective function: $J = |\bar{\epsilon}_{\text{NEES}} - 15| + \sum_i |\bar{\epsilon}_{\text{NIS},i} - m_i|$.
5. Use Bayesian optimization (e.g., Gaussian Process) to minimize $J$.

**Validation:** Optimal parameters should match hand-tuned values ±20%.

**Acceptance criteria:** Auto-tuned filter achieves NEES within 5% of target, NIS within 10%, across 1000 simulation trials.

#### 3.4 Observability Analysis

**Status:** Not started.

**Objective:** Compute the observability Gramian to verify that all 15 error states are observable given the sensor suite.

**Theory:** A state is **observable** if it can be uniquely determined from the measurement history. For linear systems:

$$
\mathcal{O} = \int_0^T \Phi(T, \tau)^T \mathbf{H}^T \mathbf{H} \Phi(T, \tau) \, d\tau
$$

The system is observable if $\mathcal{O}$ is full-rank.

**Implementation:**

- Discretize the integral over a 10-second window.
- Compute eigenvalues of $\mathcal{O}$.
- States with near-zero eigenvalues are unobservable.

**Validation:** With current sensor suite, expect position/velocity/attitude fully observable. Biases observable but weakly (eigenvalues 100–1000× smaller).

**Acceptance criteria:** All states have eigenvalues > 1e-6 (no rank deficiency). Document which phases have strongest bias observability (BOOST for gyro bias, COAST for accel bias).

---

### Phase 4: Real Hardware Integration

**Goal:** Field-test the filter on actual rocket flights with commercial off-the-shelf sensors.

#### 4.1 NMEA GPS Parser

**Status:** Not started.

**Objective:** Parse standard NMEA sentences ($GPGGA, $GPRMC) from UART GPS modules.

**Design:**

- Add `NmeaParser` class:
  ```cpp
  class NmeaParser {
  public:
    bool parseLine(const std::string& line, GpsMeasurement& out);
  };
  ```

- Extract lat/lon/alt from `$GPGGA`. Convert DMS (degrees-minutes-seconds) to decimal degrees.
- Validate checksum. Reject malformed sentences.

**Validation:** Feed recorded NMEA logs from a u-blox NEO-M8N. Verify position accuracy vs Google Maps.

**Acceptance criteria:** Parser handles 10,000 NMEA sentences without crash. Lat/lon/alt match reference within 1e-6°.

#### 4.2 Custom IMU Binary Protocol

**Status:** Not started.

**Objective:** Support the binary packet format output by the actual flight computer (e.g., STM32 with MPU6050).

**Design:**

- Define packet structure (e.g., 32-byte fixed-length):
  ```
  [SYNC (2 bytes)][TYPE (1 byte)][PAYLOAD (28 bytes)][CRC (1 byte)]
  ```
- Implement framing: search for `SYNC = 0xAA55`, validate CRC (CRC-8).
- Parse payload based on TYPE: `0x01 = IMU`, `0x02 = BARO`, etc.

**Validation:** Replay real flight logs. Compare parsed data to USB debug logs from the same flight.

**Acceptance criteria:** Parser extracts 99.9% of packets with zero false positives (incorrect CRC rejected).

#### 4.3 Magnetometer Calibration

**Status:** Not started.

**Objective:** Compensate for hard-iron and soft-iron distortions in magnetometer data.

**Theory:** Raw magnetometer reads $\mathbf{m}_{\text{raw}}$. True field is:

$$
\mathbf{m}_{\text{true}} = \mathbf{A}^{-1} (\mathbf{m}_{\text{raw}} - \mathbf{b})
$$

where $\mathbf{b}$ is hard-iron bias and $\mathbf{A}$ is soft-iron scaling matrix.

**Calibration procedure:**

1. Rotate rocket through all orientations (slow tumble in all axes).
2. Collect 1000 magnetometer samples.
3. Fit ellipsoid to point cloud (least-squares).
4. Extract $\mathbf{A}$ and $\mathbf{b}$ from ellipsoid parameters.

**Validation:** Before calibration, $\|\mathbf{m}_{\text{raw}}\|$ varies 20–50 μT. After calibration, $\|\mathbf{m}_{\text{true}}\|$ should be constant to ±2 μT.

**Acceptance criteria:** Yaw estimate drift < 5° over 60 seconds of stationary data (previously 15–30°).

#### 4.4 GPS Dropout Handling

**Status:** Not started.

**Objective:** Gracefully handle temporary GPS loss (e.g., during high-g boost or at apogee).

**Design:**

- Track time since last valid GPS fix: `double time_since_gps = t - t_last_gps`.
- If `time_since_gps > 10.0`, inflate $\mathbf{P}$ position block:
  ```cpp
  P.block<3,3>(0,0) += dt * Q_gps_loss;
  ```
- When GPS reconnects, perform sanity check: if innovation > 100m, reject packet (likely GPS glitch).

**Validation:** Simulate 20-second GPS dropout during coast. Filter should continue producing state with degraded accuracy (position uncertainty grows to ~50m). On reconnect, filter snaps back to GPS within 5 samples.

**Acceptance criteria:** No divergence or crash during dropout. Reconnect smooth (no 100m+ jumps).

#### 4.5 Flight Test (Vertical Launch)

**Status:** Not started.

**Objective:** Deploy the filter on a real rocket, compare post-flight EKF trajectory to dual-GPS ground truth.

**Procedure:**

1. Equip rocket with dual redundant GPS units (one for navigation, one for ground truth).
2. Launch to ~1 km apogee.
3. Post-flight, process navigation GPS through EKF.
4. Compare EKF position to ground-truth GPS.

**Success metrics:**

- Apogee altitude within 5% of ground truth.
- Horizontal drift < 20m CEP (Circular Error Probable).
- No filter divergence or crash during flight.

**Acceptance criteria:** Position RMSE < 10m throughout flight. Velocity RMSE < 2 m/s.

---

### Phase 5: Advanced Features

**Goal:** Extended capabilities for complex missions and edge cases.

#### 5.1 Sensor Fault Detection

**Status:** Not started.

**Objective:** Automatically detect and isolate faulty sensors (e.g., GPS spoofing, magnetometer saturation, accelerometer clipping).

**Techniques:**

- **Residual monitoring:** If innovation $\|\mathbf{y}\| > 5\sqrt{\text{trace}(\mathbf{S})}$, flag outlier.
- **Redundancy voting:** If two IMUs disagree by > 3σ, one is faulty.
- **Plausibility checks:** GPS altitude > 10 km on a rocket rated for 3 km → reject.

**Design:**

- Add `SensorHealthMonitor` class.
- Per-sensor state: `HEALTHY`, `SUSPECT`, `FAILED`.
- If sensor marked `FAILED`, exclude from updates.

**Validation:** Inject faults into synthetic data (e.g., GPS reports altitude = -1000m at t=20s). Filter should reject bad measurement, continue without divergence.

**Acceptance criteria:** 100% fault isolation rate on simulated single-sensor failures. False positive rate < 1% on clean data.

#### 5.2 Wind Estimation

**Status:** Not started.

**Objective:** Estimate horizontal wind velocity to improve trajectory prediction for guided rockets.

**Augmented state:**

$$
\mathbf{x}_{\text{aug}} = \begin{bmatrix} \mathbf{x} \\ \mathbf{w}_{\text{wind}} \end{bmatrix}, \quad \mathbf{w}_{\text{wind}} \in \mathbb{R}^2 \text{ (wind E-N)}
$$

**Dynamics:** Wind is modeled as slow random walk: $\dot{\mathbf{w}}_{\text{wind}} = \mathbf{n}_w$.

**Observability:** Wind is observable from GPS horizontal velocity vs airspeed (requires airspeed sensor or drag model).

**Validation:** Simulate 5 m/s crosswind during ascent. Estimate $\mathbf{w}_{\text{wind}}$ to within 1 m/s by apogee.

**Acceptance criteria:** Wind estimate converges within 30 seconds. Impact point prediction error reduced from 50m → 10m.

#### 5.3 Drag Coefficient Estimation

**Status:** Not started.

**Objective:** Estimate the rocket's drag coefficient $C_D$ in real-time, enabling accurate apogee prediction.

**Augmented state:**

$$
\mathbf{x}_{\text{aug}} = \begin{bmatrix} \mathbf{x} \\ C_D \end{bmatrix}
$$

**Dynamics:** During coast, acceleration should be $\mathbf{a} = -\mathbf{g} - \frac{1}{2} \rho v^2 C_D A / m \, \hat{\mathbf{v}}$.

If measured $\mathbf{a}$ differs from prediction, the Kalman filter attributes part of the error to $C_D$ via the cross-covariance $\mathbf{P}[\delta C_D, \delta \mathbf{v}]$.

**Validation:** Simulate rocket with $C_D = 0.5$. Initialize filter with $C_D = 0.3$. Estimate should converge to $0.5 \pm 0.05$ by apogee.

**Acceptance criteria:** Apogee prediction error < 2% after 10 seconds of coast.

#### 5.4 Trajectory Optimization Interface

**Status:** Not started.

**Objective:** Output the state estimate and covariance at 10 Hz to a guidance algorithm for powered descent or trajectory shaping.

**Design:**

- Add `StateObserver` callback:
  ```cpp
  class StateObserver {
  public:
    virtual void onStateUpdate(const Eigen::VectorXd& x, const Eigen::MatrixXd& P, double t) = 0;
  };
  ```

- Guidance algorithm subscribes:
  ```cpp
  filter.addObserver(&my_guidance);
  ```

**Use case:** Powered landing (e.g., SpaceX Falcon 9 style). Guidance computes thrust vector to null horizontal velocity by touchdown.

**Validation:** Simulate ballistic descent with 100m horizontal velocity. Guidance should plan thrust profile to land within 5m of target.

**Acceptance criteria:** Interface latency < 5ms. Guidance algorithm achieves soft landing 95% of trials.

---

## Author

**Project Maintainer:** Arnav Mohnot | UW SARP  
**Contact:** arnav.mohnot@gmail.com | mohnota@uw.edu  
**Documentation Version:** 2.0
