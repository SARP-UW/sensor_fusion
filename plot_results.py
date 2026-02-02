import pandas as pd
import matplotlib.pyplot as plt
import numpy as np

def get_yaw_from_quat(w, x, y, z):
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    return np.degrees(np.arctan2(siny_cosp, cosy_cosp))

def generate_vertical_ground_truth(duration=120.0, dt=0.01):
    """
    Re-creates the ideal vertical physics from DataGenerator.cpp
    so we can compare EKF vs Truth.
    """
    times = np.arange(0, duration, dt)
    true_pos_z = []
    true_vel_z = []
    
    # Physics State
    z = 0.0
    v = 0.0
    
    # Physics Parameters (Matched to DataGenerator.cpp)
    burn_time = 10.0 
    burn_acc = 20.0
    gravity = -9.81
    descent_vel = -5.0
    
    for t in times:
        if t < burn_time:
            # Phase 1: Burn
            acc = burn_acc
        elif v > 0:
            # Phase 2: Coast (Gravity only)
            acc = gravity
        elif z > 0:
            # Phase 3: Descent (Parachute)
            acc = 0.0
            v = descent_vel 
        else:
            # Landed
            acc = 0.0
            v = 0.0
            z = 0.0
            
        # Euler Integration
        if z > 0 or t < burn_time:
            v += acc * dt
            z += v * dt
            
        true_pos_z.append(z)
        true_vel_z.append(v)
        
    return times, np.array(true_pos_z), np.array(true_vel_z)

def plot_trajectory():
    # 1. Load the EKF Output
    try:
        df = pd.read_csv('build/trajectory.csv') 
        df.columns = df.columns.str.strip()
    except FileNotFoundError:
        print("Error: Could not find 'build/trajectory.csv'.")
        return

    # 2. Generate Vertical Ground Truth
    max_time = df['time'].max()
    gt_time, gt_pos, gt_vel = generate_vertical_ground_truth(duration=max_time + 1.0)

    # 3. Plotting
    fig, axes = plt.subplots(4, 1, figsize=(10, 16), sharex=True)
    
    # --- Plot 1: Altitude ---
    ax = axes[0]
    ax.plot(gt_time, gt_pos, 'k--', label='Vertical Truth (Ideal)', linewidth=1.5)
    ax.plot(df['time'], df['pos_z'], 'b-', label='EKF Altitude', linewidth=2)
    ax.set_ylabel('Altitude (m)')
    ax.set_title('EKF Estimated Trajectory vs Truth (10s Burn)')
    ax.legend(loc='upper right')
    ax.grid(True)

    # --- Plot 2: Velocity ---
    ax = axes[1]
    ax.plot(gt_time, gt_vel, 'k--', label='Vertical Truth', linewidth=1.5)
    ax.plot(df['time'], df['vel_z'], 'r-', label='EKF Vertical Vel', linewidth=2)
    ax.set_ylabel('Vert Velocity (m/s)')
    ax.legend(loc='upper right')
    ax.grid(True)

    # --- Plot 3: Horizontal Position (The Drift) ---
    # We plot "0.0" as the ideal path, so you can see the drift deviation.
    ax = axes[2]
    ax.plot(df['time'], df['pos_x'], 'g-', label='Pos X (Est)', alpha=0.7)
    ax.plot(df['time'], df['pos_y'], 'orange', label='Pos Y (Est)', alpha=0.7)
    ax.axhline(0, color='k', linestyle='--', label='Ideal Vertical Path (0.0)')
    ax.set_ylabel('Horizontal Position (m)')
    ax.set_title('Horizontal Drift from Vertical (Due to Gyro Noise)')
    ax.legend(loc='upper right')
    ax.grid(True)

    # --- Plot 4: Magnetometer Correction (Yaw) ---
    if 'quat_w' in df.columns:
        df['yaw_deg'] = df.apply(
            lambda row: get_yaw_from_quat(row['quat_w'], row['quat_x'], row['quat_y'], row['quat_z']), 
            axis=1
        )
        ax = axes[3]
        ax.plot(df['time'], df['yaw_deg'], 'm-', label='Yaw (Heading)', linewidth=1.5)
        ax.axhline(0, color='k', linestyle='--', label='Truth (0 deg)')
        ax.set_ylabel('Heading (deg)')
        ax.set_xlabel('Time (s)')
        ax.legend(loc='upper right')
        ax.grid(True)

    plt.tight_layout()
    plt.savefig('flight_analysis.png')
    print("Plot saved to 'flight_analysis.png'")
    plt.show()

if __name__ == "__main__":
    plot_trajectory()