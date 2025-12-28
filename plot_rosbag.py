#!/usr/bin/env python3
"""
ROS Bag Plotting Script for Turtlebot Simulation
Plots state tracking, reference trajectory, and control commands
"""

import rosbag
import matplotlib.pyplot as plt
import numpy as np
from pathlib import Path
import argparse
import sys


def plot_rosbag_data(bag_file):
    """
    Plot data from rosbag containing turtlebot simulation data
    
    Topics expected:
    - /turtlebot/state: x, y, w (actual state)
    - /turtlebot/reference: x, y (reference trajectory)
    - /control_commands: v_cmd, omega_cmd
    """
    
    # Check if bag file exists
    if not Path(bag_file).exists():
        print(f"Error: Bag file '{bag_file}' not found!")
        return
    
    print(f"Reading rosbag: {bag_file}")
    
    # Data storage
    data = {
        'state': {'time': [], 'x': [], 'y': [], 'w': []},
        'reference': {'time': [], 'x': [], 'y': []},
        'control': {'time': [], 'v_cmd': [], 'omega_cmd': []}
    }
    
    # Read bag file
    try:
        bag = rosbag.Bag(bag_file)
        
        # Get start time for relative timing
        start_time = None
        
        # Read state messages
        for topic, msg, t in bag.read_messages(topics=['/turtlebot/state']):
            if start_time is None:
                start_time = t.to_sec()
            data['state']['time'].append(t.to_sec() - start_time)
            data['state']['x'].append(msg.x)
            data['state']['y'].append(msg.y)
            data['state']['w'].append(msg.w)
        
        # Read reference messages
        for topic, msg, t in bag.read_messages(topics=['/turtlebot/reference']):
            if start_time is None:
                start_time = t.to_sec()
            data['reference']['time'].append(t.to_sec() - start_time)
            data['reference']['x'].append(msg.x)
            data['reference']['y'].append(msg.y)
        
        # Read control commands
        for topic, msg, t in bag.read_messages(topics=['/control_commands']):
            if start_time is None:
                start_time = t.to_sec()
            data['control']['time'].append(t.to_sec() - start_time)
            data['control']['v_cmd'].append(msg.v_cmd)
            data['control']['omega_cmd'].append(msg.omega_cmd)
        
        bag.close()
        
    except Exception as e:
        print(f"Error reading bag file: {e}")
        return
    
    # Check if we have data
    if len(data['state']['time']) == 0:
        print("Warning: No state data found in bag file!")
        return
    
    print(f"Loaded {len(data['state']['time'])} state messages")
    print(f"Loaded {len(data['reference']['time'])} reference messages")
    print(f"Loaded {len(data['control']['time'])} control messages")
    
    # Create plots
    fig = plt.figure(figsize=(15, 10))
    
    # Plot 1: X-Y Trajectory (Tracking Plot)
    ax1 = plt.subplot(2, 2, 1)
    ax1.scatter(data['state']['x'], data['state']['y'], 
                c=data['state']['time'], cmap='viridis', 
                label='Actual Trajectory', s=10, alpha=0.6)
    if len(data['reference']['x']) > 0:
        ax1.plot(data['reference']['x'], data['reference']['y'], 
                 'r-', linewidth=2, label='Reference Trajectory', alpha=0.7)
    ax1.set_xlabel('X Position (m)', fontsize=12)
    ax1.set_ylabel('Y Position (m)', fontsize=12)
    ax1.set_title('2D Trajectory: Actual vs Reference', fontsize=14, fontweight='bold')
    ax1.legend()
    ax1.grid(True, alpha=0.3)
    ax1.axis('equal')
    cbar = plt.colorbar(ax1.collections[0], ax=ax1)
    cbar.set_label('Time (s)', fontsize=10)
    
    # Plot 2: X position vs Time
    ax2 = plt.subplot(2, 2, 2)
    ax2.plot(data['state']['time'], data['state']['x'], 
             'b-', linewidth=1.5, label='X (actual)', alpha=0.8)
    if len(data['reference']['x']) > 0:
        ax2.plot(data['reference']['time'], data['reference']['x'], 
                 'r--', linewidth=2, label='X_ref', alpha=0.7)
    ax2.set_xlabel('Time (s)', fontsize=12)
    ax2.set_ylabel('X Position (m)', fontsize=12)
    ax2.set_title('X Position Tracking', fontsize=14, fontweight='bold')
    ax2.legend()
    ax2.grid(True, alpha=0.3)
    
    # Plot 3: Y position vs Time
    ax3 = plt.subplot(2, 2, 3)
    ax3.plot(data['state']['time'], data['state']['y'], 
             'b-', linewidth=1.5, label='Y (actual)', alpha=0.8)
    if len(data['reference']['y']) > 0:
        ax3.plot(data['reference']['time'], data['reference']['y'], 
                 'r--', linewidth=2, label='Y_ref', alpha=0.7)
    ax3.set_xlabel('Time (s)', fontsize=12)
    ax3.set_ylabel('Y Position (m)', fontsize=12)
    ax3.set_title('Y Position Tracking', fontsize=14, fontweight='bold')
    ax3.legend()
    ax3.grid(True, alpha=0.3)
    
    # Plot 4: Control Commands vs Time
    ax4 = plt.subplot(2, 2, 4)
    if len(data['control']['v_cmd']) > 0:
        ax4_twin = ax4.twinx()
        
        line1 = ax4.plot(data['control']['time'], data['control']['v_cmd'], 
                         'g-', linewidth=1.5, label='v_cmd (linear)', alpha=0.8)
        line2 = ax4_twin.plot(data['control']['time'], data['control']['omega_cmd'], 
                              'm-', linewidth=1.5, label='ω_cmd (angular)', alpha=0.8)
        
        ax4.set_xlabel('Time (s)', fontsize=12)
        ax4.set_ylabel('Linear Velocity v_cmd (m/s)', fontsize=12, color='g')
        ax4_twin.set_ylabel('Angular Velocity ω_cmd (rad/s)', fontsize=12, color='m')
        ax4.set_title('Control Commands', fontsize=14, fontweight='bold')
        
        ax4.tick_params(axis='y', labelcolor='g')
        ax4_twin.tick_params(axis='y', labelcolor='m')
        
        # Combine legends
        lines = line1 + line2
        labels = [l.get_label() for l in lines]
        ax4.legend(lines, labels, loc='upper right')
        
        ax4.grid(True, alpha=0.3)
    else:
        ax4.text(0.5, 0.5, 'No control command data', 
                ha='center', va='center', transform=ax4.transAxes)
    
    plt.tight_layout()
    
    # Save figure
    output_file = bag_file.replace('.bag', '_plots.png')
    plt.savefig(output_file, dpi=300, bbox_inches='tight')
    print(f"\nPlots saved to: {output_file}")
    
    # Show plots
    plt.show()
    
    # Print statistics
    print("\n" + "="*60)
    print("TRACKING STATISTICS")
    print("="*60)
    
    if len(data['reference']['x']) > 0:
        # Calculate tracking errors (interpolate reference to match state timestamps)
        ref_x_interp = np.interp(data['state']['time'], 
                                  data['reference']['time'], 
                                  data['reference']['x'])
        ref_y_interp = np.interp(data['state']['time'], 
                                  data['reference']['time'], 
                                  data['reference']['y'])
        
        x_error = np.array(data['state']['x']) - ref_x_interp
        y_error = np.array(data['state']['y']) - ref_y_interp
        position_error = np.sqrt(x_error**2 + y_error**2)
        
        print(f"X Error - Mean: {np.mean(x_error):.4f} m, "
              f"Std: {np.std(x_error):.4f} m, "
              f"Max: {np.max(np.abs(x_error)):.4f} m")
        print(f"Y Error - Mean: {np.mean(y_error):.4f} m, "
              f"Std: {np.std(y_error):.4f} m, "
              f"Max: {np.max(np.abs(y_error)):.4f} m")
        print(f"Position Error - Mean: {np.mean(position_error):.4f} m, "
              f"Max: {np.max(position_error):.4f} m")
    
    if len(data['control']['v_cmd']) > 0:
        print(f"\nLinear Velocity (v_cmd) - Mean: {np.mean(data['control']['v_cmd']):.4f} m/s, "
              f"Range: [{np.min(data['control']['v_cmd']):.4f}, "
              f"{np.max(data['control']['v_cmd']):.4f}]")
        print(f"Angular Velocity (ω_cmd) - Mean: {np.mean(data['control']['omega_cmd']):.4f} rad/s, "
              f"Range: [{np.min(data['control']['omega_cmd']):.4f}, "
              f"{np.max(data['control']['omega_cmd']):.4f}]")
    
    print("="*60)


def find_latest_rosbag(directory):
    """Find the most recent rosbag file in the directory"""
    bag_path = Path(directory)
    if not bag_path.exists():
        return None
    
    bag_files = sorted(bag_path.glob('*.bag'), key=lambda p: p.stat().st_mtime, reverse=True)
    return str(bag_files[0]) if bag_files else None


if __name__ == "__main__":
    parser = argparse.ArgumentParser(
        description='Plot turtlebot simulation data from rosbag',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
  # Plot a specific bag file
  python3 plot_rosbag.py path/to/bagfile.bag
  
  # Plot the most recent bag in rosbags directory
  python3 plot_rosbag.py --latest
  
  # Specify custom rosbags directory
  python3 plot_rosbag.py --latest --dir /path/to/rosbags
        """
    )
    
    parser.add_argument('bagfile', nargs='?', help='Path to the rosbag file')
    parser.add_argument('--latest', action='store_true', 
                       help='Use the most recent bag file in the rosbags directory')
    parser.add_argument('--dir', default='rosbags', 
                       help='Directory to search for rosbag files (default: rosbags)')
    
    args = parser.parse_args()
    
    # Determine which bag file to use
    if args.latest:
        bag_file = find_latest_rosbag(args.dir)
        if bag_file is None:
            print(f"Error: No bag files found in '{args.dir}'")
            sys.exit(1)
        print(f"Using latest bag file: {bag_file}")
    elif args.bagfile:
        bag_file = args.bagfile
    else:
        # Try to find latest in default directory
        bag_file = find_latest_rosbag(args.dir)
        if bag_file is None:
            parser.print_help()
            print(f"\nError: No bag file specified and no bags found in '{args.dir}'")
            sys.exit(1)
        print(f"Using latest bag file: {bag_file}")
    
    # Plot the data
    plot_rosbag_data(bag_file)

