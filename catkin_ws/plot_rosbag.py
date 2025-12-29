#!/usr/bin/env python3
"""
ROS Bag Plotting Script for Turtlebot Simulation
Plots state tracking, reference trajectory, and control commands
"""

import rosbag
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from matplotlib.patches import Circle, Polygon, FancyArrow
import numpy as np
from pathlib import Path
import argparse
import sys


def draw_unicycle_robot(ax, x, y, theta, scale=0.15, color='black'):
    """
    Draw a simple black circle for the robot
    
    Args:
        ax: matplotlib axes
        x, y: position of robot
        theta: orientation angle (radians)
        scale: size of the robot
        color: color of the robot
    
    Returns:
        list of artists (patches) that were drawn
    """
    artists = []
    
    # Robot body as a simple filled black circle
    circle = Circle((x, y), radius=scale/2, 
                   facecolor=color, 
                   edgecolor='black', 
                   linewidth=1.5, 
                   alpha=1.0,
                   zorder=10)
    ax.add_patch(circle)
    artists.append(circle)
    
    return artists


def animate_unicycle_trajectory(data, output_file, kp_x=3.0, kp_y=3.0, ki_x=0.1, ki_y=0.1):
    """
    Create a minimalistic animation of the unicycle robot following its trajectory
    
    Args:
        data: dictionary containing state and reference data
        output_file: path to save the animation
        kp_x: Proportional gain for x-direction (default: 3.0)
        kp_y: Proportional gain for y-direction (default: 3.0)
        ki_x: Integral gain for x-direction (default: 0.1)
        ki_y: Integral gain for y-direction (default: 0.1)
    """
    print("\nCreating animation...")
    
    # Extract bag ID from filename
    bag_id = Path(output_file).stem  # e.g., "simulation_2025-12-29-00-41-50"
    if bag_id.startswith('simulation_'):
        bag_id = bag_id.replace('simulation_', '')
    
    # Use LaTeX rendering
    plt.rcParams['text.usetex'] = False  # Set to True if LaTeX is installed
    plt.rcParams['font.family'] = 'serif'
    plt.rcParams['mathtext.fontset'] = 'cm'  # Computer Modern font (LaTeX-like)
    
    # Set up minimalistic figure
    fig, ax = plt.subplots(figsize=(8, 8))
    
    # Determine plot limits
    x_vals = data['state']['x']
    y_vals = data['state']['y']
    x_min, x_max = min(x_vals), max(x_vals)
    y_min, y_max = min(y_vals), max(y_vals)
    
    # Add margin
    margin = 0.5
    ax.set_xlim(x_min - margin, x_max + margin)
    ax.set_ylim(y_min - margin, y_max + margin)
    ax.set_aspect('equal')
    
    # Minimalistic styling - remove borders
    ax.spines['top'].set_visible(False)
    ax.spines['right'].set_visible(False)
    ax.spines['left'].set_linewidth(1.5)
    ax.spines['bottom'].set_linewidth(1.5)
    ax.spines['left'].set_color('#2E3440')
    ax.spines['bottom'].set_color('#2E3440')
    
    # Grid with minimal style
    ax.grid(True, alpha=0.15, linewidth=0.5, color='#4C566A', linestyle='-')
    ax.set_axisbelow(True)
    
    # LaTeX-style labels
    ax.set_xlabel(r'$x$ [m]', fontsize=14, color='#2E3440')
    ax.set_ylabel(r'$y$ [m]', fontsize=14, color='#2E3440')
    
    # Minimal tick styling
    ax.tick_params(colors='#2E3440', width=1.5, labelsize=11)
    
    # Plot reference trajectory if available (minimalistic)
    if len(data['reference']['x']) > 0:
        ax.plot(data['reference']['x'], data['reference']['y'], 
               color='#D08770', linewidth=1.5, linestyle='--', 
               alpha=0.6, zorder=1, label=r'Reference')
    
    # Plot full actual trajectory (very faded)
    ax.plot(x_vals, y_vals, color='#5E81AC', linewidth=0.8, 
           alpha=0.2, zorder=2)
    
    # Initialize trail line (bold and clean)
    trail_line, = ax.plot([], [], color='#5E81AC', linewidth=2.5, 
                         alpha=1.0, zorder=3, solid_capstyle='butt')
    
    # Minimal time text (no box)
    time_text = ax.text(0.02, 0.98, '', transform=ax.transAxes,
                       fontsize=11, verticalalignment='top',
                       horizontalalignment='left',
                       color='#2E3440',
                       family='monospace')
    
    # Controller gains and bag ID display (upper left)
    gains_text = (
        f'Bag: {bag_id}\n'
        r'$K_{p,x} = $' + f'{kp_x:.1f}\n'
        r'$K_{p,y} = $' + f'{kp_y:.1f}'
    )
    
    # Position text on the upper left
    gains_info_text = ax.text(0.02, 0.93, gains_text, transform=ax.transAxes,
                              fontsize=9, verticalalignment='top',
                              horizontalalignment='left',
                              color='#2E3440',
                              family='serif',
                              rotation=0)
    
    # Remove legend or make it minimal
    if len(data['reference']['x']) > 0:
        ax.legend(loc='upper right', fontsize=10, frameon=False, 
                 labelcolor='#2E3440')
    
    # Storage for robot artists
    robot_artists = []
    
    # Downsample data for animation - slower (fewer skips)
    skip = max(1, len(x_vals) // 150)  # Fewer frames for slower animation
    indices = list(range(0, len(x_vals), skip))
    
    def init():
        """Initialize animation"""
        trail_line.set_data([], [])
        time_text.set_text('')
        return [trail_line, time_text]
    
    def animate(frame_idx):
        """Update animation for each frame"""
        # Get actual data index
        idx = indices[frame_idx]
        
        # Remove previous robot
        for artist in robot_artists:
            artist.remove()
        robot_artists.clear()
        
        # Draw robot at current position
        x = x_vals[idx]
        y = y_vals[idx]
        theta = data['state']['theta'][idx]
        t = data['state']['time'][idx]
        
        new_artists = draw_unicycle_robot(ax, x, y, theta, scale=0.2)
        robot_artists.extend(new_artists)
        
        # Update trail (show path up to current point)
        trail_line.set_data(x_vals[:idx+1], y_vals[:idx+1])
        
        # Update text - minimal
        time_text.set_text(f't = {t:.2f}s')
        
        return [trail_line, time_text] + robot_artists
    
    # Create animation with slower speed
    anim = animation.FuncAnimation(
        fig, animate, init_func=init,
        frames=len(indices), interval=80,  # 80ms between frames = 12.5 fps (slower)
        blit=False, repeat=True
    )
    
    # Tight layout for minimal borders
    plt.tight_layout()
    
    # Save animation
    if output_file.endswith('.bag'):
        animation_file = output_file.replace('.bag', '_animation.gif')
    elif output_file.endswith('_plots.png'):
        animation_file = output_file.replace('_plots.png', '_animation.gif')
    else:
        animation_file = output_file + '_animation.gif'
    print(f"Saving animation to: {animation_file}")
    print("This may take a moment...")
    
    try:
        anim.save(animation_file, writer='pillow', fps=12, dpi=100)
        print(f"Animation saved successfully!")
    except Exception as e:
        print(f"Warning: Could not save animation: {e}")
        print("Showing animation instead...")
        plt.show()
    
    return anim


def plot_rosbag_data(bag_file, create_animation=True):
    """
    Plot data from rosbag containing turtlebot simulation data
    
    Args:
        bag_file: path to the rosbag file
        create_animation: if True, create animated visualization of robot trajectory
    
    Topics expected:
    - /turtlebot/state: x, y, theta, w (actual state - TurtlebotState message)
    - /turtlebot/reference: x, y (reference trajectory - ReferencePoint message)
    - /control_commands: v_cmd, omega_cmd (ControlCommands message)
    """
    
    # Check if bag file exists
    if not Path(bag_file).exists():
        print(f"Error: Bag file '{bag_file}' not found!")
        return
    
    print(f"Reading rosbag: {bag_file}")
    
    # Data storage
    data = {
        'state': {'time': [], 'x': [], 'y': [], 'theta': [], 'w': []},
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
            data['state']['theta'].append(msg.theta)
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
    
    # Use LaTeX rendering for all plots
    plt.rcParams['text.usetex'] = False  # Set to True if LaTeX is installed
    plt.rcParams['font.family'] = 'serif'
    plt.rcParams['mathtext.fontset'] = 'cm'  # Computer Modern font (LaTeX-like)
    
    # Create plots
    fig = plt.figure(figsize=(15, 12))
    
    # Plot 1: X-Y Trajectory (Tracking Plot)
    ax1 = plt.subplot(3, 2, 1)
    ax1.scatter(data['state']['x'], data['state']['y'], 
                c=data['state']['time'], cmap='viridis', 
                label='Actual Trajectory', s=10, alpha=0.6)
    if len(data['reference']['x']) > 0:
        ax1.plot(data['reference']['x'], data['reference']['y'], 
                 'r-', linewidth=2, label='Reference Trajectory', alpha=0.7)
    ax1.set_xlabel(r'$x$ [m]', fontsize=12)
    ax1.set_ylabel(r'$y$ [m]', fontsize=12)
    ax1.set_title('2D Trajectory: Actual vs Reference', fontsize=14, fontweight='bold')
    ax1.legend(frameon=False)
    ax1.grid(True, alpha=0.3)
    ax1.axis('equal')
    cbar = plt.colorbar(ax1.collections[0], ax=ax1)
    cbar.set_label(r'$t$ [s]', fontsize=10)
    
    # Plot 2: X position vs Time
    ax2 = plt.subplot(3, 2, 2)
    ax2.plot(data['state']['time'], data['state']['x'], 
             'b-', linewidth=1.5, label=r'$x$ (actual)', alpha=0.8)
    if len(data['reference']['x']) > 0:
        ax2.plot(data['reference']['time'], data['reference']['x'], 
                 'r--', linewidth=2, label=r'$x_{\mathrm{ref}}$', alpha=0.7)
    ax2.set_xlabel(r'$t$ [s]', fontsize=12)
    ax2.set_ylabel(r'$x$ [m]', fontsize=12)
    ax2.set_title(r'$x$ Position Tracking', fontsize=14, fontweight='bold')
    ax2.legend(frameon=False)
    ax2.grid(True, alpha=0.3)
    
    # Plot 3: Y position vs Time
    ax3 = plt.subplot(3, 2, 3)
    ax3.plot(data['state']['time'], data['state']['y'], 
             'b-', linewidth=1.5, label=r'$y$ (actual)', alpha=0.8)
    if len(data['reference']['y']) > 0:
        ax3.plot(data['reference']['time'], data['reference']['y'], 
                 'r--', linewidth=2, label=r'$y_{\mathrm{ref}}$', alpha=0.7)
    ax3.set_xlabel(r'$t$ [s]', fontsize=12)
    ax3.set_ylabel(r'$y$ [m]', fontsize=12)
    ax3.set_title(r'$y$ Position Tracking', fontsize=14, fontweight='bold')
    ax3.legend(frameon=False)
    ax3.grid(True, alpha=0.3)
    
    # Plot 4: Angular velocity (w) vs Time
    ax4 = plt.subplot(3, 2, 4)
    ax4.plot(data['state']['time'], data['state']['w'], 
             'c-', linewidth=1.5, label=r'$\omega$ (angular velocity)', alpha=0.8)
    ax4.set_xlabel(r'$t$ [s]', fontsize=12)
    ax4.set_ylabel(r'$\omega$ [rad/s]', fontsize=12)
    ax4.set_title('Angular Velocity', fontsize=14, fontweight='bold')
    ax4.legend(frameon=False)
    ax4.grid(True, alpha=0.3)
    
    # Plot 5: Linear Velocity Command vs Time
    ax5 = plt.subplot(3, 2, 5)
    if len(data['control']['v_cmd']) > 0:
        ax5.plot(data['control']['time'], data['control']['v_cmd'], 
                 'g-', linewidth=1.5, label=r'$v_{\mathrm{cmd}}$ (linear)', alpha=0.8)
        ax5.set_xlabel(r'$t$ [s]', fontsize=12)
        ax5.set_ylabel(r'$v_{\mathrm{cmd}}$ [m/s]', fontsize=12)
        ax5.set_title('Linear Velocity Command', fontsize=14, fontweight='bold')
        ax5.legend(frameon=False)
        ax5.grid(True, alpha=0.3)
    else:
        ax5.text(0.5, 0.5, 'No linear velocity command data', 
                ha='center', va='center', transform=ax5.transAxes)
    
    # Plot 6: Angular Velocity Command vs Time
    ax6 = plt.subplot(3, 2, 6)
    if len(data['control']['omega_cmd']) > 0:
        ax6.plot(data['control']['time'], data['control']['omega_cmd'], 
                 'm-', linewidth=1.5, label=r'$\omega_{\mathrm{cmd}}$ (angular)', alpha=0.8)
        ax6.set_xlabel(r'$t$ [s]', fontsize=12)
        ax6.set_ylabel(r'$\omega_{\mathrm{cmd}}$ [rad/s]', fontsize=12)
        ax6.set_title('Angular Velocity Command', fontsize=14, fontweight='bold')
        ax6.legend(frameon=False)
        ax6.grid(True, alpha=0.3)
    else:
        ax6.text(0.5, 0.5, 'No angular velocity command data', 
                ha='center', va='center', transform=ax6.transAxes)
    
    plt.tight_layout()
    
    # Save figure
    output_file = bag_file.replace('.bag', '_plots.png')
    plt.savefig(output_file, dpi=300, bbox_inches='tight')
    print(f"\nPlots saved to: {output_file}")
    
    # Show plots
    plt.show()
    
    # Create animation if requested
    if create_animation:
        animate_unicycle_trajectory(data, bag_file)
    
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
    else:
        print("No reference data available for tracking error calculation")
    
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
    parser.add_argument('--no-animation', action='store_true',
                       help='Skip creating the animated visualization (faster)')
    
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
    plot_rosbag_data(bag_file, create_animation=not args.no_animation)


