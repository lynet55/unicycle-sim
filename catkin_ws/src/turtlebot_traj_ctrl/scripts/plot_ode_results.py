#!/usr/bin/env python
"""
Script to visualize the results from ODE system simulations
"""

import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import os

def plot_harmonic_oscillator(filename="/tmp/harmonic_oscillator.dat"):
    """Plot harmonic oscillator results"""
    if not os.path.exists(filename):
        print(f"File not found: {filename}")
        return
    
    data = np.loadtxt(filename)
    t = data[:, 0]
    x = data[:, 1]
    v = data[:, 2]
    
    fig, (ax1, ax2, ax3) = plt.subplots(3, 1, figsize=(10, 8))
    
    ax1.plot(t, x, 'b-', linewidth=2)
    ax1.set_ylabel('Position [m]')
    ax1.set_title('Harmonic Oscillator (Mass-Spring-Damper)')
    ax1.grid(True)
    
    ax2.plot(t, v, 'r-', linewidth=2)
    ax2.set_ylabel('Velocity [m/s]')
    ax2.grid(True)
    
    ax3.plot(x, v, 'g-', linewidth=2)
    ax3.set_xlabel('Position [m]')
    ax3.set_ylabel('Velocity [m/s]')
    ax3.set_title('Phase Portrait')
    ax3.grid(True)
    
    plt.tight_layout()
    plt.savefig('/tmp/harmonic_oscillator.png', dpi=150)
    print("Saved: /tmp/harmonic_oscillator.png")

def plot_lorenz_attractor(filename="/tmp/lorenz_attractor.dat"):
    """Plot Lorenz attractor results"""
    if not os.path.exists(filename):
        print(f"File not found: {filename}")
        return
    
    data = np.loadtxt(filename)
    t = data[:, 0]
    x = data[:, 1]
    y = data[:, 2]
    z = data[:, 3]
    
    # 3D plot
    fig = plt.figure(figsize=(12, 10))
    
    ax1 = fig.add_subplot(2, 2, 1, projection='3d')
    ax1.plot(x, y, z, 'b-', linewidth=0.5)
    ax1.set_xlabel('X')
    ax1.set_ylabel('Y')
    ax1.set_zlabel('Z')
    ax1.set_title('Lorenz Attractor - 3D View')
    
    ax2 = fig.add_subplot(2, 2, 2)
    ax2.plot(x, y, 'b-', linewidth=0.5)
    ax2.set_xlabel('X')
    ax2.set_ylabel('Y')
    ax2.set_title('X-Y Projection')
    ax2.grid(True)
    
    ax3 = fig.add_subplot(2, 2, 3)
    ax3.plot(t, x, 'r-', linewidth=1)
    ax3.set_xlabel('Time [s]')
    ax3.set_ylabel('X')
    ax3.set_title('X vs Time')
    ax3.grid(True)
    
    ax4 = fig.add_subplot(2, 2, 4)
    ax4.plot(x, z, 'g-', linewidth=0.5)
    ax4.set_xlabel('X')
    ax4.set_ylabel('Z')
    ax4.set_title('X-Z Projection')
    ax4.grid(True)
    
    plt.tight_layout()
    plt.savefig('/tmp/lorenz_attractor.png', dpi=150)
    print("Saved: /tmp/lorenz_attractor.png")

def plot_unicycle_robot(filename="/tmp/unicycle_robot.dat"):
    """Plot unicycle robot trajectory"""
    if not os.path.exists(filename):
        print(f"File not found: {filename}")
        return
    
    data = np.loadtxt(filename)
    t = data[:, 0]
    x = data[:, 1]
    y = data[:, 2]
    theta = data[:, 3]
    v = data[:, 4]
    omega = data[:, 5]
    
    fig, axes = plt.subplots(2, 2, figsize=(12, 10))
    
    # Trajectory in 2D
    axes[0, 0].plot(x, y, 'b-', linewidth=2)
    axes[0, 0].plot(x[0], y[0], 'go', markersize=10, label='Start')
    axes[0, 0].plot(x[-1], y[-1], 'ro', markersize=10, label='End')
    axes[0, 0].set_xlabel('X [m]')
    axes[0, 0].set_ylabel('Y [m]')
    axes[0, 0].set_title('Robot Trajectory')
    axes[0, 0].legend()
    axes[0, 0].grid(True)
    axes[0, 0].axis('equal')
    
    # Heading angle
    axes[0, 1].plot(t, np.degrees(theta), 'r-', linewidth=2)
    axes[0, 1].set_xlabel('Time [s]')
    axes[0, 1].set_ylabel('Heading [deg]')
    axes[0, 1].set_title('Heading Angle')
    axes[0, 1].grid(True)
    
    # Velocity
    axes[1, 0].plot(t, v, 'g-', linewidth=2)
    axes[1, 0].set_xlabel('Time [s]')
    axes[1, 0].set_ylabel('Velocity [m/s]')
    axes[1, 0].set_title('Linear Velocity')
    axes[1, 0].grid(True)
    
    # Angular velocity
    axes[1, 1].plot(t, omega, 'm-', linewidth=2)
    axes[1, 1].set_xlabel('Time [s]')
    axes[1, 1].set_ylabel('Angular Velocity [rad/s]')
    axes[1, 1].set_title('Angular Velocity')
    axes[1, 1].grid(True)
    
    plt.tight_layout()
    plt.savefig('/tmp/unicycle_robot.png', dpi=150)
    print("Saved: /tmp/unicycle_robot.png")

def plot_van_der_pol(filename="/tmp/van_der_pol.dat"):
    """Plot Van der Pol oscillator results"""
    if not os.path.exists(filename):
        print(f"File not found: {filename}")
        return
    
    data = np.loadtxt(filename)
    t = data[:, 0]
    x = data[:, 1]
    v = data[:, 2]
    
    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(12, 5))
    
    ax1.plot(t, x, 'b-', linewidth=2)
    ax1.set_xlabel('Time [s]')
    ax1.set_ylabel('Position')
    ax1.set_title('Van der Pol Oscillator - Time Series')
    ax1.grid(True)
    
    ax2.plot(x, v, 'r-', linewidth=2)
    ax2.set_xlabel('Position')
    ax2.set_ylabel('Velocity')
    ax2.set_title('Phase Portrait (Limit Cycle)')
    ax2.grid(True)
    
    plt.tight_layout()
    plt.savefig('/tmp/van_der_pol.png', dpi=150)
    print("Saved: /tmp/van_der_pol.png")

def plot_pendulum(filename="/tmp/pendulum.dat"):
    """Plot pendulum results"""
    if not os.path.exists(filename):
        print(f"File not found: {filename}")
        return
    
    data = np.loadtxt(filename)
    t = data[:, 0]
    theta = data[:, 1]
    omega = data[:, 2]
    
    fig, (ax1, ax2, ax3) = plt.subplots(3, 1, figsize=(10, 8))
    
    ax1.plot(t, np.degrees(theta), 'b-', linewidth=2)
    ax1.set_ylabel('Angle [deg]')
    ax1.set_title('Pendulum with Friction')
    ax1.grid(True)
    
    ax2.plot(t, omega, 'r-', linewidth=2)
    ax2.set_ylabel('Angular Velocity [rad/s]')
    ax2.grid(True)
    
    ax3.plot(theta, omega, 'g-', linewidth=2)
    ax3.set_xlabel('Angle [rad]')
    ax3.set_ylabel('Angular Velocity [rad/s]')
    ax3.set_title('Phase Portrait')
    ax3.grid(True)
    
    plt.tight_layout()
    plt.savefig('/tmp/pendulum.png', dpi=150)
    print("Saved: /tmp/pendulum.png")

if __name__ == '__main__':
    print("Plotting ODE system results...")
    print("=" * 50)
    
    plot_harmonic_oscillator()
    plot_lorenz_attractor()
    plot_unicycle_robot()
    plot_van_der_pol()
    plot_pendulum()
    
    print("=" * 50)
    print("All plots saved to /tmp/")
    print("\nTo view plots, open the PNG files or run:")
    print("  display /tmp/*.png")
    
    # Optionally show plots interactively
    try:
        plt.show()
    except:
        pass

