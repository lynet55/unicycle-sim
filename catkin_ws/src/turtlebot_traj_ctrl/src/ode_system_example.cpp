#include <ros/ros.h>
#include <iostream>
#include <fstream>
#include <vector>
#include <boost/numeric/odeint.hpp>
#include "controller/ode_systems.hpp"

using namespace std;
using namespace boost::numeric::odeint;

/**
 * Observer class for recording trajectory data during integration
 */
struct trajectory_observer {
    vector<state_type> &states;
    vector<double> &times;

    trajectory_observer(vector<state_type> &s, vector<double> &t) 
        : states(s), times(t) {}

    void operator()(const state_type &x, double t) {
        states.push_back(x);
        times.push_back(t);
    }
};

/**
 * Save trajectory results to file
 * Format: time x y theta v omega
 */
void save_trajectory(const string &filename, 
                     const vector<state_type> &states,
                     const vector<double> &times) {
    ofstream file(filename);
    if (!file.is_open()) {
        ROS_ERROR_STREAM("Failed to open file: " << filename);
        return;
    }

    // Write header
    file << "# time x y theta v omega\n";
    
    // Write data
    for (size_t i = 0; i < times.size(); ++i) {
        file << times[i];
        for (size_t j = 0; j < states[i].size(); ++j) {
            file << " " << states[i][j];
        }
        file << "\n";
    }
    file.close();
    ROS_INFO_STREAM("Saved trajectory to: " << filename);
}

/**
 * Simulate unicycle robot using RK4 integration
 */
void simulate_unicycle_robot() {
    ROS_INFO("=== Unicycle Robot Model with RK4 Integration ===\n");
    
    // Create robot system with circular motion parameters
    // v_desired = 1.0 m/s, omega_desired = 0.5 rad/s
    // Control gains: k_v = 2.0, k_omega = 2.0
    UnicycleRobot system(1.0, 0.5, 2.0, 2.0);
    
    ROS_INFO("Robot parameters:");
    ROS_INFO("  Desired velocity: %.2f m/s", system.v_desired);
    ROS_INFO("  Desired angular velocity: %.2f rad/s", system.omega_desired);
    ROS_INFO("  Velocity control gain: %.2f", system.k_v);
    ROS_INFO("  Angular velocity control gain: %.2f\n", system.k_omega);
    
    // Initial conditions: [x, y, theta, v, omega]
    state_type x(5);
    x[0] = 0.0;  // x position [m]
    x[1] = 0.0;  // y position [m]
    x[2] = 0.0;  // heading angle [rad]
    x[3] = 0.0;  // linear velocity [m/s]
    x[4] = 0.0;  // angular velocity [rad/s]
    
    ROS_INFO("Initial conditions:");
    ROS_INFO("  Position: (%.2f, %.2f) m", x[0], x[1]);
    ROS_INFO("  Heading: %.2f rad", x[2]);
    ROS_INFO("  Velocity: %.2f m/s", x[3]);
    ROS_INFO("  Angular velocity: %.2f rad/s\n", x[4]);
    
    // Integration parameters
    double t_start = 0.0;
    double t_end = 20.0;
    double dt = 0.01;  // Time step for RK4
    
    ROS_INFO("Integration parameters:");
    ROS_INFO("  Start time: %.2f s", t_start);
    ROS_INFO("  End time: %.2f s", t_end);
    ROS_INFO("  Time step (dt): %.4f s", dt);
    ROS_INFO("  Integration method: Runge-Kutta 4th order (RK4)\n");
    
    // Storage for trajectory
    vector<state_type> states;
    vector<double> times;
    
    // Integrate using Runge-Kutta 4th order method
    ROS_INFO("Starting integration...");
    integrate_const(runge_kutta4<state_type>(),
                    system, x, t_start, t_end, dt,
                    trajectory_observer(states, times));
    
    // Print results
    ROS_INFO("\n=== Simulation Results ===");
    ROS_INFO("Simulated %zu time steps", states.size());
    ROS_INFO("Final state:");
    ROS_INFO("  Position: (%.4f, %.4f) m", x[0], x[1]);
    ROS_INFO("  Heading: %.4f rad (%.2f deg)", x[2], x[2] * 180.0 / M_PI);
    ROS_INFO("  Velocity: %.4f m/s", x[3]);
    ROS_INFO("  Angular velocity: %.4f rad/s\n", x[4]);
    
    // Calculate some statistics
    double total_distance = 0.0;
    for (size_t i = 1; i < states.size(); ++i) {
        double dx = states[i][0] - states[i-1][0];
        double dy = states[i][1] - states[i-1][1];
        total_distance += sqrt(dx*dx + dy*dy);
    }
    ROS_INFO("Total distance traveled: %.4f m", total_distance);
    
    // Save trajectory to file
    save_trajectory("/tmp/unicycle_robot.dat", states, times);
}

int main(int argc, char **argv) {
    // Initialize ROS node
    ros::init(argc, argv, "ode_system_example");
    ros::NodeHandle nh;
    
    ROS_INFO("========================================");
    ROS_INFO("Unicycle Robot ODE System Simulation");
    ROS_INFO("Using Boost Odeint with RK4 Integration");
    ROS_INFO("========================================\n");
    
    // Run simulation
    simulate_unicycle_robot();
    
    ROS_INFO("\n========================================");
    ROS_INFO("Simulation completed!");
    ROS_INFO("Results saved to /tmp/unicycle_robot.dat");
    ROS_INFO("========================================");
    
    return 0;
}

