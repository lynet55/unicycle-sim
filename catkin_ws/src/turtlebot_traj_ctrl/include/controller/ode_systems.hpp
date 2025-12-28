#ifndef ODE_SYSTEMS_HPP
#define ODE_SYSTEMS_HPP

#include <vector>
#include <cmath>

// Type definition for state vector
typedef std::vector<double> state_type;

/**
 * Unicycle robot model with first-order dynamics
 * State: [x, y, theta, v, omega]
 * Control inputs: v_desired, omega_desired
 */
class UnicycleRobot {
public:
    double v_desired;      // Desired linear velocity [m/s]
    double omega_desired;  // Desired angular velocity [rad/s]
    double k_v;           // Velocity control gain
    double k_omega;       // Angular velocity control gain

    UnicycleRobot(double v_des, double omega_des, double kv, double komega)
        : v_desired(v_des), omega_desired(omega_des), k_v(kv), k_omega(komega) {}

    /**
     * System dynamics (functor for boost::odeint)
     * state = [x, y, theta, v, omega]
     */
    void operator()(const state_type &x, state_type &dxdt, const double /* t */) {
        // Current state
        double theta = x[2];
        double v = x[3];
        double omega = x[4];

        // Kinematics
        dxdt[0] = v * cos(theta);  // dx/dt
        dxdt[1] = v * sin(theta);  // dy/dt
        dxdt[2] = omega;           // dtheta/dt

        // First-order dynamics (velocity tracking control)
        dxdt[3] = k_v * (v_desired - v);         // dv/dt
        dxdt[4] = k_omega * (omega_desired - omega);  // domega/dt
    }
};

#endif // ODE_SYSTEMS_HPP

