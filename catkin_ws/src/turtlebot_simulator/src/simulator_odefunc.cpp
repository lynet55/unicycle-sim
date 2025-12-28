#include "simulator/simulator_odefun.hpp"

void UnicycleRobot::simulator_odefunc(const state_type &state, state_type &dxdt, double t) {
    double theta = state[2];
    double v = state[3];
    double omega = state[4];

    // First-order dynamics for velocity: v_dot = (v_cmd - v) / T_a
    // First-order dynamics for angular velocity: omega_dot = (omega_cmd - omega) / T_a
    double v_dot = (this->v_cmd - v) / this->Ts;
    double omega_dot = (this->omega_cmd - omega) / this->Ts;

    // Kinematic equations for unicycle model
    dxdt[0] = v * std::cos(theta);  // dx/dt
    dxdt[1] = v * std::sin(theta);  // dy/dt
    dxdt[2] = omega;                 // dtheta/dt
    dxdt[3] = v_dot;                 // dv/dt
    dxdt[4] = omega_dot;             // domega/dt
}

void UnicycleRobot::integrate() {
    if (!modelParams_set) {
        throw std::runtime_error("Model parameters not set");
    }

    using namespace std::placeholders;
    stepper_.do_step(std::bind(&UnicycleRobot::simulator_odefunc, this, _1, _2, _3), state, t, dt);
    t += dt;
}
