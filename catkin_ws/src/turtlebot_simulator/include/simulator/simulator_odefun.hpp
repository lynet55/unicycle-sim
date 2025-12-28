#ifndef SIMULATOR_ODEFUNC_HPP
#define SIMULATOR_ODEFUNC_HPP

#include <vector>
#include <cmath>
#include <boost/numeric/odeint.hpp>

using namespace boost::numeric::odeint;

// Type definition for state vector
typedef std::vector<double> state_type;

class UnicycleRobot {
public:
    UnicycleRobot(double deltaT) : dt(deltaT), t(0.0), state(5), modelParams_set(false) {
        /* Initialize state vector */
        state[0] = 0.0; // x position [m]
        state[1] = 0.0; // y position [m]
        state[2] = 0.0; // heading angle [rad]
        state[3] = 0.0; // linear velocity [m/s]
        state[4] = 0.0; // angular velocity [rad/s]
    }

    void setInitalState(double x_init, double y_init, double theta_init, double v_init, double w_init) {
        state[0] = x_init;
        state[1] = y_init;
        state[2] = theta_init;
        state[3] = v_init;
        state[4] = w_init;
    }

    void setModelParams(double a, double Ts) {
        this->a = a;
        this->Ts = Ts;
        this->modelParams_set = true;
    }

    void setInputValues(double v_cmd, double omega_cmd) {
        this->v_cmd = v_cmd;
        this->omega_cmd = omega_cmd;
    }

    void getState(double &x, double &y, double &theta, double &v, double &omega) {
        x = state[0];
        y = state[1];
        theta = state[2];
        v = state[3];
        omega = state[4];
    }

    void getTime(double &time) {
        time = t;
    }

    void integrate();

private:
    double t, dt; // simulation parameters
    double a, Ts; // model parameters
    double v_cmd, omega_cmd; // input values

    bool modelParams_set;

        state_type state;
        runge_kutta_dopri5<state_type> stepper_;

        void simulator_odefunc(const state_type &state, state_type &dxdt, double t);
        
};

#endif // SIMULATOR_ODEFUN_HPP
