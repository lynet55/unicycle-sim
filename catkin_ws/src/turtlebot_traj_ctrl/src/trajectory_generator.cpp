#include "controller/trajectory_generator.hpp"
#include <cmath>

// Constructor
Trajectory::Trajectory(double totalTime, double amplitude) : T(totalTime) {
    // Generate initial trajectory
    generateTrajectory(100, amplitude);
}

// Private method to calculate a point at time t
std::pair<double, double> Trajectory::getPoint(double t, double amplitude) const {
    const double PI = 3.14159265358979323846;
    double angle = (2.0 * PI / T) * t;
    
    // x = a * sin(2π/T * t)
    double x = amplitude * std::sin(angle);
    
    // y = a * sin(2π/T * t) * cos(2π/T * t)
    double y = amplitude * std::sin(angle) * std::cos(angle);
    
    return std::make_pair(x, y);
}

// Generate trajectory with specified number of points
void Trajectory::generateTrajectory(int numPoints, double amplitude) {
    points.clear();
    points.reserve(numPoints);
    
    for (int i = 0; i < numPoints; ++i) {
        double t = (T * i) / (numPoints - 1);  // Time from 0 to T
        points.push_back(getPoint(t, amplitude));
    }
}

// Get the number of points
size_t Trajectory::size() const {
    return points.size();
}

// Clear all points
void Trajectory::clear() {
    points.clear();
}

// Access operator
std::pair<double, double> Trajectory::operator[](size_t index) const {
    return points[index];
}