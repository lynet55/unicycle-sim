#ifndef TRAJECTORY_HPP
#define TRAJECTORY_HPP

#include <vector>
#include <utility>

class Trajectory {
public:
    // Default constructor
    Trajectory() : T(1.0) {}
    
    // Constructor: sets the total time T
    explicit Trajectory(double totalTime, double amplitude);
    
    // Generate trajectory points using the parametric equations
    // numPoints: number of points to generate along the trajectory
    // amplitude: the parameter 'a' in the equations
    void generateTrajectory(int numPoints, double amplitude);
    
    // Get the number of points in the trajectory
    size_t size() const;
    
    // Clear all trajectory points
    void clear();
    
    // Access point at index (const version)
    std::pair<double, double> operator[](size_t index) const;

private:
    double T;  // Total time period
    std::vector<std::pair<double, double>> points;  // Container for (x, y) coordinates
    
    // Private method to get a point at a specific time t
    std::pair<double, double> getPoint(double t, double amplitude) const;
};

#endif // TRAJECTORY_HPP