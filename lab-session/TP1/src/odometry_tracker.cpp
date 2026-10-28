#include "odometry_tracker.hpp"
#include <cmath>
#include <iostream>

OdometryTracker::OdometryTracker() {
    // Initialize any necessary variables
}

OdometryTracker::~OdometryTracker() {
    // Cleanup if needed
}

void OdometryTracker::addOdometryReading(const OdometryReading& reading) {
    trajectory_.push_back(reading);
}

void OdometryTracker::resetTracker() {
    trajectory_.clear();
}

ErrorMeasurement OdometryTracker::calculateLinearError(const OdometryReading& start_pose, 
                                                      const OdometryReading& end_pose,
                                                      double expected_displacement) {
    ErrorMeasurement error;
    error.timestamp = end_pose.timestamp;
    
    // Calculate the actual displacement
    double actual_displacement = calculateDistance(start_pose, end_pose);
    
    // Calculate position error
    error.position_error = std::abs(actual_displacement - expected_displacement);
    
    // Calculate angle error (should be small for linear movement)
    error.angle_error = std::abs(normalizeAngle(end_pose.theta - start_pose.theta));
    
    return error;
}

ErrorMeasurement OdometryTracker::calculateRotationalError(const OdometryReading& start_pose,
                                                          const OdometryReading& end_pose,
                                                          double expected_angle_change) {
    ErrorMeasurement error;
    error.timestamp = end_pose.timestamp;
    
    // Calculate the actual angle change
    double actual_angle_change = normalizeAngle(end_pose.theta - start_pose.theta);
    
    // Calculate angle error
    error.angle_error = std::abs(actual_angle_change - expected_angle_change);
    
    // Calculate position error (should be small for rotation in place)
    error.position_error = calculateDistance(start_pose, end_pose);
    
    return error;
}

std::vector<OdometryReading> OdometryTracker::getTrajectory() const {
    return trajectory_;
}

double OdometryTracker::getTotalDistance() const {
    double total = 0.0;
    
    if (trajectory_.size() < 2) {
        return 0.0;
    }
    
    for (size_t i = 1; i < trajectory_.size(); ++i) {
        total += calculateDistance(trajectory_[i-1], trajectory_[i]);
    }
    
    return total;
}

double OdometryTracker::calculateDistance(const OdometryReading& pose1, const OdometryReading& pose2) const {
    double dx = pose2.x - pose1.x;
    double dy = pose2.y - pose1.y;
    return std::sqrt(dx*dx + dy*dy);
}

double OdometryTracker::normalizeAngle(double angle) const {
    while (angle > M_PI) angle -= 2.0 * M_PI;
    while (angle < -M_PI) angle += 2.0 * M_PI;
    return angle;
}