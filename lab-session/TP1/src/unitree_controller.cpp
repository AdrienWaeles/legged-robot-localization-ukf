#include "unitree_controller.hpp"
#include <iostream>
#include <cmath>
#include <thread>
#include <chrono>
#include <mutex>

UnitreeController::UnitreeController() 
    : is_connected_(false), movement_completed_(false), should_stop_thread_(false),
      target_distance_(0.0), target_angle_(0.0), current_speed_(0.0), current_angular_speed_(0.0) {
    current_pose_ = {0.0, 0.0, 0.0, 0.0};
    start_pose_ = {0.0, 0.0, 0.0, 0.0};
}

UnitreeController::~UnitreeController() {
    shutdown();
}

bool UnitreeController::initialize(const std::string& robot_ip) {
    try {
        std::cout << "Initializing robot connection to " << robot_ip << "..." << std::endl;
        
        // Simple initialization without specifying IP
        unitree::robot::ChannelFactory::Instance()->Init(0);
        
        // Create sport client
        sport_client_ = std::make_unique<unitree::robot::go2::SportClient>();
        sport_client_->SetTimeout(10.0f);
        sport_client_->Init();
        
        // Create state client
        state_client_ = std::make_unique<unitree::robot::go2::RobotStateClient>();
        state_client_->SetTimeout(10.0f);
        state_client_->Init();
        
        // Start monitoring thread
        should_stop_thread_ = false;
        state_thread_ = std::thread(&UnitreeController::stateUpdateLoop, this);
        
        // Wait for initial connection
        std::this_thread::sleep_for(std::chrono::seconds(2));
        
        is_connected_ = true;
        std::cout << "Robot connected successfully!" << std::endl;
        return true;
        
    } catch (const std::exception& e) {
        std::cerr << "Exception during initialization: " << e.what() << std::endl;
        return false;
    } catch (...) {
        std::cerr << "Unknown exception during initialization" << std::endl;
        return false;
    }
}

void UnitreeController::shutdown() {
    if (is_connected_) {
        std::cout << "Shutting down robot controller..." << std::endl;
        
        // Stop any ongoing movement
        stop();
        
        // Stop monitoring thread
        should_stop_thread_ = true;
        if (state_thread_.joinable()) {
            state_thread_.join();
        }
        
        // Clean up resources
        sport_client_.reset();
        state_client_.reset();
        
        is_connected_ = false;
        std::cout << "Robot controller shutdown complete." << std::endl;
    }
}

void UnitreeController::sendVelocityCommand(const VelocityCommand& cmd) {
    if (!is_connected_ || !sport_client_) {
        std::cerr << "Robot not connected!" << std::endl;
        return;
    }
    
    try {
        std::cout << "DEBUG: Sending Move command - vx: " << cmd.vx << ", vy: " << cmd.vy << ", vyaw: " << cmd.vyaw << std::endl;
        
        // Only use the Move method that exists
        sport_client_->Move(
            static_cast<float>(cmd.vx), 
            static_cast<float>(cmd.vy), 
            static_cast<float>(cmd.vyaw)
        );
    } catch (const std::exception& e) {
        std::cerr << "Error sending velocity command: " << e.what() << std::endl;
    }
}

void UnitreeController::stop() {
    VelocityCommand cmd = {0.0, 0.0, 0.0};
    sendVelocityCommand(cmd);
}

void UnitreeController::standUp() {
    if (!is_connected_ || !sport_client_) {
        std::cerr << "Robot not connected!" << std::endl;
        return;
    }
    
    try {
        sport_client_->StandUp();
        std::cout << "Standing up..." << std::endl;
    } catch (const std::exception& e) {
        std::cerr << "Error during standUp: " << e.what() << std::endl;
    }
}

void UnitreeController::standDown() {
    if (!is_connected_ || !sport_client_) {
        std::cerr << "Robot not connected!" << std::endl;
        return;
    }
    
    try {
        sport_client_->StandDown();
        std::cout << "Standing down..." << std::endl;
    } catch (const std::exception& e) {
        std::cerr << "Error during standDown: " << e.what() << std::endl;
    }
}

RobotPose UnitreeController::getCurrentPose() const {
    std::lock_guard<std::mutex> lock(pose_mutex_);
    return current_pose_;
}

bool UnitreeController::isConnected() const {
    return is_connected_;
}

void UnitreeController::moveForward(double distance, double speed) {
    std::cout << "Starting forward movement: " << distance << "m at " << speed << "m/s" << std::endl;
    
    // Ensure robot is in sport mode
    if (sport_client_) {
        try {
            sport_client_->RecoveryStand();  // Try this method
            std::this_thread::sleep_for(std::chrono::milliseconds(500));
        } catch (...) {
            // Ignore if method doesn't exist
        }
    }
    
    start_pose_ = getCurrentPose();
    target_distance_ = distance;
    target_angle_ = 0.0;
    current_speed_ = speed;
    movement_completed_ = false;
    movement_start_time_ = std::chrono::steady_clock::now();
    
    // Initial command will be sent by stateUpdateLoop
}

void UnitreeController::moveBackward(double distance, double speed) {
    std::cout << "Starting backward movement: " << distance << "m at " << speed << "m/s" << std::endl;
    
    start_pose_ = getCurrentPose();
    target_distance_ = -distance;  // Negative for backward
    target_angle_ = 0.0;
    current_speed_ = speed;
    movement_completed_ = false;
    movement_start_time_ = std::chrono::steady_clock::now();
    
    // Initial command will be sent by stateUpdateLoop
}

void UnitreeController::rotateInPlace(double angle_rad, double angular_speed) {
    std::cout << "Starting rotation: " << angle_rad << " rad at " << angular_speed << " rad/s" << std::endl;
    
    start_pose_ = getCurrentPose();
    target_angle_ = angle_rad;
    target_distance_ = 0.0;
    current_angular_speed_ = angular_speed;  // Store the actual angular speed
    movement_completed_ = false;
    movement_start_time_ = std::chrono::steady_clock::now();
    
    double direction = (angle_rad > 0) ? 1.0 : -1.0;
    VelocityCommand cmd = {0.0, 0.0, direction * angular_speed};
    sendVelocityCommand(cmd);
}

bool UnitreeController::waitForMovementCompletion(double timeout_seconds) {
    auto start_time = std::chrono::steady_clock::now();
    
    std::cout << "Waiting for movement completion (timeout: " << timeout_seconds << "s)..." << std::endl;
    
    while (!movement_completed_) {
        auto current_time = std::chrono::steady_clock::now();
        auto elapsed = std::chrono::duration_cast<std::chrono::seconds>(current_time - start_time).count();
        
        if (elapsed >= timeout_seconds) {
            std::cout << "Movement timeout!" << std::endl;
            stop();
            return false;
        }
        
        // Print progress periodically
        if (elapsed > 0 && elapsed % 2 == 0) {
            auto current_pose = getCurrentPose();
            if (target_distance_ > 0) {
                double traveled = calculateDistance(start_pose_, current_pose);
                std::cout << "Distance progress: " << traveled << "/" << target_distance_ << "m" << std::endl;
            } else if (std::abs(target_angle_) > 0) {
                double rotated = std::abs(normalizeAngle(current_pose.theta - start_pose_.theta));
                std::cout << "Rotation progress: " << rotated << "/" << std::abs(target_angle_) << " rad" << std::endl;
            }
        }
        
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
    
    std::cout << "Movement completed successfully!" << std::endl;
    stop();
    return true;
}

void UnitreeController::stateUpdateLoop() {
    std::cout << "State monitoring started" << std::endl;
    
    while (!should_stop_thread_) {
        try {
            auto current_time = std::chrono::steady_clock::now();
            
            // Update timestamp
            {
                std::lock_guard<std::mutex> lock(pose_mutex_);
                current_pose_.timestamp = std::chrono::duration_cast<std::chrono::milliseconds>(
                    current_time.time_since_epoch()).count() / 1000.0;
            }
            
            // Check movement completion and send continuous commands
            if (!movement_completed_) {
                auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(
                    current_time - movement_start_time_).count() / 1000.0;
                
                double expected_duration = 0.0;
                
                // Handle both forward AND backward movement
                if (std::abs(target_distance_) > 0) {
                    expected_duration = std::abs(target_distance_) / current_speed_;
                    
                    // Continuously send velocity command for linear movement
                    if (elapsed < expected_duration) {
                        double direction = (target_distance_ > 0) ? 1.0 : -1.0;
                        VelocityCommand cmd = {current_speed_ * direction, 0.0, 0.0};
                        sendVelocityCommand(cmd);
                        std::cout << "Sending velocity: " << cmd.vx << " m/s" << std::endl;
                    }
                    
                } else if (std::abs(target_angle_) > 0) {
                    expected_duration = std::abs(target_angle_) / current_angular_speed_;
                    
                    // Continuously send velocity command for rotation
                    if (elapsed < expected_duration) {
                        double direction = (target_angle_ > 0) ? 1.0 : -1.0;
                        VelocityCommand cmd = {0.0, 0.0, direction * current_angular_speed_};
                        sendVelocityCommand(cmd);
                        std::cout << "Sending angular velocity: " << cmd.vyaw << " rad/s" << std::endl;
                    }
                }
                
                std::cout << "Movement progress: " << elapsed << "/" << expected_duration << "s" << std::endl;
                
                if (elapsed >= expected_duration) {
                    std::cout << "Movement duration completed!" << std::endl;
                    movement_completed_ = true;
                    stop();
                }
                
                // Safety timeout
                if (elapsed > 30) {
                    std::cout << "Safety timeout - stopping movement" << std::endl;
                    movement_completed_ = true;
                    stop();
                }
            }
        } catch (const std::exception& e) {
            std::cerr << "Error in state update loop: " << e.what() << std::endl;
        }
        
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }
    
    std::cout << "State monitoring stopped" << std::endl;
}

double UnitreeController::calculateDistance(const RobotPose& pose1, const RobotPose& pose2) const {
    double dx = pose2.x - pose1.x;
    double dy = pose2.y - pose1.y;
    return std::sqrt(dx * dx + dy * dy);
}

double UnitreeController::normalizeAngle(double angle) const {
    while (angle > M_PI) angle -= 2.0 * M_PI;
    while (angle < -M_PI) angle += 2.0 * M_PI;
    return angle;
}

void UnitreeController::debugSportClient() {
    if (!sport_client_) {
        std::cout << "Sport client is null" << std::endl;
        return;
    }
    
    std::cout << "Testing available sport client methods..." << std::endl;
    
    try {
        sport_client_->StandUp();
        std::cout << "✓ StandUp works" << std::endl;
    } catch (...) {
        std::cout << "✗ StandUp failed" << std::endl;
    }
    
    try {
        sport_client_->StandDown();
        std::cout << "✓ StandDown works" << std::endl;
    } catch (...) {
        std::cout << "✗ StandDown failed" << std::endl;
    }
    
    try {
        sport_client_->RecoveryStand();
        std::cout << "✓ RecoveryStand works" << std::endl;
    } catch (...) {
        std::cout << "✗ RecoveryStand failed" << std::endl;
    }
}

void UnitreeController::enterSportMode() {
    if (!sport_client_) {
        std::cerr << "Sport client not available!" << std::endl;
        return;
    }
    
    try {
        std::cout << "Attempting to enter sport mode..." << std::endl;
        sport_client_->RecoveryStand();
        std::this_thread::sleep_for(std::chrono::seconds(1));
        std::cout << "Sport mode activated" << std::endl;
    } catch (const std::exception& e) {
        std::cerr << "Error entering sport mode: " << e.what() << std::endl;
    }
}
