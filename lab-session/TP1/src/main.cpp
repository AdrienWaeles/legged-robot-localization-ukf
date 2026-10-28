#include "unitree_controller.hpp"
#include "ukf_filter.hpp"
#include "odometry_tracker.hpp"
#include <iostream>
#include <iomanip>
#include <vector>
#include <thread>
#include <chrono>
#include <cmath>
#include <functional>
#include <Eigen/Dense>

// Add this function HERE (before TestResults struct)
bool performRobustRotation(UnitreeController& controller, double target_angle_deg, bool use_ukf = false, UKFFilter* filter = nullptr) {
    std::cout << "Starting robust " << target_angle_deg << "° rotation..." << std::endl;
    
    // Multiple speed attempts - start slow, increase if needed
    std::vector<double> speed_levels = {0.15, 0.25, 0.35, 0.5};
    std::vector<double> duration_multipliers = {1.0, 0.8, 0.6, 0.4};
    
    double target_rad = target_angle_deg * M_PI / 180.0;
    
    for (size_t attempt = 0; attempt < speed_levels.size(); attempt++) {
        double angular_speed = speed_levels[attempt];
        double base_duration = std::abs(target_rad) / angular_speed;
        double actual_duration = base_duration * duration_multipliers[attempt];
        
        std::cout << "  Attempt " << (attempt + 1) << "/" << speed_levels.size() 
                  << " - Speed: " << angular_speed << " rad/s, Duration: " << actual_duration << "s" << std::endl;
        
        // Execute rotation in phases
        int num_phases = 4;
        double phase_duration = actual_duration / num_phases;
        double direction = (target_angle_deg > 0) ? 1.0 : -1.0;
        
        for (int phase = 0; phase < num_phases; phase++) {
            std::cout << "    Phase " << (phase + 1) << "/" << num_phases << std::endl;
            
            auto phase_start = std::chrono::steady_clock::now();
            auto phase_end = phase_start + std::chrono::milliseconds(static_cast<int>(phase_duration * 1000));
            
            // Send commands during this phase
            while (std::chrono::steady_clock::now() < phase_end) {
                VelocityCommand cmd = {0.0, 0.0, direction * angular_speed};
                controller.sendVelocityCommand(cmd);
                std::this_thread::sleep_for(std::chrono::milliseconds(25)); // Higher frequency
            }
            
            // Brief pause between phases
            controller.stop();
            std::this_thread::sleep_for(std::chrono::milliseconds(200));
        }
        
        // Final stop
        controller.stop();
        std::cout << "  Rotation attempt " << (attempt + 1) << " completed" << std::endl;
        
        // Wait to see if robot responds
        std::this_thread::sleep_for(std::chrono::seconds(2));
        
        // Test if we should try again with different parameters
        std::cout << "  Checking robot response..." << std::endl;
        std::this_thread::sleep_for(std::chrono::seconds(1));
        
        // If this is not the last attempt, ask if we should continue
        if (attempt < speed_levels.size() - 1) {
            std::cout << "  Should we try with different parameters? (Robot might need more/less force)" << std::endl;
            std::this_thread::sleep_for(std::chrono::seconds(1));
        }
    }
    
    // UKF update
    if (use_ukf && filter) {
        filter->predict(5.0); // Approximate total time
        RobotPose current = controller.getCurrentPose();
        Eigen::VectorXd measurement(3);
        measurement << current.x, current.y, current.theta;
        filter->update(measurement);
        std::cout << "UKF updated after rotation" << std::endl;
    }
    
    std::cout << "Robust rotation sequence completed!" << std::endl;
    return true;
}

bool performSDKRotation(UnitreeController& controller, double target_angle_deg, bool use_ukf = false, UKFFilter* filter = nullptr) {
    std::cout << "Starting SDK-based " << target_angle_deg << "° rotation..." << std::endl;
    
    // Convert to radians
    double target_rad = target_angle_deg * M_PI / 180.0;
    double direction = (target_angle_deg > 0) ? 1.0 : -1.0;
    
    // SDK-based rotation using multiple approaches
    std::vector<std::pair<std::string, std::function<void()>>> rotation_strategies = {
        
        {"Strategy 1: Pure Rotation", [&]() {
            std::cout << "  Using pure vyaw rotation..." << std::endl;
            double vyaw_speed = 0.3; // Within [-4, 4] rad/s range
            double duration = std::abs(target_rad) / vyaw_speed;
            
            auto start_time = std::chrono::steady_clock::now();
            auto end_time = start_time + std::chrono::milliseconds(static_cast<int>(duration * 1000));
            
            while (std::chrono::steady_clock::now() < end_time) {
                VelocityCommand cmd = {0.0, 0.0, direction * vyaw_speed};
                controller.sendVelocityCommand(cmd);
                std::this_thread::sleep_for(std::chrono::milliseconds(30));
            }
            controller.stop();
        }},
        
        {"Strategy 2: Slow Precise Rotation", [&]() {
            std::cout << "  Using slow precise rotation..." << std::endl;
            double vyaw_speed = 0.15; // Slower for precision
            double duration = std::abs(target_rad) / vyaw_speed;
            
            // Break into segments for better control
            int segments = 6;
            double segment_duration = duration / segments;
            
            for (int i = 0; i < segments; i++) {
                std::cout << "    Segment " << (i+1) << "/" << segments << std::endl;
                
                auto segment_start = std::chrono::steady_clock::now();
                auto segment_end = segment_start + std::chrono::milliseconds(static_cast<int>(segment_duration * 1000));
                
                while (std::chrono::steady_clock::now() < segment_end) {
                    VelocityCommand cmd = {0.0, 0.0, direction * vyaw_speed};
                    controller.sendVelocityCommand(cmd);
                    std::this_thread::sleep_for(std::chrono::milliseconds(25));
                }
                
                // Brief pause between segments
                controller.stop();
                std::this_thread::sleep_for(std::chrono::milliseconds(300));
            }
        }},
        
        {"Strategy 3: Burst Rotation", [&]() {
            std::cout << "  Using burst rotation approach..." << std::endl;
            double vyaw_speed = 0.5; // Faster bursts
            
            // Calculate number of short bursts needed
            int num_bursts = 8;
            double burst_duration = (std::abs(target_rad) / vyaw_speed) / num_bursts;
            
            for (int burst = 0; burst < num_bursts; burst++) {
                std::cout << "    Burst " << (burst+1) << "/" << num_bursts << std::endl;
                
                // Short burst of rotation
                auto burst_start = std::chrono::steady_clock::now();
                auto burst_end = burst_start + std::chrono::milliseconds(static_cast<int>(burst_duration * 1000));
                
                while (std::chrono::steady_clock::now() < burst_end) {
                    VelocityCommand cmd = {0.0, 0.0, direction * vyaw_speed};
                    controller.sendVelocityCommand(cmd);
                    std::this_thread::sleep_for(std::chrono::milliseconds(20));
                }
                
                // Stop and wait between bursts
                controller.stop();
                std::this_thread::sleep_for(std::chrono::milliseconds(500));
            }
        }}
    };
    
    // Try each strategy
    for (const auto& strategy : rotation_strategies) {
        std::cout << "" << strategy.first << std::endl;
        
        // Execute the strategy
        strategy.second();
        
        std::cout << "  Strategy completed, checking robot response..." << std::endl;
        std::this_thread::sleep_for(std::chrono::seconds(2));
        
        // Let the robot settle
        std::cout << "  Allowing robot to settle..." << std::endl;
        std::this_thread::sleep_for(std::chrono::seconds(2));
    }
    
    // Final safety stop
    controller.stop();
    
    // UKF update
    if (use_ukf && filter) {
        filter->predict(8.0); // Total approximate time
        RobotPose current = controller.getCurrentPose();
        Eigen::VectorXd measurement(3);
        measurement << current.x, current.y, current.theta;
        filter->update(measurement);
        std::cout << "UKF updated after SDK rotation" << std::endl;
    }
    
    std::cout << "SDK-based rotation sequence completed!" << std::endl;
    return true;
}

struct TestResults {
    double position_error;
    double angle_error;
    double total_error;
    std::string test_name;
};

void printTestResults(const TestResults& results) {
    std::cout << "=== " << results.test_name << " Results ===" << std::endl;
    std::cout << "Position error: " << std::fixed << std::setprecision(3) 
              << results.position_error << " m (" << results.position_error * 100 << " cm)" << std::endl;
    std::cout << "Angle error: " << std::fixed << std::setprecision(1) 
              << results.angle_error << "°" << std::endl;
    std::cout << "Total error: " << std::fixed << std::setprecision(3) 
              << results.total_error << " m" << std::endl;
}

TestResults performLinearTest(UnitreeController& controller, bool use_ukf = false, 
                             UKFFilter* filter = nullptr, OdometryTracker* tracker = nullptr) {
    std::cout << "\n=== Linear Test " << (use_ukf ? "(WITH UKF)" : "(WITHOUT UKF)") << " ===" << std::endl;
    
    // Get initial pose
    RobotPose initial_pose = controller.getCurrentPose();
    
    if (use_ukf && filter) {
        std::cout << "Initializing UKF filter..." << std::endl;
        
        // Initialize UKF with proper state vector and covariance
        Eigen::VectorXd initial_state(3);
        initial_state << initial_pose.x, initial_pose.y, initial_pose.theta;
        
        Eigen::MatrixXd initial_covariance = Eigen::MatrixXd::Identity(3, 3) * 0.1;
        
        filter->initialize(initial_state, initial_covariance);
        std::cout << "UKF filter initialized successfully!" << std::endl;
    }
    
    std::cout << "Initial pose: (" << std::fixed << std::setprecision(3) 
              << initial_pose.x << ", " << initial_pose.y << ", " 
              << initial_pose.theta << ")" << std::endl;
    
    // Test 1: Forward movement (2m)
    std::cout << "\n--- Moving Forward 2m ---" << std::endl;
    controller.moveForward(2.0, 0.3);
    controller.waitForMovementCompletion(15.0);
    
    RobotPose pose_after_forward = controller.getCurrentPose();
    if (use_ukf && filter) {
        std::cout << "Applying UKF prediction and update..." << std::endl;
        
        // Apply UKF filtering
        filter->predict(2.0 / 0.3);  // time = distance / speed
        
        // Create measurement vector from robot pose
        Eigen::VectorXd measurement(3);
        measurement << pose_after_forward.x, pose_after_forward.y, pose_after_forward.theta;
        filter->update(measurement);
        
        // Get filtered state
        Eigen::VectorXd filtered_state = filter->getState();
        pose_after_forward.x = filtered_state(0);
        pose_after_forward.y = filtered_state(1);
        pose_after_forward.theta = filtered_state(2);
        
        std::cout << "UKF filtering applied!" << std::endl;
    }
    
    std::cout << "Pose after forward: (" << std::fixed << std::setprecision(3) 
              << pose_after_forward.x << ", " << pose_after_forward.y << ", " 
              << pose_after_forward.theta << ")" << std::endl;
    
    std::this_thread::sleep_for(std::chrono::seconds(2));
    
    // Test 2: Backward movement (2m)
    std::cout << "\n--- Moving Backward 2m ---" << std::endl;
    controller.moveBackward(2.0, 0.3);
    controller.waitForMovementCompletion(15.0);
    
    RobotPose final_pose = controller.getCurrentPose();
    if (use_ukf && filter) {
        std::cout << "Applying UKF prediction and update..." << std::endl;
        
        // Apply UKF filtering
        filter->predict(2.0 / 0.3);
        
        // Create measurement vector from robot pose
        Eigen::VectorXd measurement(3);
        measurement << final_pose.x, final_pose.y, final_pose.theta;
        filter->update(measurement);
        
        // Get filtered state
        Eigen::VectorXd filtered_state = filter->getState();
        final_pose.x = filtered_state(0);
        final_pose.y = filtered_state(1);
        final_pose.theta = filtered_state(2);
        
        std::cout << "UKF filtering applied!" << std::endl;
    }
    
    std::cout << "Final pose: (" << std::fixed << std::setprecision(3) 
              << final_pose.x << ", " << final_pose.y << ", " 
              << final_pose.theta << ")" << std::endl;
    
    // Calculate errors from the tracked pose
    double position_error = std::sqrt(std::pow(final_pose.x - initial_pose.x, 2) + 
                                    std::pow(final_pose.y - initial_pose.y, 2));
    double angle_error = std::abs(final_pose.theta - initial_pose.theta) * 180.0 / M_PI;
    
    // NOTE: the controller does not read the robot state yet, so the pose stays at zero
    // and these errors are not meaningful until state feedback is implemented.
    
    TestResults results;
    results.test_name = use_ukf ? "Linear Test WITH UKF" : "Linear Test WITHOUT UKF";
    results.position_error = position_error;
    results.angle_error = angle_error;
    results.total_error = position_error;
    
    return results;
}

TestResults performLinearWithRotationTest(UnitreeController& controller, bool use_ukf = false,
                             UKFFilter* filter = nullptr, OdometryTracker* tracker = nullptr) {
    std::cout << "\n=== Linear with Rotation Test " << (use_ukf ? "(WITH UKF)" : "(WITHOUT UKF)") << " ===" << std::endl;
    
    // Get initial pose
    RobotPose initial_pose = controller.getCurrentPose();
    
    if (use_ukf && filter) {
        std::cout << "Initializing UKF filter..." << std::endl;
        
        // Initialize UKF with proper state vector and covariance
        Eigen::VectorXd initial_state(3);
        initial_state << initial_pose.x, initial_pose.y, initial_pose.theta;
        
        Eigen::MatrixXd initial_covariance = Eigen::MatrixXd::Identity(3, 3) * 0.1;
        
        filter->initialize(initial_state, initial_covariance);
        std::cout << "UKF filter initialized successfully!" << std::endl;
    }
    
    std::cout << "Initial pose: (" << std::fixed << std::setprecision(3) 
              << initial_pose.x << ", " << initial_pose.y << ", " 
              << initial_pose.theta << ")" << std::endl;
    
    // Test sequence: Forward 1m -> 180° -> Forward 1m -> 180°
    std::vector<std::pair<std::string, std::function<void()>>> movements = {
        {"Moving Forward 1m (Step 1)", [&]() {
            controller.moveForward(1.0, 0.3);
            controller.waitForMovementCompletion(10.0);
            if (use_ukf && filter) {
                filter->predict(1.0 / 0.3);
                RobotPose current = controller.getCurrentPose();
                Eigen::VectorXd measurement(3);
                measurement << current.x, current.y, current.theta;
                filter->update(measurement);
                std::cout << "UKF updated after forward movement" << std::endl;
            }
        }},
        {"Rotating 180° (Step 1)", [&]() {
            performSDKRotation(controller, 180.0, use_ukf, filter);
            std::this_thread::sleep_for(std::chrono::seconds(2));
        }},
        
        {"Moving Forward 1m (Step 2)", [&]() {
            controller.moveForward(1.0, 0.3);
            controller.waitForMovementCompletion(10.0);
            if (use_ukf && filter) {
                filter->predict(1.0 / 0.3);
                RobotPose current = controller.getCurrentPose();
                Eigen::VectorXd measurement(3);
                measurement << current.x, current.y, current.theta;
                filter->update(measurement);
                std::cout << "UKF updated after forward movement" << std::endl;
            }
        }},
        
        {"Rotating 180° (Step 2)", [&]() {
            performSDKRotation(controller, 180.0, use_ukf, filter);
            std::this_thread::sleep_for(std::chrono::seconds(2));
        }}
    };
    
    // Execute movements
    for (const auto& movement : movements) {
        std::cout << "\n--- " << movement.first << " ---" << std::endl;
        movement.second();
        
        RobotPose current_pose = controller.getCurrentPose();
        if (use_ukf && filter) {
            Eigen::VectorXd filtered_state = filter->getState();
            current_pose.x = filtered_state(0);
            current_pose.y = filtered_state(1);
            current_pose.theta = filtered_state(2);
        }
        std::cout << "Current pose: (" << std::fixed << std::setprecision(3) 
                  << current_pose.x << ", " << current_pose.y << ", " 
                  << current_pose.theta << ")" << std::endl;
        
        std::this_thread::sleep_for(std::chrono::seconds(2));
    }
    
    // Get final pose
    RobotPose final_pose = controller.getCurrentPose();
    if (use_ukf && filter) {
        Eigen::VectorXd filtered_state = filter->getState();
        final_pose.x = filtered_state(0);
        final_pose.y = filtered_state(1);
        final_pose.theta = filtered_state(2);
    }
    
    std::cout << "\nFinal pose: (" << std::fixed << std::setprecision(3) 
              << final_pose.x << ", " << final_pose.y << ", " 
              << final_pose.theta << ")" << std::endl;
    
    // Calculate errors from the tracked pose
    double position_error = std::sqrt(std::pow(final_pose.x - initial_pose.x, 2) + 
                                    std::pow(final_pose.y - initial_pose.y, 2));
    double angle_error = std::abs(final_pose.theta - initial_pose.theta) * 180.0 / M_PI;
    
    // NOTE: the controller does not read the robot state yet, so the pose stays at zero
    // and these errors are not meaningful until state feedback is implemented.
    
    TestResults results;
    results.test_name = use_ukf ? "Linear with Rotation Test WITH UKF" : "Linear with Rotation Test WITHOUT UKF";
    results.position_error = position_error;
    results.angle_error = angle_error;
    results.total_error = position_error;
    
    return results;
}

int main(int argc, char** argv) {
    std::string robot_ip = "192.168.123.161";
    
    if (argc > 1) {
        robot_ip = argv[1];
    }
    
    std::cout << "Unitree GO2 Complete Localization Test Suite" << std::endl;
    std::cout << "Robot IP: " << robot_ip << std::endl;
    
    UnitreeController controller;
    UKFFilter ukf_filter;
    OdometryTracker odometry_tracker;
    
    // Initialize
    if (!controller.initialize(robot_ip)) {
        std::cerr << "Failed to initialize robot controller" << std::endl;
        return -1;
    }

    // Robot activation
    std::cout << "\n=== Robot Activation ===" << std::endl;
    for (int attempt = 1; attempt <= 3; attempt++) {
        std::cout << "Activation attempt " << attempt << "/3" << std::endl;
        controller.enterSportMode();
        std::this_thread::sleep_for(std::chrono::seconds(2));
        controller.standUp();
        std::this_thread::sleep_for(std::chrono::seconds(4));
        
        std::cout << "Did the robot stand up? (y/n): ";
        char response;
        std::cin >> response;
        if (response == 'y' || response == 'Y') {
            std::cout << "Success! Starting test suite..." << std::endl;
            break;
        }
        
        if (attempt == 3) {
            std::cout << "Robot not responding. Exiting." << std::endl;
            return -1;
        }
    }
    
    std::vector<TestResults> all_results;
    
    try {
        std::cout << "\n" << std::string(60, '=') << std::endl;
        std::cout << "STARTING COMPREHENSIVE TEST SUITE" << std::endl;
        std::cout << std::string(60, '=') << std::endl;
        
        // Test 1: Linear Test WITHOUT UKF
        std::cout << "\n>>> TEST 1: Linear Movement (Without UKF) <<<" << std::endl;
        TestResults linear_without_ukf = performLinearTest(controller, false);
        all_results.push_back(linear_without_ukf);
        printTestResults(linear_without_ukf);
        
        std::cout << "\nPress Enter to continue to Test 2..." << std::endl;
        std::cin.get();
        std::cin.get();
        
        // Test 2: Linear Test WITH UKF
        std::cout << "\n>>> TEST 2: Linear Movement (With UKF) <<<" << std::endl;
        TestResults linear_with_ukf = performLinearTest(controller, true, &ukf_filter, &odometry_tracker);
        all_results.push_back(linear_with_ukf);
        printTestResults(linear_with_ukf);
        
        std::cout << "\nPress Enter to continue to Test 3..." << std::endl;
        std::cin.get();
        
        // Test 3: Linear with Rotation Test WITHOUT UKF
        std::cout << "\n>>> TEST 3: Linear with Rotation Movement (Without UKF) <<<" << std::endl;
        TestResults linear_rotation_without_ukf = performLinearWithRotationTest(controller, false);
        all_results.push_back(linear_rotation_without_ukf);
        printTestResults(linear_rotation_without_ukf);
        
        std::cout << "\nPress Enter to continue to Test 4..." << std::endl;
        std::cin.get();
        
        // Test 4: Linear with Rotation Test WITH UKF
        std::cout << "\n>>> TEST 4: Linear with Rotation Movement (With UKF) <<<" << std::endl;
        TestResults linear_rotation_with_ukf = performLinearWithRotationTest(controller, true, &ukf_filter, &odometry_tracker);
        all_results.push_back(linear_rotation_with_ukf);
        printTestResults(linear_rotation_with_ukf);
        
        // Final Analysis
        std::cout << "\n" << std::string(60, '=') << std::endl;
        std::cout << "FINAL COMPARATIVE ANALYSIS" << std::endl;
        std::cout << std::string(60, '=') << std::endl;
        
        // Linear test comparison
        double linear_improvement = ((linear_without_ukf.total_error - linear_with_ukf.total_error) 
                                   / linear_without_ukf.total_error) * 100.0;
        std::cout << "\n=== Linear Test Improvement ===" << std::endl;
        std::cout << "Without UKF: " << std::fixed << std::setprecision(3) 
                  << linear_without_ukf.total_error << " m error" << std::endl;
        std::cout << "With UKF:    " << std::fixed << std::setprecision(3) 
                  << linear_with_ukf.total_error << " m error" << std::endl;
        std::cout << "Improvement: " << std::fixed << std::setprecision(1) 
                  << linear_improvement << "%" << std::endl;
        
        // Linear with rotation test comparison
        double rotation_improvement = ((linear_rotation_without_ukf.total_error - linear_rotation_with_ukf.total_error) 
                                   / linear_rotation_without_ukf.total_error) * 100.0;
        std::cout << "\n=== Linear with Rotation Test Improvement ===" << std::endl;
        std::cout << "Without UKF: " << std::fixed << std::setprecision(3) 
                  << linear_rotation_without_ukf.total_error << " m error" << std::endl;
        std::cout << "With UKF:    " << std::fixed << std::setprecision(3) 
                  << linear_rotation_with_ukf.total_error << " m error" << std::endl;
        std::cout << "Improvement: " << std::fixed << std::setprecision(1) 
                  << rotation_improvement << "%" << std::endl;
        
        // Overall assessment
        std::cout << "\n=== Overall UKF Performance ===" << std::endl;
        if (linear_improvement > 0 && rotation_improvement > 0) {
            std::cout << "UKF SUCCESSFUL: Improved accuracy in both tests!" << std::endl;
        } else if (linear_improvement > 0 || rotation_improvement > 0) {
            std::cout << "UKF PARTIALLY SUCCESSFUL: Improved one test" << std::endl;
        } else {
            std::cout << " UKF needs tuning: No improvement detected" << std::endl;
        }
        
        // Sit down
        std::cout << "\n=== Test Complete - Sitting Down ===" << std::endl;
        controller.standDown();
        
    } catch (const std::exception& e) {
        std::cerr << "Error during testing: " << e.what() << std::endl;
        // Emergency shutdown
        controller.standDown();
    }
    
    controller.shutdown();
    std::cout << "\n=== All Tests Completed Successfully! ===" << std::endl;
    return 0;
}
