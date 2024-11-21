// Copyright (C) 2007 Francois Cauwe <francois at cauwe dot org>
// This library is free software; you can redistribute it and/or
// modify it under the terms of the GNU Lesser General Public
// License as published by the Free Software Foundation; either
// version 2.1 of the License, or (at your option) any later version.

#include <stdio.h>
#include <iostream>
#include <chrono>
#include <cstdlib>
#include <memory>
#include <sstream>

#include "std_msgs/msg/float64_multi_array.hpp"
#include "sensor_msgs/msg/joint_state.hpp"

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/wait_for_message.hpp"

#include "kdl_robot.h"
#include "kdl_control.h"
#include "kdl_planner.h"
#include "kdl_parser/kdl_parser.hpp"

using namespace KDL;
using FloatArray = std_msgs::msg::Float64MultiArray;
using namespace std::chrono_literals;

class Iiwa_pub_sub : public rclcpp::Node {
public:
    Iiwa_pub_sub() : Node("ros2_kdl_node") {
        // Declare parameters
        declare_parameter("cmd_interface", "position"); // defaults to "position"
        get_parameter("cmd_interface", cmd_interface_);
        declare_parameter("current_planner_index", -1); // -1 means not set, default

        iteration_ = 0;
        t_ = 0;
        joint_state_available_ = false;
        trajectory_completed_ = true;
        errorPublisher_ = create_publisher<std_msgs::msg::Float64MultiArray>("trajectory_error", 10);
        torqueErrorPublisher_ = create_publisher<std_msgs::msg::Float64MultiArray>("torque_error", 10);

        // Retrieve robot_description param
        auto parameters_client = std::make_shared<rclcpp::SyncParametersClient>(this, "robot_state_publisher");
        while (!parameters_client->wait_for_service(1s)) {
            if (!rclcpp::ok()) {
                RCLCPP_ERROR(get_logger(), "Interrupted while waiting for the service. Exiting.");
                rclcpp::shutdown();
            }
            RCLCPP_INFO(get_logger(), "Service not available, waiting again...");
        }
        auto parameter = parameters_client->get_parameters({"robot_description"});

        // Create KDLRobot structure
        KDL::Tree robot_tree;
        if (!kdl_parser::treeFromString(parameter[0].value_to_string(), robot_tree)) {
            std::cout << "Failed to retrieve robot_description param!";
        }
        robot_ = std::make_shared<KDLRobot>(robot_tree);

        // Create joint arrays
        unsigned int nj = robot_->getNrJnts();
        KDL::JntArray q_min(nj), q_max(nj);
        q_min.data << -2.96, -2.09, -2.96, -2.09, -2.96, -2.09, -2.96; // TODO: read from urdf file
        q_max.data << 2.96, 2.09, 2.96, 2.09, 2.96, 2.09, 2.96;        // TODO: read from urdf file
        robot_->setJntLimits(q_min, q_max);
        joint_positions_ = KDL::JntArray(nj);
        joint_velocities_ = KDL::JntArray(nj);
        joint_efforts_ = KDL::JntArray(nj);

        Kp_ = Eigen::Vector3d::Constant(50.0);
        Kd_ = Eigen::Vector3d::Constant(10.0);

        // Subscriber to joint states
        jointSubscriber_ = create_subscription<sensor_msgs::msg::JointState>(
            "/joint_states", 10, std::bind(&Iiwa_pub_sub::joint_state_subscriber, this, std::placeholders::_1));

        // Wait for the joint_state topic
        while (!joint_state_available_) {
            RCLCPP_INFO(get_logger(), "No data received yet! ...");
            rclcpp::spin_some(get_node_base_interface());
        }

        // Update KDLRobot object
        robot_->update(toStdVector(joint_positions_.data), toStdVector(joint_velocities_.data));
        KDL::Frame f_T_ee = KDL::Frame::Identity();
        robot_->addEE(f_T_ee);
        robot_->update(toStdVector(joint_positions_.data), toStdVector(joint_velocities_.data));

        // Compute EE frame
        init_cart_pose_ = robot_->getEEFrame();

        // Compute IK
        KDL::JntArray q(nj);
        robot_->getInverseKinematics(init_cart_pose_, q);

        // Initialize controller
        KDLController controller_(*robot_);

        // EE's trajectory initial position (just an offset)
        Eigen::Vector3d init_position(Eigen::Vector3d(init_cart_pose_.p.data) - Eigen::Vector3d(0, 0, 0.0));

        // EE's trajectory end position (just opposite y)
        Eigen::Vector3d end_position;
        end_position << init_position[0], -init_position[1], init_position[2];

        // Create and test different planners
        double traj_duration = 1.5, acc_duration = 0.375, trajRadius = 0.1;

        // Save the initial joint positions
        init_joint_positions_ = joint_positions_;

        planners_.emplace_back(traj_duration, acc_duration, init_position, end_position); // Linear Trapezoidal
        planners_.back().setProfileType(ProfileType::TRAPEZOIDAL);
        planners_.back().setTrajectoryType(TrajectoryType::LINEAR);

        planners_.emplace_back(traj_duration, acc_duration, init_position, end_position); // Linear Cubic
        planners_.back().setProfileType(ProfileType::CUBIC);
        planners_.back().setTrajectoryType(TrajectoryType::LINEAR);

        planners_.emplace_back(traj_duration, init_position, trajRadius); // Circular Trapezoidal
        planners_.back().setProfileType(ProfileType::TRAPEZOIDAL);
        planners_.back().setTrajectoryType(TrajectoryType::CIRCULAR);

        planners_.emplace_back(traj_duration, init_position, trajRadius); // Circular Cubic
        planners_.back().setProfileType(ProfileType::CUBIC);
        planners_.back().setTrajectoryType(TrajectoryType::CIRCULAR);

        // Create command publisher and timer based on cmd_interface_
        std::string cmd_topic;
        if (cmd_interface_ == "position")
            cmd_topic = "/iiwa_arm_controller/commands";
        else if (cmd_interface_ == "velocity")
            cmd_topic = "/velocity_controller/commands";
        else if (cmd_interface_ == "effort")
            cmd_topic = "/effort_controller/commands";

        cmdPublisher_ = create_publisher<FloatArray>(cmd_topic, 10);
        timer_ = create_wall_timer(std::chrono::milliseconds(100), std::bind(&Iiwa_pub_sub::cmd_publisher, this));

        // Timer to periodically check the parameter
        check_timer_ = create_wall_timer(std::chrono::milliseconds(500), std::bind(&Iiwa_pub_sub::check_planner_index, this));

        RCLCPP_INFO(get_logger(), "Node initialized, waiting for planner selection...");
    }

private:
    void cmd_publisher() {
        if (trajectory_completed_) {
            RCLCPP_INFO(get_logger(), "Trajectory already completed. Waiting for new planner selection...");            
            return;
        }
        if (current_planner_index_ == -1) {
            RCLCPP_INFO(get_logger(), "Planner index not set. Waiting...");
            return;
        }
        // Handle returning to initial position (planner index = 4)
        if (current_planner_index_ == 4) {
            RCLCPP_INFO(get_logger(), "Returning to initial position.");
            double dt = 0.1;  // Time increment
            t_ += dt;
            double alpha = std::min(t_ / 2.0, 1.0);  // Assume movement lasts 2 seconds
            // Specific gains for modes
            double Kp_velocity = 1.0;
            double Kp_effort = 5.0;
            double Kd_effort = 2.0;
            
            for (unsigned int i = 0; i < joint_positions_.data.size(); ++i) {
                if (cmd_interface_ == "position") {
                    // Linear interpolation for position
                    joint_positions_cmd_(i) = (1.0 - alpha) * joint_positions_.data[i] + alpha * init_joint_positions_.data[i];
                } 
                else if (cmd_interface_ == "velocity") {
                    // Compute velocity proportional to position error
                    double position_error = init_joint_positions_.data[i] - joint_positions_.data[i];
                    joint_velocities_cmd(i) = Kp_velocity * position_error;
                } 
                else if (cmd_interface_ == "effort") {
                    // Compute efforts with PD control
                    double position_error = init_joint_positions_.data[i] - joint_positions_.data[i];
                    double velocity_error = -joint_velocities_.data[i];
                    joint_efforts_(i) = Kp_effort * position_error + Kd_effort * velocity_error;
                }
            }
            // Check if initial position is reached
            if (alpha >= 1.0 ||
                (cmd_interface_ == "velocity" && is_close_to_target(joint_positions_, init_joint_positions_)) ||
                (cmd_interface_ == "effort" && is_close_to_target(joint_positions_, init_joint_positions_) && is_velocity_zero(joint_velocities_))) {
                RCLCPP_INFO(get_logger(), "Reached initial position.");
                current_planner_index_ = -1;  // Reset planner index
                t_ = 0;                        // Reset time
                trajectory_completed_ = true;   // Mark trajectory as completed
                set_parameter(rclcpp::Parameter("current_planner_index", -1));
                return;
            }
            // Prepare message to publish based on mode
            std_msgs::msg::Float64MultiArray cmd_msg;
            set_desired_commands();
            cmd_msg.data = desired_commands_;
            cmdPublisher_->publish(cmd_msg);
            RCLCPP_INFO(get_logger(), "Published command message to return to initial position.");
            return;
        }
        //logic for other planners
        RCLCPP_INFO(get_logger(), "cmd_publisher called. iteration: %d", iteration_);
        iteration_ += 1;
        double dt = 0.1;
        t_ += dt;
        RCLCPP_INFO(get_logger(), "Updated time t_: %f", t_);
        KDLController controller_(*robot_);
        
        double max_duration = 2;
        
        if (t_ < max_duration) {
            RCLCPP_INFO(get_logger(), "Calculating trajectory point.");
            trajectory_point p = planners_[current_planner_index_].compute_trajectory(t_);
            RCLCPP_INFO(get_logger(), "Computed trajectory point in joint space. Position: [%f, %f, %f]", p.pos[0], p.pos[1], p.pos[2]);
            
            Eigen::Vector3d error = computeLinearError(p.pos, Eigen::Vector3d(robot_->getEEFrame().p.data));
            Eigen::Vector3d o_error = computeOrientationError(toEigen(init_cart_pose_.M), toEigen(robot_->getEEFrame().M));
            
            std_msgs::msg::Float64MultiArray error_msg;
            error_msg.data.push_back(error.norm());
            error_msg.data.push_back(o_error.norm());
            error_msg.data.push_back(p.pos[0]);
            error_msg.data.push_back(p.pos[1]);
            error_msg.data.push_back(p.pos[2]);
            // Publish on /trajectory_error the values of position error, orientation error, position x, y, z
            errorPublisher_->publish(error_msg);
            
            if (cmd_interface_ == "position") {
                RCLCPP_INFO(get_logger(), "cmd_interface is position. Calculating next frame.");
                KDL::Frame nextFrame;
                nextFrame.M = robot_->getEEFrame().M;  // Same Orientation
                nextFrame.p = robot_->getEEFrame().p + (toKDL(p.vel) + toKDL(error)) * dt;
                
                joint_positions_cmd_ = joint_positions_;
                robot_->getInverseKinematics(nextFrame, joint_positions_cmd_);
                RCLCPP_INFO(get_logger(), "Next joint positions computed using inverse kinematics.");
            } 
            else if (cmd_interface_ == "velocity") {
                RCLCPP_INFO(get_logger(), "cmd_interface is velocity. Calculating differential IK.");
                Vector6d cartvel;
                double Kp_error = 1.0;
                cartvel << p.vel + Kp_error * error, o_error;
                
                joint_velocities_cmd.data = pseudoinverse(robot_->getEEJacobian().data) * cartvel;
                RCLCPP_INFO(get_logger(), "Differential IK computed using Jacobian pseudoinverse.");
            } 
            else if (cmd_interface_ == "effort") {
                RCLCPP_INFO(get_logger(), "cmd_interface is effort. Calculating torques.");
                
                // Desired position
                KDL::Frame desPos;
                desPos.p = KDL::Vector(p.pos[0], p.pos[1], p.pos[2]);
                desPos.M = KDL::Rotation::Identity();  // Same orientation
                // Desired velocity
                KDL::Twist desVel;
                desVel.vel = KDL::Vector(p.vel[0], p.vel[1], p.vel[2]);
                desVel.rot = KDL::Vector(0.0, 0.0, 0.0);  // Zero angular velocity
                // Desired acceleration
                KDL::Twist desAcc;
                desAcc.vel = KDL::Vector(p.acc[0], p.acc[1], p.acc[2]);
                desAcc.rot = KDL::Vector(0.0, 0.0, 0.0);  // Zero angular acceleration
                
                // Control gains
                double Kpp = 2;
                double Kpo = 1;
                double Kdp = 2;
                double Kdo = 1;
                
                // Compute torques using idCntr function
                Eigen::VectorXd torques = controller_.idCntr(desPos, desVel, desAcc, Kpp, Kpo, Kdp, Kdo);
                
                // Assign computed torques to joints
                for (int i = 0; i < torques.size(); ++i) {
                    joint_efforts_(i) = torques(i);
                }
                RCLCPP_INFO(get_logger(), "Effort control torques computed.");
                // Publish torques on torque_error topic
                std_msgs::msg::Float64MultiArray torque_msg;
                for (int i = 0; i < joint_efforts_.rows(); ++i) {
                    torque_msg.data.push_back(joint_efforts_(i));
                }
                torqueErrorPublisher_->publish(torque_msg);
            } 

            robot_->update(toStdVector(joint_positions_.data), toStdVector(joint_velocities_.data));
            RCLCPP_INFO(get_logger(), "Robot model updated.");
            // Prepare desired commands
            set_desired_commands();
            std_msgs::msg::Float64MultiArray cmd_msg;
            cmd_msg.data = desired_commands_;
            cmdPublisher_->publish(cmd_msg);
            RCLCPP_INFO(get_logger(), "Published command message.");
        } 
        else {
            RCLCPP_INFO(get_logger(), "Finished executing planner %u ...", current_planner_index_);
            current_planner_index_ = -1;  // Reset planner index after execution
            t_ = 0;                        // Reset time
            trajectory_completed_ = true;   // Mark trajectory as completed
            RCLCPP_INFO(get_logger(), "Trajectory execution completed.");
            
            if (cmd_interface_ == "velocity") {
                std::fill(desired_commands_.begin(), desired_commands_.end(), 0.0); //set "desire_command" to 0
                std_msgs::msg::Float64MultiArray cmd_msg;
                cmd_msg.data = desired_commands_;
                cmdPublisher_->publish(cmd_msg);
            }
            set_parameter(rclcpp::Parameter("current_planner_index", -1));
        }
    }

    void check_planner_index() {
        // Retrieve the current planner index from the parameter server
        int planner_index;
        get_parameter("current_planner_index", planner_index);
        if (planner_index >= 0 && planner_index < planners_.size() && (planner_index != current_planner_index_ || trajectory_completed_)) {
            current_planner_index_ = planner_index;
            RCLCPP_INFO(get_logger(), "Starting planner %d ...", current_planner_index_);
            t_ = 0;                      // Reset time
            trajectory_completed_ = false; // Reset completion flag
            timer_->reset();             // Restart the timer
        } else if (planner_index == 4) {
            RCLCPP_INFO(get_logger(), "Returning to initial joint positions...");
            current_planner_index_ = planner_index;
            t_ = 0;                      // Reset time
            trajectory_completed_ = false;
            timer_->reset();
        } else {
            RCLCPP_INFO(get_logger(), "Waiting for a valid planner index (0 - %ld) or return command (4)...", planners_.size() - 1);
        }
    }

    void joint_state_subscriber(const sensor_msgs::msg::JointState& sensor_msg) {
        joint_state_available_ = true;
        for (unsigned int i = 0; i < sensor_msg.position.size(); i++) {
            joint_positions_.data[i] = sensor_msg.position[i];
            joint_velocities_.data[i] = sensor_msg.velocity[i];
        }
    }

    bool is_close_to_target(const KDL::JntArray& current, const KDL::JntArray& target, double tolerance = 0.01) {
        for (unsigned int i = 0; i < current.rows(); ++i) {
            if (std::abs(current(i) - target(i)) > tolerance) {
                return false;  // At least one joint is not close to the desired position
            }
        }
        return true;  // All joints are close to the desired position
    }

    bool is_velocity_zero(const KDL::JntArray& velocities, double tolerance = 0.01) {
        for (unsigned int i = 0; i < velocities.rows(); ++i) {
            if (std::abs(velocities(i)) > tolerance) {
                return false;  // At least one joint is still moving
            }
        }
        return true;  // All joints have negligible velocity
    }

    void set_desired_commands() {
        if (cmd_interface_ == "position") {
            for (long int i = 0; i < joint_positions_cmd_.rows(); ++i) {
                desired_commands_[i] = joint_positions_cmd_(i);
            }
        } else if (cmd_interface_ == "velocity") {
            for (long int i = 0; i < joint_velocities_cmd.rows(); ++i) {
                desired_commands_[i] = joint_velocities_cmd(i);
            }
        } else if (cmd_interface_ == "effort") {
            for (long int i = 0; i < joint_efforts_.rows(); ++i) {
                desired_commands_[i] = joint_efforts_(i);
            }
        }
    }

    std::string toString(const Eigen::VectorXd& vec) {
    std::ostringstream oss;
    for (int i = 0; i < vec.size(); ++i) {
        oss << vec[i];
        if (i != vec.size() - 1) {
            oss << ", ";
        }
    }
    return oss.str();
    }
    
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr jointSubscriber_;
    rclcpp::Publisher<FloatArray>::SharedPtr cmdPublisher_;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::TimerBase::SharedPtr check_timer_;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr errorPublisher_;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr torqueErrorPublisher_;

    std::vector<double> desired_commands_ = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    KDL::JntArray joint_positions_;
    KDL::JntArray joint_velocities_;
    KDL::JntArray joint_positions_cmd_;
    KDL::JntArray joint_velocities_cmd;
    KDL::JntArray joint_efforts_;

    std::shared_ptr<KDLRobot> robot_;
    std::vector<KDLPlanner> planners_;

    int iteration_;
    bool joint_state_available_;
    bool trajectory_completed_;
    double t_;
    std::string cmd_interface_;
    KDL::Frame init_cart_pose_;
    int current_planner_index_ = -1;
    KDL::JntArray init_joint_positions_;

    Eigen::Vector3d Kp_;
    Eigen::Vector3d Kd_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Iiwa_pub_sub>());
    rclcpp::shutdown();
    return 1;
}

