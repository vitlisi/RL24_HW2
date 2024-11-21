#include "kdl_control.h"
#include "utils.h"

KDLController::KDLController(KDLRobot &_robot)
{
    robot_ = &_robot;
}

Eigen::VectorXd KDLController::idCntr_2(KDL::JntArray &_qd,
                                      KDL::JntArray &_dqd,
                                      KDL::JntArray &_ddqd,
                                      double _Kp, double _Kd)
{
    // read current state
    Eigen::VectorXd q = robot_->getJntValues();
    Eigen::VectorXd dq = robot_->getJntVelocities();

    // calculate errors
    Eigen::VectorXd e = _qd.data - q;
    Eigen::VectorXd de = _dqd.data - dq;

    Eigen::VectorXd ddqd = _ddqd.data;
    return robot_->getJsim() * (ddqd + _Kd*de + _Kp*e)
            + robot_->getCoriolis() + robot_->getGravity() /*friction compensation?*/;
}

Eigen::VectorXd KDLController::idCntr(KDL::Frame& _desPos, KDL::Twist& _desVel, KDL::Twist& _desAcc,
                                      double _Kpp, double _Kpo, double _Kdp, double _Kdo) {
    
    // Retrieve the current state of the manipulator
    KDL::Frame ee_frame = robot_->getEEFrame();                // Current end-effector position
    KDL::Twist ee_velocity = robot_->getEEVelocity();          // Current end-effector velocity
    Eigen::MatrixXd jacobian = robot_->getEEJacobian().data;   // Analytical Jacobian matrix
    Eigen::VectorXd jacobian_dot_q_dot = robot_->getEEJacDotqDot();  // Product of Jacobian derivative and joint velocities

    // Compute errors in operational space
    Vector6d e;    
    Vector6d edot; 
    computeErrors(_desPos, ee_frame, _desVel, ee_velocity, e, edot);

    // Compute gain matrices
    Eigen::VectorXd Kp(6), Kd(6);
    Kp << _Kpp, _Kpp, _Kpp, _Kpo, _Kpo, _Kpo;
    Kd << _Kdp, _Kdp, _Kdp, _Kdo, _Kdo, _Kdo;
    
    // Compute desired acceleration in operational space
    Eigen::VectorXd acc_des(6);
    acc_des.head(3) = toEigen(_desAcc.vel);  // Desired linear acceleration
    acc_des.tail(3) = toEigen(_desAcc.rot);  // Desired angular acceleration

    // Control output combining desired acceleration and PD terms
    Eigen::VectorXd control_output = acc_des + Kd.cwiseProduct(edot) + Kp.cwiseProduct(e);

    Eigen::VectorXd q_ddot = pseudoinverse(jacobian) * (control_output - jacobian_dot_q_dot);  // Desired joint accelerations

    // Compute control law
    Eigen::MatrixXd Jsim = robot_->getJsim();                // Joint-space inertia matrix, B
    Eigen::VectorXd n = robot_->getCoriolis(); // + robot_->getGravity();  // Coriolis and gravity terms (gravity is zero to Gazebo)
    Eigen::VectorXd tau = Jsim * q_ddot + n;                 // Torque commands

    return tau;
}

