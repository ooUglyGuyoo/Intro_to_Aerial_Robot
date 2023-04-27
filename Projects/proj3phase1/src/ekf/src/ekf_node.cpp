#include <iostream>
#include <ros/ros.h>
#include <ros/console.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/Range.h>
#include <nav_msgs/Odometry.h>
#include <Eigen/Eigen>
#include <cmath>

using namespace std;
using namespace Eigen;
ros::Publisher odom_pub;
MatrixXd Q = MatrixXd::Identity(12, 12);
MatrixXd Rt = MatrixXd::Identity(6,6);
static double last_imu_time_stamp = 0.0;
bool update = false;//whether vo is used

//define the state and covariance matrix
Eigen::Vector3d gravity_acc(0, 0, 9.8); // gravity acceleration
Eigen::VectorXd state = VectorXd::Zero(15);// postition, orentation, linear velocity, gyroscope bias, accelerometer bias
Eigen::MatrixXd state_covariance = 0.5*MatrixXd::Identity(15,15);// covariance matrix

void imu_callback(const sensor_msgs::Imu::ConstPtr &msg)
{
    // --- /djiros/imu
    // header: 
    // seq: 71708
    // stamp: 
    //     secs: 1509987829
    //     nsecs: 386356656
    // frame_id: "FLU"
    // orientation: 
    //      x: -0.134212310719
    //      y: 0.0109417611901
    //      z: 0.727404749585
    //      w: 0.672866723957
    // orientation_covariance: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
    // angular_velocity: 
    //      x: -0.113365992904
    //      y: 0.246387138963
    //      z: 0.45954015851
    // angular_velocity_covariance: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
    // linear_acceleration: 
    //      x: -0.610419531614
    //      y: -3.27797826976
    //      z: 9.50987492621
    // linear_acceleration_covariance: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
    // ---

    //=============================================================================================================
    // Get the time interval between two imu messages
    cout << "-imu_callback" << endl;
    double dt = msg->header.stamp.toSec() - last_imu_time_stamp;
    if(!update || dt > 1){
        last_imu_time_stamp = msg->header.stamp.toSec();
        return;
    }
    
    //=============================================================================================================
    // Get accelaration and angular velocity from the input message /djiros/imu
    Eigen::Vector3d acc(msg->linear_acceleration.x, msg->linear_acceleration.y, msg->linear_acceleration.z);
    Eigen::Vector3d angular_vel(msg->angular_velocity.x,msg->angular_velocity.y,msg->angular_velocity.z);

    // G operation: Instantaneous body angular velocity (Lectrue 2 slide page 50)
    // R operation: Rotation matrix from the IMU frame to the world frame. defined on Lecture 9 page 34
    // q: defined on Lecture 9 page 34
    Eigen::Matrix3d G, R;
    double phi = state(3);
    double theta = state(4);
    double psi = state(5);
    Eigen::Vector3d q(state(3), state(4), state(5)); // phi, theta, psi; roll, pitch, yaw
    G <<    cos(q(1)),    0,  -cos(q(0))*sin(q(1)),
            0,             1,  sin(q(0)),
            sin(q(1)),    0,  cos(q(1))*cos(q(0));
    
    R <<    cos(q(2))*cos(q(1))- sin(q(0))*sin(q(2))*sin(q(1)),  -cos(q(0))*sin(q(2)), cos(q(2))*sin(q(1))+cos(q(1))*sin(q(0))*sin(q(2)),
            cos(q(1))*sin(q(2))+cos(q(2))*sin(q(0))*sin(q(1)),   cos(q(0))*cos(q(2)),  sin(q(2))*sin(q(1))-cos(q(2))*sin(q(0))*cos(q(1)),
            -cos(q(0))*sin(q(1)),                               sin(q(0)),           cos(q(0))*cos(q(1));

    Eigen::Matrix3d G_inverse = G.inverse();

    //=============================================================================================================
    // Assumptions
    // x_dot = f(mean_t-1, u_t, 0) ; See Lectrue 9 page 37
    cout << "---Assumptions" << endl;
    Eigen::VectorXd f_t = VectorXd::Zero(15);
    f_t.block<3,1>(0,0) = state.block<3,1>(6,0);//linear velocity
    f_t.block<3,1>(3,0) = (G_inverse) * (angular_vel-Eigen::Vector3d(state(9),state(10),state(11)));//angle_dot
    f_t.block<3,1>(6,0) = gravity_acc + R * (acc-Eigen::Vector3d(state(12),state(13),state(14)));//acceleration

    //=============================================================================================================
    // Linearization
    cout << "---Lineraization" << endl;
    Eigen::MatrixXd A_t = MatrixXd::Zero(15,15);// df/dx partial derivative
    Eigen::MatrixXd U_t = MatrixXd::Zero(15,12);// df/dn partial derivative

    Matrix3d G_inverse_dot, R_dot;

    // Copied from previous year's code, generated code from matlab
    // Calculate G_inverse_dot and R_dot
    // Copy start
    double x4,x5,x6,x7,x8,x9,x10,x11,x12,x13,x14,x15;
    double n1,n2,n3,n4,n5,n6,w1,w2,w3;
    x4 = state(3); x5 = state(4), x6 = state(5), x7 = state(6), x8 = state(7), x9 = state(8), x10 = state(9), x11 = state(10), x12 = state(11), x13 = state(12), x14 = state(13), x15 = state(14);
    n1 = 0, n2 = 0, n3 = 0, n4= 0, n5 = 0, n6 = 0;
    w1 = angular_vel(0), w2 = angular_vel(1), w3 = angular_vel(2);

    G_inverse_dot << 0, (sin(x5)*(n1 - w1 + x10))/(cos(x5)*cos(x5) + sin(x5)*sin(x5)) - (cos(x5)*(n3 - w3 + x12))/(cos(x5)*cos(x5) + sin(x5)*sin(x5)), 0, 
                 (cos(x4)*cos(x5)*(n3 - w3 + x12))/(cos(x4)*cos(x5)*cos(x5) + cos(x4)*sin(x5)*sin(x5)) - (cos(x4)*sin(x5)*(n1 - w1 + x10))/(cos(x4)*cos(x5)*cos(x5) + cos(x4)*sin(x5)*sin(5)) + (cos(x5)*sin(x4)*(sin(x4)*cos(x5)*cos(x5) + sin(x4)*sin(x5)*sin(x5))*(n3 - w3 + x12))/(cos(x4)*cos(x5)*cos(x5) + cos(x4)*sin(x5)*sin(x5))*(cos(x4)*cos(x5)*cos(x5) + cos(x4)*sin(x5)*sin(x5)) - (sin(x4)*sin(x5)*(sin(x4)*cos(x5)*cos(x5) + sin(x4)*sin(x5)*sin(x5))*(n1 - w1 + x10))/(cos(x4)*cos(x5)*cos(x5) + cos(x4)*sin(x5)*sin(x5))*(cos(x4)*cos(x5)*cos(x5) + cos(x4)*sin(x5)*sin(x5)), 
                 - (sin(x4)*sin(x5)*(n3 - w3 + x12))/(cos(x4)*cos(x5)*cos(x5) + cos(x4)*sin(x5)*sin(x5)) - (cos(x5)*sin(x4)*(n1 - w1 + x10))/(cos(x4)*cos(x5)*cos(x5) + cos(x4)*sin(x5)*sin(x5)), 0,
                (sin(x5)*(sin(x4)*cos(x5)*cos(x5) + sin(x4)*sin(x5)*sin(x5))*(n1 - w1 + x10))/(cos(x4)*cos(x5)*cos(x5) + cos(x4)*sin(x5)*sin(x5))*(cos(x4)*cos(x5)*cos(x5) + cos(x4)*sin(x5)*sin(x5)) - (cos(x5)*(sin(x4)*cos(x5)*cos(x5) + sin(x4)*sin(x5)*sin(x5))*(n3 - w3 + x12))/(cos(x4)*cos(x5)*cos(x5) + cos(x4)*sin(x5)*sin(x5))*(cos(x4)*cos(x5)*cos(x5) + cos(x4)*sin(x5)*sin(x5)), (cos(x5)*(n1 - w1 + x10))/(cos(x4)*cos(x5)*cos(x5) + cos(x4)*sin(x5)*sin(x5)) + (sin(x5)*(n3 - w3 + x12))/(cos(x4)*cos(x5)*cos(x5) + cos(x4)*sin(x5)*sin(x5)), 0;
    cout << "-----calculate G_inverse_dot" << endl;
    
    R_dot << acc(1)*sin(phi)*sin(psi) + acc(2)*cos(phi)*cos(theta)*sin(psi) - acc(0)*cos(phi)*sin(theta)*sin(psi), acc(2)*(cos(theta)*cos(psi) - sin(phi)*sin(theta)*sin(psi)) - acc(0)*(cos(psi)*sin(theta) + cos(theta)*sin(phi)*sin(psi)), - acc(0)*(cos(theta)*sin(psi) + cos(psi)*sin(phi)*sin(theta)) - acc(2)*(sin(theta)*sin(psi) - cos(theta)*cos(psi)*sin(phi)) - acc(1)*cos(phi)*cos(psi),
           acc(0)*cos(phi)*cos(psi)*sin(theta) - acc(2)*cos(phi)*cos(theta)*cos(psi) - acc(1)*cos(psi)*sin(phi), acc(2)*(cos(theta)*sin(psi) + cos(psi)*sin(phi)*sin(theta)) - acc(0)*(sin(theta)*sin(psi) - cos(theta)*cos(psi)*sin(phi)),   acc(0)*(cos(theta)*cos(psi) - sin(phi)*sin(theta)*sin(psi)) + acc(2)*(cos(psi)*sin(theta) + cos(theta)*sin(phi)*sin(psi)) - acc(1)*cos(phi)*sin(psi),
           acc(1)*cos(phi) - acc(2)*cos(theta)*sin(phi) + acc(0)*sin(phi)*sin(theta), - acc(0)*cos(phi)*cos(theta) - acc(2)*cos(phi)*sin(theta), 0;
    cout << "-----calculate R_dot" << endl;
    // copy end

    U_t.block<3,3>(3,0) << -(G_inverse);
    U_t.block<3,3>(6,3) << -(R);
    U_t.block<6,6>(9,6) << MatrixXd::Identity(6,6);
    cout << "-----calculate U_t" << endl;

    A_t.block<3,3>(0,6) = MatrixXd::Identity(3,3);
    A_t.block<3,3>(3,3) << G_inverse_dot;
    A_t.block<3,3>(6,3) << R_dot;
    A_t.block<3,3>(3,9) << -(G_inverse);
    A_t.block<3,3>(6,12) << -(R);
    cout << "-----calculate A_t" << endl;

    //=============================================================================================================
    // Discretization
    cout << "---Discretization" << endl;
    Eigen::MatrixXd F_t = MatrixXd::Identity(15,15) + dt * A_t;
    Eigen::MatrixXd V_t = dt * U_t;

    //=============================================================================================================
    // Update mean
    // mean_t = mean_t-1 + dt * f(mean_t-1, u_t, 0)
    cout << "---Update mean" << endl;
    state = state + dt * f_t;
    state(3) = atan2(sin(state(3)),cos(state(3)));
    state(4) = atan2(sin(state(4)),cos(state(4)));
    state(5) = atan2(sin(state(5)),cos(state(5)));

    //=============================================================================================================
    // Update covariance
    // Lecture 9 page 24
    cout << "---Update covariance" << endl;
    state_covariance = F_t * state_covariance * F_t.transpose() + V_t * Q * V_t.transpose();// state covariance propagation

    //=============================================================================================================
    // update the last imu time stamp
    cout << "---update the last imu time stamp" << endl;
    last_imu_time_stamp = msg->header.stamp.toSec();
}

//Rotation from the camera frame to the IMU frame
Eigen::Matrix3d Rcam;

void odom_callback(const nav_msgs::Odometry::ConstPtr &msg)
{
    //your code for update
    // camera position in the IMU frame = (0.05, 0.05, 0)
    // camera orientaion in the IMU frame = Quaternion(0, 1, 0, 0); w x y z, respectively
    //					   RotationMatrix << 1, 0, 0,
    //							             0, -1, 0,
    //                                       0, 0, -1;
    // --- /tag_detector/odom_yourwork
    // header: 
    // seq: 37
    // stamp: 
    //     secs: 1509987822
    //     nsecs:  38856656
    // frame_id: "world"
    // child_frame_id: ''
    // pose: 
    //  pose: 
    //     position: 
    //     x: -0.371295861838
    //     y: -2.10601827502
    //     z: 1.16449813257
    //     orientation: 
    //     x: -0.00204470759888
    //     y: -0.0281980864503
    //     z: -0.0572645533829
    //     w: 0.997958645444
    // covariance: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
    // twist: 
    //  twist: 
    //     linear: 
    //     x: 0.0
    //     y: 0.0
    //     z: 0.0
    //     angular: 
    //     x: 0.0
    //     y: 0.0
    //     z: 0.0
    // covariance: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
    // ---

    cout << "-odom_callback" << endl;

    //=============================================================================================================
    // Transfer imu to world frame
    cout << "---Transfer imu to world frame" << endl;
    Eigen::Vector3d c_t_w, i_t_c, w_t_i; // translation matrix
    Eigen::Matrix3d c_R_w, i_R_c, w_R_i; // rotation matrix
    Eigen::Matrix3d clock_wise_rotate;
    Quaterniond q(msg->pose.pose.orientation.w, msg->pose.pose.orientation.x, msg->pose.pose.orientation.y, msg->pose.pose.orientation.z);
    c_t_w << msg->pose.pose.position.x, msg->pose.pose.position.y, msg->pose.pose.position.z; 
    c_R_w = q.toRotationMatrix();
    i_t_c << 0.05, 0.05, 0;
    i_R_c << 1,  0,  0,
             0,  -1,  0,
             0,  0,  -1;
    w_R_i = c_R_w.transpose() * i_R_c.transpose();
    w_t_i = - w_R_i * i_t_c - (c_R_w.transpose() * c_t_w);

    double phi = asin(w_R_i(2, 1));
    double theta = atan2(-w_R_i(2, 0) / cos(phi), w_R_i(2, 2) / cos(phi));
    double psi = atan2(-w_R_i(0, 1) / cos(phi), w_R_i(1, 1) / cos(phi));

    //=============================================================================================================
    // Assumptions
    cout << "---Assumption" << endl;
    Eigen::VectorXd z_t = VectorXd::Zero(6), gain = VectorXd::Zero(6); // measurement
    z_t << w_t_i(0), w_t_i(1), w_t_i(2), phi, theta, psi;
    cout <<  "-----z_t.transpose(): " << z_t.transpose() << endl;

    //=============================================================================================================
    // Linearization
    cout << "---Lineraization" << endl;
    Eigen::MatrixXd C_t = MatrixXd::Zero(6,15), K_t;
    C_t.block(0,0,6,6).setIdentity(6,6);

    //=============================================================================================================
    // Update
    cout << "---Update" << endl;
    K_t = (C_t * state_covariance * C_t.transpose() + Rt).lu().solve(C_t * state_covariance).transpose(); // Ax=b to get K
    gain = z_t - C_t * state;
    gain(3) = atan2(sin(gain(3)),cos(gain(3)));
    gain(4) = atan2(sin(gain(4)),cos(gain(4)));
    gain(5) = atan2(sin(gain(5)),cos(gain(5)));
    state += K_t * gain;
    state_covariance -= K_t * C_t * state_covariance;
    cout << "-----state.segment(0,3).transpose(): " << state.segment(0,3).transpose() << endl;
    
    Eigen::Matrix3d R_ekf;
    R_ekf << cos(state(5))*cos(state(4))- sin(state(3))*sin(state(5))*sin(state(4)), 
            -cos(state(3))*sin(state(5)), 
            cos(state(5))*sin(state(4))+cos(state(4))*sin(state(3))*sin(state(5)),
            -(cos(state(4))*sin(state(5))+cos(state(5))*(sin(state(3)))*sin(state(4))), 
            -(cos(state(3))*cos(state(5))), 
            -(sin(state(5))*sin(state(4))-cos(state(5))*sin(state(3))*cos(state(4))),
            -(-cos(state(3))*sin(state(4))), 
            -(sin(state(3))), 
            -(cos(state(3))*cos(state(4)));
    // The y and z element of R_ekf is turn into negative since turning down there at the ekf_odometry part doesn't seems to work for unkown reason
    
    //=============================================================================================================
    // Publish odometry topic
    cout << "---Punlish obometry topic" << endl;
    Eigen::Quaterniond Q_ekf(R_ekf);
    nav_msgs::Odometry ekf_odometry;
    ekf_odometry.header.stamp = msg->header.stamp;
    ekf_odometry.header.frame_id = "world";
    ekf_odometry.pose.pose.position.x = -state(0);
    ekf_odometry.pose.pose.position.y = -state(1);
    ekf_odometry.pose.pose.position.z = -state(2);
    ekf_odometry.twist.twist.linear.x = state(6);
    ekf_odometry.twist.twist.linear.y = state(7);
    ekf_odometry.twist.twist.linear.z = state(8);
    ekf_odometry.pose.pose.orientation.w = Q_ekf.w();
    ekf_odometry.pose.pose.orientation.x = Q_ekf.x();
    ekf_odometry.pose.pose.orientation.y = Q_ekf.y();
    ekf_odometry.pose.pose.orientation.z = Q_ekf.z();
    odom_pub.publish(ekf_odometry);
    update = true;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "ekf");
    ros::NodeHandle n("~");
    ros::Subscriber s1 = n.subscribe("imu", 1000, imu_callback);
    ros::Subscriber s2 = n.subscribe("tag_odom", 1000, odom_callback);
    odom_pub = n.advertise<nav_msgs::Odometry>("ekf_odom", 100);
    Rcam = Quaterniond(0, 1, 0, 0).toRotationMatrix();
    cout << "R_cam" << endl << Rcam << endl;
    // Q: imu covariance matrix; 
    // Rt: visual odomtry covariance matrix
    // You should also tune these parameters
    Q.topLeftCorner(6, 6) = 0.1 * Q.topLeftCorner(6, 6);
    Q.bottomRightCorner(6, 6) = 0.04 * Q.bottomRightCorner(6, 6);
    Rt.topLeftCorner(3, 3) = 0.1 * Rt.topLeftCorner(3, 3);
    Rt.bottomRightCorner(3, 3) = 0.1 * Rt.bottomRightCorner(3, 3);
    Rt.bottomRightCorner(1, 1) = 0.1 * Rt.bottomRightCorner(1, 1);

    ros::spin();
}
