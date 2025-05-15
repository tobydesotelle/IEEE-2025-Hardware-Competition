#include <ros/ros.h>
#include <geometry_msgs/Pose2D.h>
#include <geometry_msgs/Vector3.h>
#include <std_msgs/Bool.h>
#include <cmath>
#include <nav_msgs/Odometry.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>

class PoseController {
public:
    PoseController() {
        ros::NodeHandle nh;

        // Load PID parameters from parameter server
        nh.param("kp_x", kp_x, 1.0);
        nh.param("ki_x", ki_x, 0.000);
        nh.param("kd_x", kd_x, 0.0);
		
		can_proceed_ = false;
        nh.param("kp_y", kp_y, 1.0);
        nh.param("ki_y", ki_y, 0.00);
        nh.param("kd_y", kd_y, 0.0);

        nh.param("kp_theta", kp_theta, 1.0);
        nh.param("ki_theta", ki_theta, 0.0);
        nh.param("kd_theta", kd_theta, 0.0);
		desired_pose.x= 0.79375;
		desired_pose.y= 0.150;
		desired_pose.theta= 0.0;
        // Stopping thresholds (5% of goal)
        nh.param("stop_threshold_position", stop_threshold_position, 0.008);  // 5% of total distance
        nh.param("stop_threshold_theta", stop_threshold_theta, 0.005);  // 5% of total angle (radians)

        // Subscribers & Publisher
        desired_pose_sub = nh.subscribe("/waypoint", 10, &PoseController::desiredPoseCallback, this);
        //current_pose_sub = nh.subscribe("/robot_pose", 10, &PoseController::currentPoseCallback, this);
		current_pose_sub = nh.subscribe("/odometry/filtered", 10, &PoseController::currentPoseCallback, this);
		proceed_signal_sub_ = nh.subscribe("/OpenCR/scoop_done", 10, &PoseController::proceedSignalCallback, this);
        local_velocity_pub = nh.advertise<geometry_msgs::Vector3>("/local_velocities", 10);
		goal_reached_pub = nh.advertise<std_msgs::Bool>("/goal_reached", 10);
        std_msgs::Bool goal_msg;
		goalReached = false;
		goal_reported = false;
		last_time = ros::Time::now();
    }

    void spin() {
        ros::Rate rate(10);  // Run at 50Hz
		//ros::Duration(10.0).sleep();
		while(!can_proceed_){
			ros::spinOnce();
			rate.sleep();
		}
        while (ros::ok()) {
            ros::spinOnce();
			controller();
            rate.sleep();
        }
    }
	void proceedSignalCallback(const std_msgs::Bool::ConstPtr& msg) {
        can_proceed_ = true;
        if (can_proceed_) {
            ROS_INFO("Received proceed signal.");
        }
    }

private:
    ros::Subscriber desired_pose_sub;
    ros::Subscriber current_pose_sub;
    ros::Publisher local_velocity_pub;
	ros::Publisher goal_reached_pub;

	ros::Subscriber proceed_signal_sub_;
	
    geometry_msgs::Pose2D desired_pose;
    geometry_msgs::Pose2D current_pose;
	
	std_msgs::Bool goal_msg;
	
	double pos_x, pos_y, yaw;
	
    double kp_x, ki_x, kd_x;
    double kp_y, ki_y, kd_y;
    double kp_theta, ki_theta, kd_theta;

    double stop_threshold_position;
    double stop_threshold_theta;
	
	bool goalReached;
	bool goal_reported;
	bool can_proceed_;
	
    double prev_error_x = 0, prev_error_y = 0, prev_error_theta = 0;
    double integral_x = 0, integral_y = 0, integral_theta = 0;
    ros::Time last_time;
	double wrapAngle(double angle) {
		while (angle > M_PI) angle -= 2 * M_PI;
		while (angle < -M_PI) angle += 2 * M_PI;
		return angle; 
	}
    void desiredPoseCallback(const geometry_msgs::Pose2D::ConstPtr& msg) {
        desired_pose = *msg;
    }
void controller(){
	ros::Time now = ros::Time::now();
    double dt = (now - last_time).toSec();
    last_time = now;

    if (dt <= 0) return;  // Prevent division by zero

    // Compute errors in the global frame
    double error_x = desired_pose.x - pos_x;
    double error_y = desired_pose.y - pos_y;
    double error_theta = desired_pose.theta - yaw;
	double dist_error = sqrt(error_x*error_x+ error_y *error_y);
	double beta = atan2(error_y, error_x);
	//ROS_INFO("X %f y %f t %f",desired_pose.x,desired_pose.y,desired_pose.theta);
    // Normalize theta error to be between -π and π
    while (error_theta > M_PI) error_theta -= 2 * M_PI;
    while (error_theta < -M_PI) error_theta += 2 * M_PI;

    // Compute PID terms
    integral_x += error_x * dt;
    integral_y += error_y * dt;
    integral_theta += error_theta * dt;

    double derivative_x = (error_x - prev_error_x) / dt;
    double derivative_y = (error_y - prev_error_y) / dt;
    double derivative_theta = (error_theta - prev_error_theta) / dt;

    // Compute control outputs in global frame
    double global_vx = kp_x * error_x + ki_x * integral_x + kd_x * derivative_x;
    double global_vy = kp_y * error_y + ki_y * integral_y + kd_y * derivative_y;
    double omega = kp_theta * error_theta + ki_theta * integral_theta + kd_theta * derivative_theta;

    // Convert to robot's local frame
    double theta = yaw;  // Robot's current heading
    geometry_msgs::Vector3 local_velocity;
    local_velocity.x = global_vx * cos(theta) + global_vy * sin(theta);
    local_velocity.y = -global_vx * sin(theta) + global_vy * cos(theta);
    local_velocity.z = -1*omega;

    // Publish local velocity
    local_velocity_pub.publish(local_velocity);

    // Store previous errors
    prev_error_x = error_x;
    prev_error_y = error_y;
    prev_error_theta = error_theta;
	
	
	
	
	
}
	
void currentPoseCallback(const nav_msgs::Odometry::ConstPtr& msg) {
        pos_x = msg->pose.pose.position.x;
        pos_y = msg->pose.pose.position.y;

        // Convert quaternion to yaw
        tf2::Quaternion q(
            msg->pose.pose.orientation.x,
            msg->pose.pose.orientation.y,
            msg->pose.pose.orientation.z,
            msg->pose.pose.orientation.w
        );

        tf2::Matrix3x3 m(q);
        double roll, pitch;
        m.getRPY(roll, pitch, yaw);  // Extract Yaw (in radians)
    }

};

int main(int argc, char** argv) {
    ros::init(argc, argv, "pose_controller");
    PoseController controller;
    controller.spin();  // Run the control loop at 50Hz

    return 0;
}
