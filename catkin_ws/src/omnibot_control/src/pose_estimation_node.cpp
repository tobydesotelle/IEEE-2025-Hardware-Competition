#include <ros/ros.h>
#include <std_msgs/Float32.h>
#include <sensor_msgs/Imu.h>
#include <geometry_msgs/Pose2D.h>
#include <geometry_msgs/PoseStamped.h>
#include <vector>
#include <unistd.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_broadcaster.h>



#define ALPHA 1.0
#define BETA 0.99999
#define IMU_AVG 15
// Robot-specific constants
const float WHEEL_RADIUS = 0.054 / 2;  // Wheel radius in meters
const float ROBOT_RADIUS = 0.25 / 2;   // Robot radius in meters
const float TWO_PI = 2.0 * M_PI;
#define HZ 30
const float DT = 1.0/HZ;  // Time step (assumes 50 Hz update rate)

ros::Time prev_time;
bool is_first_time = true;
double yaw = 0.0;
double gyro_bias = 0.0;

// Global pose
geometry_msgs::Pose2D robot_pose; 
geometry_msgs::Quaternion odom_quat;

//encoder values 
std::vector<double> previous_encoder_values(4, 0.0);
std::vector<double> current_encoder_values(4, 0.0);

bool is_initialized = false;
bool is_initialized_imu = false;
bool is_initialized_encoders = false;

float imu_yaw = 0;
float imu_yaw_speed = 0;
float imu_offset = 0;
float prev_imu_yaw_filtered = 0.0;

float prev_imu_yaw = 0.0; // Previous IMU reading
float imu_yaw_filtered = 0.0; // Filtered IMU yaw
float imu_yaw_bandpassed = 0.0;

float imu_avg = 0;
int imu_num_dat = 0;
float imu_yaw_avg[IMU_AVG];
ros::Time current_time;
ros::Time pose_wait;



// Callback functions for each encoder
void encoder1Callback(const std_msgs::Float32::ConstPtr& msg) {
  current_encoder_values[0] = msg->data;
  is_initialized_encoders = true;
}

void encoder2Callback(const std_msgs::Float32::ConstPtr& msg) {
  current_encoder_values[1] = msg->data;
}

void encoder3Callback(const std_msgs::Float32::ConstPtr& msg) {
  current_encoder_values[2] = msg->data;
}

void encoder4Callback(const std_msgs::Float32::ConstPtr& msg) {
  current_encoder_values[3] = msg->data;
}
std::vector<double> a_coeffs_ = {1.0, -1.1430, 0.4652};;
std::vector<double> b_coeffs_= {0.0675, 0.0, -0.0675};;
std::vector<double> z_states_= {0.0, 0.0};;
double gyro_z;
double filtered_z;
float imu_theta = 0.0;
double applyBandPassFilter(double input, std::vector<double>& states)
{
	// Apply the difference equation of the IIR filter
	double output = b_coeffs_[0] * input + states[0];

	// Update states
	states[0] = b_coeffs_[1] * input - a_coeffs_[1] * output + states[1];
	states[1] = b_coeffs_[2] * input - a_coeffs_[2] * output;

	return output;
}
void imuCallback(const std_msgs::Float32& msg) {
    // imu_theta = msg.data * 0.01745; // Convert degrees to radians
    // if (!is_initialized_imu) {
        // imu_offset = imu_theta; 
        // is_initialized_imu = true;
        // ROS_INFO("IMU initialized with offset: %f", imu_offset);
    // }
    // imu_theta -= imu_offset;
	//imu_theta = atan2(sin(imu_theta), cos(imu_theta));
}
double a0 = 1.0;
double a1 = -3.1098; 
double a2 = 3.6996;
double a3 = -2.0064;
double a4 = 0.4306;

double b0 = 0.0466;
double b1 = 0.0;
double b2 = -0.0932;
double b3 = 0.0;
double b4 = 0.0466;

// Filter state variables
double prev_x[4] = {0.0, 0.0, 0.0, 0.0};
double prev_y[4] = {0.0, 0.0, 0.0, 0.0};

bool calibrating = true;
std::vector<double> calibration_samples;
const int calibration_sample_count = 100;

void imuDataCallBack(const sensor_msgs::Imu::ConstPtr& msg){
    // Get the current time from the message header
    ros::Time current_time = msg->header.stamp;
	is_initialized_imu = true;
    if (calibrating)
    {
        // Collect calibration samples while IMU is stationary
        calibration_samples.push_back((-1*msg->angular_velocity.z)*0.017453);
        
        if (calibration_samples.size() >= calibration_sample_count)
        {
            // Calculate the average bias from collected samples
            double sum = 0.0;
            for (double sample : calibration_samples)
            {
                sum += sample;
            }
            gyro_bias = sum / calibration_samples.size();
            calibrating = false;
			is_initialized_imu = true;
            ROS_INFO("Gyro calibration completed. Bias: %f", gyro_bias);
        }
        return;
    }

    // Extract and correct the z-axis gyroscope data
    double gyro_z = (-1 * msg->angular_velocity.z * 0.017453) - gyro_bias;

    // Ensure the data is in radians per second
    // Uncomment if needed
    // gyro_z *= (M_PI / 180.0);

    // Apply band-pass filter
    double y0 = b0 * gyro_z + b1 * prev_x[0] + b2 * prev_x[1] + b3 * prev_x[2] + b4 * prev_x[3]
            - a1 * prev_y[0] - a2 * prev_y[1] - a3 * prev_y[2] - a4 * prev_y[3];

	// Update stored samples
	prev_x[3] = prev_x[2];
	prev_x[2] = prev_x[1];
	prev_x[1] = prev_x[0];
	prev_x[0] = gyro_z;

	prev_y[3] = prev_y[2];
	prev_y[2] = prev_y[1];
	prev_y[1] = prev_y[0];
	prev_y[0] = y0;

    // Calculate dt
    double dt = 0.0;
    if (is_first_time)
    {
        // For the first message, assume dt based on expected sampling rate
        is_first_time = false;
        dt = 1.0 / 100.0; // Example: 100 Hz sampling rate
    }
    else
    {
        dt = (current_time - prev_time).toSec();
    }

    // Update previous time
    prev_time = current_time;

    // Integrate to get yaw (only if dt > 0)
    if (dt > 0)
    {
        yaw += y0 * dt;

        // Wrap yaw angle between -π and π
        yaw = atan2(sin(yaw), cos(yaw));
    }

    // Publish yaw
    
    imu_theta = yaw; // In radians
    

    // Optionally, print the results
    //ROS_INFO("Filtered Gyro Z: %f, dt: %f, Yaw: %f radians", y0, dt, yaw);
}

void calculatePose(ros::Publisher& pose_pub, ros::Publisher& odom_pub,tf::TransformBroadcaster& odom_to_base_broadcaster,tf::TransformBroadcaster& map_to_odom_trans_broadcaster) {
  current_time = ros::Time::now();
    if (is_initialized_encoders && is_initialized_imu && !is_initialized) {
        is_initialized = true;
        previous_encoder_values = current_encoder_values;
        ROS_INFO("Pose initialized!");
    }
	
	if (!is_initialized || !is_initialized_encoders || !is_initialized_imu) {
        // Wait until all sensors are initialized
        return;
    }
	//if(!is_initialized){
	//	previous_encoder_values = current_encoder_values;
	//	return;
	//}
	std::vector<double> wheel_speeds(4, 0.0);
	for(int i = 0; i<4 ; ++i){
		wheel_speeds[i] = (current_encoder_values[i] - previous_encoder_values[i]) / DT;
	}
	previous_encoder_values = current_encoder_values;
  // Compute the robot's linear velocity (Vx, Vy) and angular velocity (omega)
  double Vx = 0, Vy = 0, omega = 0;
	Vx = ((WHEEL_RADIUS * sqrt(2)) / 4.0) * ( -wheel_speeds[0] - wheel_speeds[1] + wheel_speeds[2] + wheel_speeds[3] );
    Vy = ((WHEEL_RADIUS * sqrt(2)) / 4.0) * ( wheel_speeds[0] - wheel_speeds[1] - wheel_speeds[2] + wheel_speeds[3] );
    omega = -1*(WHEEL_RADIUS / (4.0 *ROBOT_RADIUS )) * (wheel_speeds[0] + wheel_speeds[1] + wheel_speeds[2] + wheel_speeds[3] );
	
	double odom_theta = robot_pose.theta + omega * DT;
	
  // Update robot pose
  robot_pose.x += Vx * DT * cos(robot_pose.theta) - Vy * DT * sin(robot_pose.theta);
  robot_pose.y += Vx * DT * sin(robot_pose.theta) + Vy * DT * cos(robot_pose.theta);
  robot_pose.theta = ALPHA * odom_theta + (1 - ALPHA) * imu_theta;
  //ROS_INFO(" ODOM: %f     IMU: %f",odom_theta,imu_theta);
  robot_pose.theta = atan2(sin(robot_pose.theta), cos(robot_pose.theta));

  odom_quat = tf::createQuaternionMsgFromYaw(robot_pose.theta);

  // Publish the odometry message
  nav_msgs::Odometry odom;
  odom.header.stamp = current_time;
  odom.header.frame_id = "odom";
  odom.child_frame_id = "base_link";
  // Set position
  odom.pose.pose.position.x = robot_pose.x;
  odom.pose.pose.position.y = robot_pose.y;
  odom.pose.pose.position.z = 0.0;
  odom.pose.pose.orientation = odom_quat;
  // Set velocity
  odom.twist.twist.linear.x = Vx;
  odom.twist.twist.linear.y = Vy;
  odom.twist.twist.angular.z = omega;
 

  // Publish the updated pose
  pose_pub.publish(robot_pose);
  odom_pub.publish(odom);
	geometry_msgs::TransformStamped odom_trans;
	odom_trans.header.stamp = current_time;
	odom_trans.header.frame_id = "odom";
	odom_trans.child_frame_id = "base_link";

	odom_trans.transform.translation.x = robot_pose.x;
	odom_trans.transform.translation.y = robot_pose.y;
	odom_trans.transform.translation.z = 0.0;
	odom_trans.transform.rotation = odom_quat;
	
	geometry_msgs::TransformStamped transformStamped;
	transformStamped.header.stamp = current_time;
	transformStamped.header.frame_id = "map";
	transformStamped.child_frame_id = "odom";
	transformStamped.transform.translation.x = 0.0;
	transformStamped.transform.translation.y = 0.0;
	transformStamped.transform.translation.z = 0.0;
	tf2::Quaternion q;
	  q.setRPY(0, 0, 0);
	  transformStamped.transform.rotation.x = q.x();
	  transformStamped.transform.rotation.y = q.y();
	  transformStamped.transform.rotation.z = q.z();
	  transformStamped.transform.rotation.w = q.w();

	// Broadcast only the map -> odom transform
	map_to_odom_trans_broadcaster.sendTransform(transformStamped);
	odom_to_base_broadcaster.sendTransform(odom_trans);
}
void timerCallback(const ros::TimerEvent&)
{	
	is_initialized = true;
	previous_encoder_values = current_encoder_values;
	imu_offset = imu_theta;
	robot_pose.x = 0.79375;
	robot_pose.y = 0.150;
	robot_pose.theta = 0.0;
    
    ROS_INFO("Pose intilized!");
}


int main(int argc, char** argv) {
  // Initialize the ROS node
 ros::init(argc, argv, "pose_estimation_node");
    ros::NodeHandle nh;

    // Instantiate ROS publishers and subscribers after ros::init()
    ros::Publisher pose_pub = nh.advertise<geometry_msgs::Pose2D>("/robot_pose", 10);
    ros::Publisher odom_pub = nh.advertise<nav_msgs::Odometry>("/odom", 50);

    ros::Subscriber encoder1_sub = nh.subscribe("/enc/motor1_encoder", 10, encoder1Callback);
    ros::Subscriber encoder2_sub = nh.subscribe("/enc/motor2_encoder", 10, encoder2Callback);
    ros::Subscriber encoder3_sub = nh.subscribe("/enc/motor3_encoder", 10, encoder3Callback);
    ros::Subscriber encoder4_sub = nh.subscribe("/enc/motor4_encoder", 10, encoder4Callback);
	
	//add sub for imu yaw
	ros::Subscriber imu_sub_ = nh.subscribe("/imu/yaw", 10, imuCallback);
	ros::Subscriber imu_data_sub_ = nh.subscribe("/imu/data", 10, imuDataCallBack);
	
	
  // Instantiate Transform Broadcasters
    tf::TransformBroadcaster odom_to_base_broadcaster;
	tf::TransformBroadcaster map_to_odom_trans_broadcaster;
    tf::TransformBroadcaster pose_broadcaster;
  // Initialize robot pose
	  robot_pose.x = 0.79375;
	  robot_pose.y = 0.150;
	  robot_pose.theta = 0.0; 
	geometry_msgs::TransformStamped odom_trans;
	odom_trans.header.stamp = current_time;
	odom_trans.header.frame_id = "odom";
	odom_trans.child_frame_id = "base_link";

	odom_trans.transform.translation.x = robot_pose.x;
	odom_trans.transform.translation.y = robot_pose.y;
	odom_trans.transform.translation.z = 0.0;
	odom_trans.transform.rotation = odom_quat;


	// Broadcast only the map -> odom transform
	odom_to_base_broadcaster.sendTransform(odom_trans);
  // Set the update rate
  ros::Rate rate(HZ);
  //ros::Timer timer = nh.createTimer(ros::Duration(0.1), timerCallback, true);
  while (ros::ok()) {

    ros::spinOnce();
    calculatePose(pose_pub, odom_pub,odom_to_base_broadcaster,map_to_odom_trans_broadcaster);
	//updateTf();
    rate.sleep();
  }

  return 0;
}
