#include <ros/ros.h>
#include <geometry_msgs/Vector3.h>
#include <std_msgs/Float64MultiArray.h>
#include <std_msgs/Float64.h>
#include <cmath>


// Robot-specific constants
const float WHEEL_RADIUS = 0.054 / 2;  // Wheel radius in meters
const float ROBOT_RADIUS = 0.25 / 2;   // Robot radius in meters

ros::Publisher wheel_1;
ros::Publisher wheel_2;
ros::Publisher wheel_3;
ros::Publisher wheel_4;

std_msgs::Float64 w1;
std_msgs::Float64 w2; 
std_msgs::Float64 w3;
std_msgs::Float64 w4;
void velCap(std_msgs::Float64 w){
	if(w.data >=230) w.data = 230;
	if(w.data <=-230) w.data = -230;
}
	
void velocityCallback(const geometry_msgs::Vector3::ConstPtr& msg)
{
    // Extract desired velocities
    float Vx = msg->x; // Linear velocity in x direction (m/s)
    float Vy = msg->y; // Linear velocity in y direction (m/s)
    float omega = msg->z; // Angular velocity around z-axis (rad/s)
	const float sqrt2_over_2 = std::sqrt(2) / 2.0;


	w1.data = (1.0 / WHEEL_RADIUS) * (-sqrt2_over_2 * Vx + sqrt2_over_2 * Vy + ROBOT_RADIUS * omega);
    w2.data = (1.0 / WHEEL_RADIUS) * (-sqrt2_over_2 * Vx - sqrt2_over_2 * Vy + ROBOT_RADIUS * omega);
    w3.data = (1.0 / WHEEL_RADIUS) * ( sqrt2_over_2 * Vx - sqrt2_over_2 * Vy + ROBOT_RADIUS * omega);
    w4.data = (1.0 / WHEEL_RADIUS) * ( sqrt2_over_2 * Vx + sqrt2_over_2 * Vy + ROBOT_RADIUS * omega);
	
	w1.data = ((w1.data * 60) / (2 * M_PI))/ 0.229;
	w2.data = ((w2.data * 60) / (2 * M_PI))/ 0.229;
	w3.data = ((w3.data * 60) / (2 * M_PI))/ 0.229;
	w4.data = ((w4.data * 60) / (2 * M_PI))/ 0.229;
	
	if(w1.data >=230) w1.data = 230;
	if(w1.data <=-230) w1.data = -230;
	if(w2.data >=230) w2.data = 230;
	if(w2.data <=-230) w2.data = -230;
	if(w3.data >=230) w3.data = 230;
	if(w3.data <=-230) w3.data = -230;
	if(w4.data >=230) w4.data = 230;
	if(w4.data <=-230) w4.data = -230;
	// velCap(w1);
	// velCap(w2);
	// velCap(w3);
	// velCap(w4);
    // Publish the wheel speeds
    wheel_1.publish(w1);
    wheel_2.publish(w2);
    wheel_3.publish(w3);
    wheel_4.publish(w4);

}

int main(int argc, char** argv)
{
    // Initialize the ROS node
    ros::init(argc, argv, "omni_wheel_ik_node");
    ros::NodeHandle nh;

    // Publisher for wheel speeds
    wheel_1 = nh.advertise<std_msgs::Float64>("/OpenCR/wheel_speeds/wheel_1_speed", 10);
    wheel_2 = nh.advertise<std_msgs::Float64>("/OpenCR/wheel_speeds/wheel_2_speed", 10);
    wheel_3 = nh.advertise<std_msgs::Float64>("/OpenCR/wheel_speeds/wheel_3_speed", 10);
    wheel_4 = nh.advertise<std_msgs::Float64>("/OpenCR/wheel_speeds/wheel_4_speed", 10);

    // Subscriber for desired velocities
    ros::Subscriber velocity_sub = nh.subscribe("local_velocities", 10, velocityCallback);
    // Spin to process callbacks
    ros::spin();

    return 0;
}
