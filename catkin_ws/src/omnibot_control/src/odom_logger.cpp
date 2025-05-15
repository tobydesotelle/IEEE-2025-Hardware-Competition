#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <fstream>

// Class to handle odom data logging
class OdomLogger {
public:
    OdomLogger() {
        // Subscribe to the /odom topic
        odom_sub_ = nh_.subscribe("/odom", 1000, &OdomLogger::odomCallback, this);

        // Open the file to save odom data
        file_.open("/home/ubuntu/catkin_ws/odom_data.csv", std::ios::out | std::ios::trunc);
        if (file_.is_open()) {
            // Write the CSV header
            file_ << "Time,Position_X,Position_Y,Position_Z,Orientation_X,Orientation_Y,Orientation_Z,Orientation_W\n";
        } else {
            ROS_ERROR("Failed to open file for writing odometry data.");
        }
    }

    ~OdomLogger() {
        if (file_.is_open()) {
            file_.close();
        }
    }

    void odomCallback(const nav_msgs::Odometry::ConstPtr& msg) {
        if (file_.is_open()) {
            // Get timestamp
            double time = msg->header.stamp.toSec();

            // Get position
            double pos_x = msg->pose.pose.position.x;
            double pos_y = msg->pose.pose.position.y;
            double pos_z = msg->pose.pose.position.z;

            // Get orientation
            double ori_x = msg->pose.pose.orientation.x;
            double ori_y = msg->pose.pose.orientation.y;
            double ori_z = msg->pose.pose.orientation.z;
            double ori_w = msg->pose.pose.orientation.w;

            // Write data to CSV
            file_ << time << ","
                  << pos_x << "," << pos_y << "," << pos_z << ","
                  << ori_x << "," << ori_y << "," << ori_z << "," << ori_w << "\n";
        }
    }

private:
    ros::NodeHandle nh_;
    ros::Subscriber odom_sub_;
    std::ofstream file_;
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "odom_logger");
    OdomLogger odom_logger;
    ros::spin();
    return 0;
}
