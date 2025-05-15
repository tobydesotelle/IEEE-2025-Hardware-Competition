
#include "macros.h"

void ros_inti();

void wheel_1_write(const std_msgs::Float64& msg);
void wheel_2_write(const std_msgs::Float64& msg);
void wheel_3_write(const std_msgs::Float64& msg);
void wheel_4_write(const std_msgs::Float64& msg);


void detectLedPub();
void pubScoopDone(bool status);
void pubSortDone(bool status);
void actionDone(bool status);
void scoop_requested(const std_msgs::Bool& msg);
void scoop_tilt_rqst_sub(const std_msgs::Bool& msg);
void tilt_grab(const std_msgs::Bool& msg);
void place_beacon_sub(const std_msgs::Bool& msg);
void dump_geo_rqst(const std_msgs::Bool& msg);
void dump_neo_rqst(const std_msgs::Bool& msg);

void publishMotor1Data();
void publishMotor2Data();
void publishMotor3Data();
void publishMotor4Data();

void control_callback();
void calibrateGyro();
void publishBno();
void eulerToQuaternion(double roll, double pitch, double yaw, geometry_msgs::Quaternion& q);
void readAndPublishIMU();
void writeRegister(uint8_t reg, uint8_t value);

