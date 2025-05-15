

#include "macros.h"
#include "ros_functions.h"
#include "Lidar.h"
#include <sensor_msgs/MagneticField.h>


#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>




Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28); // 0x28 is default I2C address

char sort_control;
//bool sort_done;
//cIMU imu;
sensor_msgs::Imu imu_msg;
sensor_msgs::MagneticField mag_msg;
std_msgs::Float32 motor1_data, motor2_data, motor3_data, motor4_data;
std_msgs::Bool scoop_done_msg;
std_msgs::Bool sort_done_msg;
std_msgs::Bool action_done_msg;
std_msgs::Bool LED_msg;






std_msgs::Float32 yaw_msg;
std_msgs::Char sort_status;

ros::Publisher motor1_pub("/enc/motor1_encoder", &motor1_data);
ros::Publisher motor2_pub("/enc/motor2_encoder", &motor2_data);
ros::Publisher motor3_pub("/enc/motor3_encoder", &motor3_data);
ros::Publisher motor4_pub("/enc/motor4_encoder", &motor4_data);



ros::Publisher imu_pub("/imu/data", &imu_msg);
ros::Publisher mag_pub("/imu/mag", &mag_msg);

ros::Publisher yaw_pub("/imu/yaw", &yaw_msg);
geometry_msgs::TransformStamped tfs_msg;
tf::TransformBroadcaster tfbroadcaster;

ros::Publisher scoop_done_pub("/OpenCR/scoop_done", &scoop_done_msg);
ros::Publisher sort_done_pub("/OpenCR/sort_done", &sort_done_msg);
ros::Publisher action_done_pub("/OpenCR/action_done", &sort_done_msg);
ros::Publisher Dyn_cntl_pub("/servo/scoop_pose/Status", &sort_status);
ros::Publisher detect_LED_pub("/OpenCR/LED", &LED_msg);
 
// Declare the range messages

//ros::Subscriber<std_msgs::Char>  Dyn_cntl_sub("/OpenCR/servo/control_sub", control_callback);
ros::Subscriber<std_msgs::Float64> w1_topic("/OpenCR/wheel_speeds/wheel_1_speed", wheel_1_write);
ros::Subscriber<std_msgs::Float64> w2_topic("/OpenCR/wheel_speeds/wheel_2_speed", wheel_2_write);
ros::Subscriber<std_msgs::Float64> w3_topic("/OpenCR/wheel_speeds/wheel_3_speed", wheel_3_write);
ros::Subscriber<std_msgs::Float64> w4_topic("/OpenCR/wheel_speeds/wheel_4_speed", wheel_4_write);

ros::Subscriber<std_msgs::Bool> scoop_request_sub("/OpenCR/scoop", scoop_requested);
ros::Subscriber<std_msgs::Bool> scoop_tilt_request_sub("/OpenCR/scoop_tilt", scoop_tilt_rqst_sub);
ros::Subscriber<std_msgs::Bool> tilt_grab_sub("/OpenCR/tilt_grab_up", tilt_grab);
ros::Subscriber<std_msgs::Bool> beacon_sub("/OpenCR/beacon_place", place_beacon_sub);
ros::Subscriber<std_msgs::Bool> dump_geo_sub("/OpenCR/dump_geo", dump_geo_rqst);
ros::Subscriber<std_msgs::Bool> dump_neo_sub("/OpenCR/dump_neo", dump_neo_rqst);
uint8_t dxl_FL = DXL_FL_ID;
uint8_t dxl_FR = DXL_FR_ID;
uint8_t dxl_RL = DXL_RL_ID;
uint8_t dxl_RR = DXL_RR_ID;

bool scoop_request = false;
bool scoop_tilt_rqst = false;
bool tilt_grab_up = false;
bool place_beacon = false;
bool dump_geo = false;
bool dump_neo = false;


void ros_inti() {
  nh.initNode();
  Wire.begin();
  Wire.setClock(100000); // use 400 kHz I2C
  nh.subscribe(w1_topic);
  nh.subscribe(w2_topic);
  nh.subscribe(w3_topic);
  nh.subscribe(w4_topic);
  
  nh.subscribe(scoop_request_sub);
  nh.subscribe(scoop_tilt_request_sub);
  nh.subscribe(tilt_grab_sub);
  nh.subscribe(beacon_sub);
  nh.subscribe(dump_geo_sub);
  nh.subscribe(dump_neo_sub);
  nh.advertise(scoop_done_pub);
  nh.advertise(sort_done_pub);
  
  //nh.advertise(lidar_scan_pub);

  nh.advertise(motor1_pub);
  nh.advertise(motor2_pub);
  nh.advertise(motor3_pub);
  nh.advertise(motor4_pub);
  
  nh.advertise(action_done_pub);
  
  nh.advertise(imu_pub);
  nh.advertise(mag_pub);
  nh.advertise(yaw_pub);

	tfbroadcaster.init(nh);
  bno.begin();
	delay(100);
	//imu.begin();
  
  
}


int32_t radToDyn(const std_msgs::Float64& msg) {
  if (msg.data >= 1023) return 1023;
  if (msg.data <= -1023) return -1023;
  return msg.data;
}

//Action Done
void detectLedPub(){
	LED_msg.data = true;
	detect_LED_pub.publish(&LED_msg);
}
void pubScoopDone(bool status) {
  scoop_done_msg.data = status;
  scoop_done_pub.publish(&scoop_done_msg);
}
void pubSortDone(bool status) {
  sort_done_msg.data = status;
  sort_done_pub.publish(&scoop_done_msg);
}
void actionDone(bool status) {
	action_done_msg.data = status;
	action_done_pub.publish(&action_done_msg);	
}


//Action requests
void scoop_requested(const std_msgs::Bool& msg) {
  scoop_request = msg.data;
}
void scoop_tilt_rqst_sub(const std_msgs::Bool& msg){
	scoop_tilt_rqst = msg.data;
}
void tilt_grab(const std_msgs::Bool& msg){
	tilt_grab_up = msg.data;
}
void place_beacon_sub(const std_msgs::Bool& msg){
	place_beacon = msg.data;
}
void dump_geo_rqst(const std_msgs::Bool& msg){
	dump_geo = msg.data;
}
void dump_neo_rqst(const std_msgs::Bool& msg){
	dump_neo = msg.data;
}


void wheel_1_write(const std_msgs::Float64& msg) {

  dxl_wb.goalVelocity(2, radToDyn(msg));
}
void wheel_2_write(const std_msgs::Float64& msg) {
  dxl_wb.goalVelocity(3, radToDyn(msg));
}
void wheel_3_write(const std_msgs::Float64& msg) {
  dxl_wb.goalVelocity(4, radToDyn(msg));
}
void wheel_4_write(const std_msgs::Float64& msg) {
  dxl_wb.goalVelocity(1, radToDyn(msg));
}

// The publishers are remapped for the topic becasue of math
//id 1 =wheel 2
//id 2 = wheel 3
//id 3 = wheel 4
//id 4 = wheel 1
void publishMotor1Data() {
  float radian;
  if (dxl_wb.getRadian(1, &radian)) {
    motor1_data.data = radian;
  } else {
    nh.logerror("Failed to get encoder data for motor 2");
    motor1_data.data = 0.0;
  }
  motor1_pub.publish(&motor2_data);
}

// Function to publish encoder data for Motor 2
void publishMotor2Data() {
  float radian;
  if (dxl_wb.getRadian(2, &radian)) {
    motor2_data.data = radian;
  } else {
    nh.logerror("Failed to get encoder data for motor 2");
    motor2_data.data = 0.0;
  }
  motor2_pub.publish(&motor3_data);
}

// Function to publish encoder data for Motor 3
void publishMotor3Data() {
  float radian;
  if (dxl_wb.getRadian(3, &radian)) {
    motor3_data.data = radian;
  } else {
    nh.logerror("Failed to get encoder data for motor 3");
    motor3_data.data = 0.0;
  }
  motor3_pub.publish(&motor4_data);
}

// Function to publish encoder data for Motor 4
void publishMotor4Data() {
  float radian;
  if (dxl_wb.getRadian(4, &radian)) {
    motor4_data.data = radian;
  } else {
    nh.logerror("Failed to get encoder data for motor 4");
    motor4_data.data = 0.0;
  }
  motor4_pub.publish(&motor1_data);
}

float gyroBias[3] = {0.0, 0.0, 0.0};
const int calibrationSamples = 100;

// void calibrateGyro() {
//     for (int i = 0; i < calibrationSamples; i++) {
//         imu.update();
//         gyroBias[0] += imu.gyroData[0];
//         gyroBias[1] += imu.gyroData[1];
//         gyroBias[2] += imu.gyroData[2];
//         delay(10); // Adjust delay as needed
//     }
//     gyroBias[0] /= calibrationSamples;
//     gyroBias[1] /= calibrationSamples;
//     gyroBias[2] /= calibrationSamples;
// }





// void readAndPublishIMU() {
//   static uint32_t tTime[3];
//   static uint32_t imu_time = 0;

//   if ((millis() - tTime[1]) >= 20) {
//     imu.update();
//     tTime[1] = millis();

//     imu_msg.header.stamp = nh.now();
//     imu_msg.header.frame_id = "imu_link";

//     imu_msg.angular_velocity.x = (imu.gyroData[0] - gyroBias[0]) * 0.06098f * 0.017453;
// 	imu_msg.angular_velocity.y = (imu.gyroData[1] - gyroBias[1]) * 0.06098f * 0.017453;
// 	imu_msg.angular_velocity.z = (imu.gyroData[2] - gyroBias[2]) * 0.06098f * 0.017453;


//     imu_msg.linear_acceleration.x = imu.accData[0]* 0.004787f;
//     imu_msg.linear_acceleration.y = imu.accData[1]* 0.004787f;
//     imu_msg.linear_acceleration.z = imu.accData[2]* 0.004787f;
//     double yaw = atan2(2.0 * (imu.quat[0] * imu.quat[3] + imu.quat[1] * imu.quat[2]),
//                        1.0 - 2.0 * (imu.quat[2] * imu.quat[2] + imu.quat[3] * imu.quat[3]));

//     geometry_msgs::Quaternion new_quat;
//     new_quat.x = 0;
//     new_quat.y = 0;
//     new_quat.z = sin(yaw / 2.0);
//     new_quat.w = cos(yaw / 2.0);
//     imu_msg.orientation = new_quat;
//     imu_msg.orientation_covariance[0] = 1000.0;  // Trust orientation with a large covariance (if using fusion)

//     // Set angular velocity covariance: low uncertainty in gyro data
//     imu_msg.angular_velocity_covariance[0] = 0.01;  // Covariance for angular velocity around x-axis
//     imu_msg.angular_velocity_covariance[4] = 0.01;  // Covariance for angular velocity around y-axis
//     imu_msg.angular_velocity_covariance[8] = 0.01;  // Covariance for angular velocity around z-axis

//     // Set linear acceleration covariance: small uncertainty in accelerometer data
//     imu_msg.linear_acceleration_covariance[0] = 0.1;  // Covariance for acceleration along x-axis
//     imu_msg.linear_acceleration_covariance[4] = 0.1;  // Covariance for acceleration along y-axis
//     imu_msg.linear_acceleration_covariance[8] = 0.1;  // Covariance for acceleration along z-axis
//     // Publish the IMU message
//     imu_pub.publish(&imu_msg);
//     //yaw_msg.data = imu.rpy[2]; // Assuming IMU.rpy[2] is yaw in radians
//     //yaw_pub.publish(&yaw_msg);
//   }
// }



void publishBno() {
    sensors_event_t orientationData, angVelocityData, linearAccelData, magData;

    // Get sensor data
    bno.getEvent(&orientationData, Adafruit_BNO055::VECTOR_EULER);
    bno.getEvent(&angVelocityData, Adafruit_BNO055::VECTOR_GYROSCOPE);
    bno.getEvent(&linearAccelData, Adafruit_BNO055::VECTOR_LINEARACCEL);
    bno.getEvent(&magData, Adafruit_BNO055::VECTOR_MAGNETOMETER);

    // Debug prints
    // Serial.println("BNO055 Sensor Readings:");
    // Serial.print("Orientation (Euler): ");
    // Serial.print(orientationData.orientation.x);
    // Serial.print(", ");
    // Serial.print(orientationData.orientation.y);
    // Serial.print(", ");
    // Serial.println(orientationData.orientation.z);

    // Serial.print("Gyro (rad/s): ");
    // Serial.print(angVelocityData.gyro.x);
    // Serial.print(", ");
    // Serial.print(angVelocityData.gyro.y);
    // Serial.print(", ");
    // Serial.println(angVelocityData.gyro.z);

    // Serial.print("Linear Acceleration (m/s^2): ");
    // Serial.print(linearAccelData.acceleration.x);
    // Serial.print(", ");
    // Serial.print(linearAccelData.acceleration.y);
    // Serial.print(", ");
    // Serial.println(linearAccelData.acceleration.z);

    // Serial.print("Magnetometer (uT): ");
    // Serial.print(magData.magnetic.x);
    // Serial.print(", ");
    // Serial.print(magData.magnetic.y);
    // Serial.print(", ");
    // Serial.println(magData.magnetic.z);

    imu_msg.header.stamp = nh.now();
    imu_msg.header.frame_id = "imu_link";
	
	  double roll = orientationData.orientation.x * M_PI / 180.0;
	  double pitch = orientationData.orientation.y * M_PI / 180.0;
	  double yaw = orientationData.orientation.z * M_PI / 180.0;
	
	  eulerToQuaternion(roll, pitch, yaw, imu_msg.orientation);
	
    // Orientation covariance
    imu_msg.orientation_covariance[0] = 0.1;
    imu_msg.orientation_covariance[4] = 0.1;
    imu_msg.orientation_covariance[8] = 0.1;

    // Angular Velocity (Gyroscope) - Radians per second
    imu_msg.angular_velocity.x = angVelocityData.gyro.x;
    imu_msg.angular_velocity.y = angVelocityData.gyro.y;
    imu_msg.angular_velocity.z = angVelocityData.gyro.z;
    imu_msg.angular_velocity_covariance[0] = 0.001;
    imu_msg.angular_velocity_covariance[4] = 0.001;
    imu_msg.angular_velocity_covariance[8] = 0.001;

    // Linear Acceleration (m/s^2)
    imu_msg.linear_acceleration.x = linearAccelData.acceleration.x;
    imu_msg.linear_acceleration.y = linearAccelData.acceleration.y;
    imu_msg.linear_acceleration.z = linearAccelData.acceleration.z;
    imu_msg.linear_acceleration_covariance[0] = 0.01;
    imu_msg.linear_acceleration_covariance[4] = 0.01;
    imu_msg.linear_acceleration_covariance[8] = 0.01;

    imu_pub.publish(&imu_msg);

    mag_msg.header.stamp = nh.now();
    mag_msg.header.frame_id = "imu_link";

    mag_msg.magnetic_field.x = magData.magnetic.x;
    mag_msg.magnetic_field.y = magData.magnetic.y;
    mag_msg.magnetic_field.z = magData.magnetic.z;
    mag_msg.magnetic_field_covariance[0] = 0.01;
    mag_msg.magnetic_field_covariance[4] = 0.01;
    mag_msg.magnetic_field_covariance[8] = 0.01;

    mag_pub.publish(&mag_msg);
}
void eulerToQuaternion(double roll, double pitch, double yaw, geometry_msgs::Quaternion& q) {
    // Convert roll, pitch, yaw (radians) to quaternion
    double cy = cos(yaw * 0.5);
    double sy = sin(yaw * 0.5);
    double cp = cos(pitch * 0.5);
    double sp = sin(pitch * 0.5);
    double cr = cos(roll * 0.5);
    double sr = sin(roll * 0.5);

    q.w = cr * cp * cy + sr * sp * sy;
    q.x = sr * cp * cy - cr * sp * sy;
    q.y = cr * sp * cy + sr * cp * sy;
    q.z = cr * cp * sy - sr * sp * cy;
}


	



