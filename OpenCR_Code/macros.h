#define DXL_FR_ID 1          // ID of the front-right Dynamixel
#define DXL_FL_ID 2          // ID of the front-left Dynamixel
#define DXL_RR_ID 3          // ID of the rear-right Dynamixel
#define DXL_RL_ID 4          // ID of the rear-left Dynamixel
#define DXL_TILT 5           // ID of tilt Dynamixel
#define DXL_SORT 6           // ID of sort Dynamixel
#define DXL_SCOOP_R 7          // ID of first collection Dynamixel
#define DXL_SCOOP_L 8          // ID of second collection Dynamixel
#define DXL_DUMP_L 9
#define DXL_DUMP_R 10
#define DXL_BEACON 11

#define HOMEING_VEL \
  { 0, 0, 0, 0, 0, 50, 0, 50, 50, 50, 50}


#define WHEEL_ACC 0

#define ACCELERATION_MODE 0  // Argument for wheel mode
#define VELOCITY_CONTROL_MODE 1
#define POSITION_CONTROL_MODE 3
#define EXT_POSITION_CONTROL_MODE 4
#define POSITION_TRESHOLD 50
#define TIMEOUT 5000 //5 Second timeout
#define STOP  0
#define HOMING_VELOCITY 50  // Velocity of Homing function
#define LOAD_THRESHOLD 150  // Value of load to complete the homing task
#define MOTOR_IDS { 1, 2, 3, 4 }
#define SAMPLING_PERIOD 20   // Sampling period in microseconds
#define BAUDRATE 57600       // Baudrate of Dynamixel communication

#if defined(__OPENCM904__)
#define DEVICE_NAME "DXL_PORT"
#elif defined(__OPENCR__)
#define DEVICE_NAME "DXL_PORT"
#endif
#define TILT_UP_POS 2200
#define TILT_DOWN_POS 2840
#define TILT_GRAB_UP  2400

#define SCOOP_UP_POS 1150
#define SCOOP_DOWN_POS	-50
#define SCOOP_TILT_POS 250
#define SCOOP_R_DOWN_POS 3075
#define SCOOP_L_DOWN_POS 2025
#define SCOOP_L_UP_POS 900

#define DUMP_L_DOWN_POS 492
#define DUMP_L_UP_POS -550
#define DUMP_R_DOWN_POS 2018
#define DUMP_R_UP_POS 1000

#define SORT_CENTER 1050
#define SORT_NEB	-775
#define SORT_GEO	2855
#define SORT_DETECT_POS	1800
//States



#define NUMBER_OF_MOTORS 7               // Total number of motors
#define POSITION_THRESHOLD 50             // Adjust based on your motor's precision
#define QUEUE_SIZE 20                     // Max number of positions in each motor's queue


#include <stdint.h>
#include <DynamixelWorkbench.h>
#include <ros.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/LaserScan.h>
#include <std_msgs/Float64MultiArray.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Char.h>
#include <tf/tf.h>
#include <tf/transform_broadcaster.h>
#include <Wire.h>
#include <IMU.h>
//#include "Adafruit_VL53L1X.h"
#include <limits>
#include <VL53L1X.h>




#define IDLE 0
#define UP_RQST 1
#define UP 2
#define DOWN 3
#define GRAB_UP 4

#define START 5
#define DETECT_PART_START_TIMER 6
#define DETECT_PART_START 7
#define DETECT_MAG 8
#define SORT_GEO_STATE 9
#define SORT_NEB_STATE 10
#define CENTER 11
#define CENTERED 12
#define DOWN_DONE 13
#define GRAB_UP 14
#define GRAB_UP_IDLE 15
#define SCOOP_TILT_UP 16
#define SCOOP_TILT_UP_IDLE 17
#define UP_SHAKE 18
#define SCOOP_TILT_DOWN 19
#define DETECT_TIME	800


#define NUM_SENSORS 4
#define LIGHT_TRESHOLD 200


extern DynamixelWorkbench dxl_wb;
extern ros::NodeHandle nh;


extern bool scoop_request;
extern bool scoop_tilt_rqst;
extern bool tilt_grab_up;
extern bool place_beacon;
extern bool dump_geo;
extern bool dump_neo;