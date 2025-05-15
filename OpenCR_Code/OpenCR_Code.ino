#include "ros_functions.h"
#include "motor_functions.h"
#include "Lidar.h"
#include "Sensors.h"

DynamixelWorkbench dxl_wb;
ros::NodeHandle nh;
int pos;

bool led_started = false;


unsigned long currentMillis;

unsigned long previous50HzMillis = 0;
const unsigned long interval50Hz = 20;  // 20 milliseconds

unsigned long previous30HzMillis = 0;
const unsigned long interval30Hz = 33;  // 33 milliseconds

unsigned long previous10HzMillis = 0;
const unsigned long interval10Hz = 100;  // 100 milliseconds

unsigned long previous5HzMillis = 0;
const unsigned long interval5Hz = 200;  // 100 milliseconds

void setup() {
  Serial.begin(2000000);
  ros_inti();
  initSensors();
  intiDyn();
  initMotorQueue();
  initSort();
  //initLidar3();
  setupWheels();
 //calibrateGyro();
  dxl_wb.torqueOff(DXL_SCOOP_L);
  dxl_wb.torqueOff(DXL_SCOOP_R);
  dxl_wb.itemWrite(DXL_SCOOP_L, "Profile_Acceleration", 0);
  dxl_wb.itemWrite(DXL_SCOOP_R, "Profile_Acceleration", 0);
  dxl_wb.torqueOn(DXL_SCOOP_L);
  dxl_wb.torqueOn(DXL_SCOOP_R);
  
  
  dxl_wb.torqueOff(DXL_DUMP_L);
  dxl_wb.torqueOff(DXL_DUMP_R);
  dxl_wb.itemWrite(DXL_DUMP_L, "Profile_Acceleration", 0);
  dxl_wb.itemWrite(DXL_DUMP_R, "Profile_Acceleration", 0);
  dxl_wb.itemWrite(DXL_DUMP_L, "Profile_Velocity", 80);
  dxl_wb.itemWrite(DXL_DUMP_R, "Profile_Velocity", 80);
  dxl_wb.torqueOn(DXL_DUMP_L);
  dxl_wb.torqueOn(DXL_DUMP_R);
  

	homeAll();
  //addPositionToQueue(DXL_SCOOP_R, SCOOP_UP_POS, 800);
  // addPositionToQueue(DXL_SCOOP_L, -SCOOP_UP_POS, 800);
  // addPositionToQueue(DXL_DUMP_L,DUMP_L_UP_POS);
  // addPositionToQueue(DXL_DUMP_R,DUMP_R_UP_POS);
}
unsigned long starttime;
unsigned long endtime;
void loop() {
  currentMillis = millis();

  nh.spinOnce();  // If this needs to run every cycle

  





  

  // 50Hz Tasks
  if (currentMillis - previous50HzMillis >= interval50Hz) {
    previous50HzMillis = currentMillis;
	  
    //readAndPublishIMU();
    starttime = millis();
    publishBno();
    
     
    endtime = millis();
  Serial.println(endtime - starttime);
    // Place functions you want to run at 50Hz here
  }

  // 30Hz Tasks
  if (currentMillis - previous30HzMillis >= interval30Hz) {
    previous30HzMillis = currentMillis;
    publishMotor1Data();
    publishMotor2Data();
    publishMotor3Data();
    publishMotor4Data();
  }

  //10Hz Tasks
  if (currentMillis - previous10HzMillis >= interval10Hz) {
    previous10HzMillis = currentMillis;
    stateMachine();
    updateMotors();
	
  }
  
if (currentMillis - previous5HzMillis >= interval5Hz) {
    previous5HzMillis = currentMillis;
    // publishLidarData();
	if(!led_started){
		if(detectLed()){
			led_started = true;
      addPositionToQueue(DXL_TILT,TILT_DOWN_POS);
		}
	}
    
    
  }
  
 
}
