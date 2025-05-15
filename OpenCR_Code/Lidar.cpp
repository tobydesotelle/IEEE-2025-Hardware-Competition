#define READ_SENSOR(i)                                   \
    sensors[i].read();                                   \
    if (sensors[i].timeoutOccurred()) {                  \
        sensors[i].read();                               \
    }                                                    \
    ranges[i] = (sensors[i].ranging_data.range_status == 0) ? \
                (sensors[i].ranging_data.range_mm * 0.001f) : NAN;
#include "Lidar.h"

// Define XSHUT pins for each sensor
const int XSHUT_PINS[5] = { 3, 4, 5, 2, 7 };
const uint8_t I2C_ADDRESSES[5] = { 0x30, 0x31, 0x32, 0x37, 0x34 };  // Assigning addresses to all sensors
const int TOTAL_SENSORS = 4;                                        // Configuring 5 sensors, but only initializing 4
//const int NUM_SENSORS = 4;  
const int sensorCount = 4; 
sensor_msgs::LaserScan scan_msg;
ros::Publisher lidar_scan_pub("scan", &scan_msg);

VL53L1X sensors[NUM_SENSORS];  // Only 4 sensors will be initialized
/*
void setSensorAddress(Adafruit_VL53L1X &sensor, uint8_t newAddress) {
    Wire.beginTransmission(0x29);  // Default address of VL53L1X
    Wire.write(0x8A);  // Address register
    Wire.write(newAddress);  // New I2C address
    Wire.endTransmission();
}
*/
void initLidar3(){
  

  // Disable/reset all sensors by driving their XSHUT pins low.
  for (uint8_t i = 0; i < sensorCount; i++)
  {
    pinMode(XSHUT_PINS[i], OUTPUT);
    digitalWrite(XSHUT_PINS[i], LOW);
  }

  // Enable, initialize, and start each sensor, one by one.
  for (uint8_t i = 0; i < sensorCount; i++)
  {
    // Stop driving this sensor's XSHUT low. This should allow the carrier
    // board to pull it high. (We do NOT want to drive XSHUT high since it is
    // not level shifted.) Then wait a bit for the sensor to start up.
    pinMode(XSHUT_PINS[i], INPUT);
    delay(10);

    sensors[i].setTimeout(500);
    if (!sensors[i].init())
    {
      Serial.print("Failed to detect and initialize sensor ");
      Serial.println(i);
      while (1);
    }

    // Each sensor must have its address changed to a unique value other than
    // the default of 0x29 (except for the last one, which could be left at
    // the default). To make it simple, we'll just count up from 0x2A.
    sensors[i].setAddress(0x30 + i);
    sensors[i].setMeasurementTimingBudget(20000);
    sensors[i].setDistanceMode(VL53L1X::Short);
    sensors[i].startContinuous(5);
  }
  nh.advertise(lidar_scan_pub);
}
void testRead(){
  for (uint8_t i = 0; i < sensorCount; i++)
  {
    //delay(25);
    Serial.print("Sensor ");
    Serial.print(i);
    Serial.print(": ");
    uint16_t reading = sensors[i].read();
    if (sensors[i].timeoutOccurred()) {
      Serial.print(" TIMEOUT");
      // Handle the timeout appropriately
      // For example, you can retry reading the sensor
      reading = sensors[i].read();
      if (sensors[i].timeoutOccurred()) {
        Serial.print(" TIMEOUT again");
        // You can also set a default value or skip this sensor reading
        reading = 0;
      }
    }
    Serial.print(sensors[i].ranging_data.range_mm);
    
    Serial.print('\t');
    Serial.print(sensors[i].ranging_data.range_status);
    Serial.print('\t');
  }
  
  Serial.println();
}
void publishLidarData() {
    // Create a LaserScan message
    sensor_msgs::LaserScan scan_msg;

    // Set the header
    scan_msg.header.stamp = nh.now();
    scan_msg.header.frame_id = "lidar_frame";

    // Precompute constants
    const float full_circle = 2.0f * M_PI; // 360 degrees in radians

    // Define the scan parameters
    scan_msg.angle_min = 0.0f;
    scan_msg.angle_max = full_circle;
    scan_msg.angle_increment = full_circle / sensorCount; // Adjust based on number of sensors
    scan_msg.time_increment = 0.0f; // Assuming synchronous readings
    scan_msg.scan_time = 0.1f; // Adjust to your sensor update rate
    scan_msg.range_min = 0.03f; // Minimum measurable distance (30 mm)
    scan_msg.range_max = 4.0f;  // Maximum measurable distance (4000 mm)

    // Declare the ranges array
    static float ranges[sensorCount]; // Use maximum expected sensor count

    // Collect sensor data
   //static float ranges[4];
	const float sensor_offsets[sensorCount] = {0.14f, 0.14f, 0.14f, 0.14f};
    // Read Sensors
    READ_SENSOR(0)
    READ_SENSOR(1)
    READ_SENSOR(2)
    READ_SENSOR(3)
	
	 for (int i = 0; i < sensorCount; i++) {
        if (!isnan(ranges[i])) { 
            ranges[i] += sensor_offsets[i]; // Add fixed offset
        }
    }

    // Assign the ranges array and its length to the scan message
    scan_msg.ranges        = ranges;
    scan_msg.ranges_length = sensorCount; // Explicitly set the length

    // Publish the scan message
    lidar_scan_pub.publish(&scan_msg);
}



