#include "motor_functions.h"
#include "ros_functions.h"
#include "Sensors.h"
#include "macros.h"

#define PORT "/dev/ttyACM0"






int ore_count = 0;

int geo_count = 0;
int neb_count = 0;
bool mag_detected = false;


unsigned long current_time;
unsigned long start_time;

char scoopState = IDLE;
char sortState = IDLE;
char beaconState = IDLE;
char dumpState = IDLE;
bool sort_done = true;
// bool beacon_down = false;
// Arrays to store motor IDs (from 1 to 10)
uint8_t motor_ids[NUMBER_OF_MOTORS] = {5, 6, 7, 8, 9, 10, 11};

 // Duration to wait after reaching position// Array to hold the state of each motor
MotorState motors[NUMBER_OF_MOTORS];




void intiDyn() {
  const char* log;
  // Initialize Dynamixel DEVICE_NAME
  if (!dxl_wb.init(PORT, BAUDRATE, &log)) {
    Serial.println("Failed to initialize Dynamixel");
    Serial.println(log);
  }
  Serial.println("Dynamixel Workbench initialized");

  // Set to Protocol 2.0
  dxl_wb.setPacketHandler(2.0);  // Use Protocol 2.0
}

void initMotor(uint8_t motor_id, uint8_t control_mode) {
  uint16_t model_number = 0;
  const char* log;
  dxl_wb.ping(motor_id, &model_number, &log);
  if (!dxl_wb.ping(motor_id, &model_number, &log)) {
    Serial.print("Failed to ping motor ");
    Serial.print(motor_id);
    Serial.println(log);
  }
  Serial.print("Motor ");
  Serial.print(motor_id);
  Serial.print(" pinged successfully. Model Number: ");
  Serial.println(model_number);
  setMode(motor_id, control_mode);
}
void setMode(uint8_t motor_id, uint8_t control_mode) {
  const char* log;
  if (!dxl_wb.torqueOff(motor_id)) {
    Serial.print("Failed to disable torque for motor");
    Serial.print(motor_id);
    Serial.print(": ");
    Serial.println(log);
    return;
  }
  if (!dxl_wb.setOperatingMode(motor_id, control_mode, &log)) {
    Serial.print("Failed to set control mode for motor ");
    Serial.print(motor_id);
    Serial.println(log);
  }
  Serial.print("Motor ");
  Serial.print(motor_id);
  switch (control_mode) {
    case 0:
      Serial.println(" set to ACCELERATION_MODE");
      break;
    case 1:
      Serial.println(" set to VELOCITY_CONTROL_MODE");
      break;
    case 3:
      Serial.println(" set to POSITION_CONTROL_MODE");
      break;
    case 4:
      Serial.println(" set to EXT_POSITION_CONTROL_MODE");
      break;
  }
  if (!dxl_wb.torqueOn(motor_id, &log)) {
    Serial.print("Failed to enable torque for motor ");
    Serial.print(motor_id);
    Serial.println(log);
  }
  Serial.print("Torque enabled for motor ");
  Serial.println(motor_id);
}
void sortInit() {
  const char* log;
  initMotor(DXL_SORT, EXT_POSITION_CONTROL_MODE);
  dxl_wb.itemWrite(DXL_SORT, "Profile_Velocity", 0);
  dxl_wb.itemWrite(DXL_SORT, "Profile_Acceleration", 0);
  // Center
  if (!dxl_wb.goalPosition(DXL_SORT, 1022)) {
    Serial.print("Failed to set Position: ");
    Serial.print(DXL_SORT);
    Serial.println(log);
  }
}


void setupWheels() {
  uint8_t motor_ids[] = MOTOR_IDS;  // Motor IDs
  for (int i = 0; i < 4; i++) {
    initMotor(motor_ids[i], VELOCITY_CONTROL_MODE);
     dxl_wb.itemWrite(motor_ids[i], "Profile_Acceleration", WHEEL_ACC);
  }
}

void status() {
}

void homeAll(){
	addPositionToQueue(DXL_TILT,TILT_DOWN_POS);
	addPositionToQueue(DXL_SORT,SORT_CENTER);
	addPositionToQueue(DXL_DUMP_R,DUMP_R_DOWN_POS);
	addPositionToQueue(DXL_DUMP_L,DUMP_L_DOWN_POS);
	addPositionToQueue(DXL_SCOOP_R,SCOOP_DOWN_POS - 0);
	addPositionToQueue(DXL_SCOOP_L,-SCOOP_DOWN_POS + 0);
	addPositionToQueue(DXL_BEACON,0);
	addPositionToQueue(DXL_TILT,TILT_UP_POS);
}
void initSort(){
  initMotor(DXL_TILT,EXT_POSITION_CONTROL_MODE);
  initMotor(DXL_SCOOP_L,EXT_POSITION_CONTROL_MODE);
  initMotor(DXL_SCOOP_R,EXT_POSITION_CONTROL_MODE);
  initMotor(DXL_DUMP_L,EXT_POSITION_CONTROL_MODE);
  initMotor(DXL_DUMP_R,EXT_POSITION_CONTROL_MODE);
  initMotor(DXL_SORT,EXT_POSITION_CONTROL_MODE);
  initMotor(DXL_BEACON,EXT_POSITION_CONTROL_MODE);
}


void updateMotors() {
  for (int i = 0; i < NUMBER_OF_MOTORS; i++) {
    uint8_t motor_id = motor_ids[i];
    MotorState &motor = motors[i];

    // If not moving and not waiting, and there's a position in the queue, start moving
    if (!motor.isMoving && !motor.isWaiting && !isQueueEmpty(i)) {
      PositionCommand cmd = dequeue(i);
      if (cmd.position != -1) { // Check for error
        motor.desired_position = cmd.position;
        motor.waitTime = cmd.waitTime;
        setPos(motor_id, cmd.position);
        motor.isMoving = true;
        Serial.print("Motor ");
        Serial.print(motor_id);
        Serial.print(" moving to position: ");
        Serial.println(cmd.position);
      }
    }

    // If moving, check if the motor has reached its desired position
    if (motor.isMoving) {
      if (reachedGoal(i)) {
        Serial.print("Motor ");
        Serial.print(motor_id);
        Serial.print(" reached position: ");
        Serial.println(motor.desired_position);
        motor.isMoving = false;   // Movement complete

        // Start waiting if wait time is greater than 0
        if (motor.waitTime > 0) {
          motor.isWaiting = true;
          motor.waitStartTime = millis();
          Serial.print("Motor ");
          Serial.print(motor_id);
          Serial.print(" waiting for ");
          Serial.print(motor.waitTime);
          Serial.println(" ms");
        }
      } else {
        Serial.print("Motor ");
        Serial.print(motor_id);
        Serial.print(" current position: ");
        Serial.println(motor.current_position);
      }
    }

    // If waiting, check if the wait time has elapsed
    if (motor.isWaiting) {
      unsigned long elapsed = millis() - motor.waitStartTime;
      if (elapsed >= motor.waitTime) {
        motor.isWaiting = false;
        Serial.print("Motor ");
        Serial.print(motor_id);
        Serial.println(" wait time elapsed");
      }
    }
  }
}

void initMotorQueue() {
  for (int i = 0; i < NUMBER_OF_MOTORS; i++) {
    motors[i].queueHead = 0;
    motors[i].queueTail = 0;
    motors[i].desired_position = 0;
    motors[i].current_position = 0;
    motors[i].isMoving = false;
    motors[i].isWaiting = false;
    motors[i].waitStartTime = 0;
    motors[i].waitTime = 0;
  }
}

// Function to add a desired position and wait time to a motor's queue
void addPositionToQueue(uint8_t motor_id, int position, unsigned long waitTime) {
  int motor_index = motor_id - 5; // Motor IDs 5-10 correspond to indices 0-5
  if (motor_index < 0 || motor_index >= NUMBER_OF_MOTORS) {
    Serial.print("Invalid motor ID: ");
    Serial.println(motor_id);
    return;
  }
  MotorState &motor = motors[motor_index];
  if ((motor.queueTail + 1) % QUEUE_SIZE != motor.queueHead) { // Check if the queue is not full
    motor.positionQueue[motor.queueTail].position = position;
    motor.positionQueue[motor.queueTail].waitTime = waitTime;
    motor.queueTail = (motor.queueTail + 1) % QUEUE_SIZE;
    // Serial.print("Added position ");
    // Serial.print(position);
    // Serial.print(" with wait time ");
    // Serial.print(waitTime);
    // Serial.print(" ms to motor ");
    // Serial.println(motor_id);
  } else {
    // Serial.print("Position queue for motor ");
    // Serial.print(motor_id);
    // Serial.println(" is full!");
  }
}
void addPositionToQueue(uint8_t motor_id, int position){
	addPositionToQueue(motor_id,position,0);
}

// Function to check if the queue is empty
bool isQueueEmpty(int motor_index) {
  MotorState &motor = motors[motor_index];
  return (motor.queueHead == motor.queueTail) && !motor.isWaiting;
}

// Function to dequeue the next position command
PositionCommand dequeue(int motor_index) {
  MotorState &motor = motors[motor_index];
  PositionCommand cmd = {-1, 0}; // Default command indicating an error
  if (!isQueueEmpty(motor_index)) {
    cmd = motor.positionQueue[motor.queueHead];
    motor.queueHead = (motor.queueHead + 1) % QUEUE_SIZE;
  } else {
    // Serial.print("Position queue for motor ");
    // Serial.print(motor_ids[motor_index]);
    // Serial.println(" is empty!");
  }
  return cmd;
}

// Function to set the motor position
void setPos(uint8_t motor_id, int position) {
  const char* log;

  // Replace this with your actual code to set the motor position
  // For example:
  if (!dxl_wb.goalPosition(motor_id, position)) {
    // Serial.print("Failed to set position for motor ");
    // Serial.print(motor_id);
    // Serial.print(": ");
    // Serial.println(log);
  } else {
    int motor_index = motor_id - 5;
    if (motor_index >= 0 && motor_index < NUMBER_OF_MOTORS) {
      motors[motor_index].desired_position = position;
    }
  }
}

// Function to get the motor position
void getPos(uint8_t motor_id, int32_t &pos) {
  const char* log;
  // Replace this with your actual code to get the motor position
  // For example:
  if (!dxl_wb.getPresentPositionData(motor_id, &pos, &log)) {
    // Serial.print("Failed to get position for motor ");
    // Serial.print(motor_id);
    // Serial.print(": ");
    // Serial.println(log);
  }
}

// Function to check if the motor has reached its goal
bool reachedGoal(int motor_index) {
  uint8_t motor_id = motor_ids[motor_index];
  MotorState &motor = motors[motor_index];
  getPos(motor_id, motor.current_position);
  if (abs(motor.desired_position - motor.current_position) <= POSITION_THRESHOLD) {
    return true;
  }
  return false;
}

bool first_time = true;
void scoopStateMachine(){
	switch (scoopState) {
		case IDLE:
			// if(first_time){
			// scoopState = UP_RQST;
			// //beaconState = DOWN;
			// first_time = false;
			// }
			if(scoop_request && sort_done){
				scoopState = UP_RQST;
				pubScoopDone(false);
				scoop_request = false;
			}
			if(scoop_tilt_rqst){
				scoopState = SCOOP_TILT_UP;
				pubScoopDone(false);
				addPositionToQueue(DXL_SCOOP_R, SCOOP_TILT_POS, 800);
				addPositionToQueue(DXL_SCOOP_L, -SCOOP_TILT_POS, 800);
				scoop_request = false;
				scoop_tilt_rqst = false;
			}
			// if(bucketGrab){
			// 	scoopState = GRAB_UP;
			// }
		break;
		case UP_RQST:
			
			addPositionToQueue(DXL_SCOOP_R, SCOOP_UP_POS, 800);
			addPositionToQueue(DXL_SCOOP_L, -SCOOP_UP_POS, 800);
			scoopState = UP; 
		break;
		case UP:
			if (isQueueEmpty(DXL_SCOOP_R-5) && isQueueEmpty(DXL_SCOOP_L-5)){
				addPositionToQueue(DXL_SCOOP_R, SCOOP_DOWN_POS);
				addPositionToQueue(DXL_SCOOP_L, -SCOOP_DOWN_POS);
				sortState = START;
				scoopState = DOWN;
			}
		break;
		case DOWN:
			if (isQueueEmpty(DXL_SCOOP_R-5) && isQueueEmpty(DXL_SCOOP_L-5)){
				scoopState = IDLE;
				pubScoopDone(true);
			}
		break;
		case SCOOP_TILT_UP:
			if (isQueueEmpty(DXL_SCOOP_R-5) && isQueueEmpty(DXL_SCOOP_L-5)){
				//delay(100);
				pubScoopDone(true);
				scoopState = SCOOP_TILT_DOWN;
			}
			
		break;
		case SCOOP_TILT_DOWN:
			if(scoop_tilt_rqst){
				scoop_tilt_rqst = false;
        pubScoopDone(true);
				addPositionToQueue(DXL_SCOOP_R, SCOOP_DOWN_POS, 800);
				addPositionToQueue(DXL_SCOOP_L, -SCOOP_DOWN_POS, 800);
				scoopState = IDLE;
			}
			if(scoop_request && sort_done){
				scoopState = UP_RQST;
				scoop_request = false;
				sort_done = false;
			}
		
		break;
		case SCOOP_TILT_UP_IDLE:
			if(scoop_request && sort_done){
				scoop_request = false;
				sort_done = false;
				scoopState = UP_RQST;
			}
		break;
  }
	
}


void sortStateMachine(){
	current_time = millis();
	switch (sortState) {
		case IDLE:
			if(tilt_grab_up){
				sortState = GRAB_UP;
			}
		break;
		case START:
		pubSortDone(false);
		sort_done = false;
		
		if(isQueueEmpty(DXL_SCOOP_R-5) && isQueueEmpty(DXL_SCOOP_L-5)){
			addPositionToQueue(DXL_TILT, TILT_UP_POS+50);
			sortState = UP;
		}
			
		break;
		case UP:
			if(isQueueEmpty(DXL_TILT-5)){
				sortState = DETECT_PART_START_TIMER;
			}
		break;
		case DETECT_PART_START_TIMER:
			//delay(150);
			start_time = current_time;
			sortState = DETECT_PART_START;
			
		break;
		case DETECT_PART_START:
			if(detectOre() && isQueueEmpty(DXL_SORT-5)){
				if(!detectMag()){
					sortState = SORT_GEO_STATE;
					break;
				}
				addPositionToQueue(DXL_SORT, SORT_DETECT_POS-200);
				addPositionToQueue(DXL_SORT, SORT_CENTER);
				addPositionToQueue(DXL_SORT, SORT_DETECT_POS-200);
				addPositionToQueue(DXL_SORT, SORT_CENTER-200);
				addPositionToQueue(DXL_TILT, TILT_UP_POS+100);
				addPositionToQueue(DXL_TILT, TILT_UP_POS+250);
				addPositionToQueue(DXL_TILT, TILT_UP_POS+100);
				addPositionToQueue(DXL_TILT, TILT_UP_POS);
				// addPositionToQueue(DXL_SORT, SORT_DETECT_POS-200);
				// addPositionToQueue(DXL_SORT, SORT_CENTER-200);
				
				
				sortState = DETECT_MAG;
			}
			else{
				//check timeout and sort done if timed out back to idle
				if((current_time - start_time) > DETECT_TIME){	
					sortState = DOWN;
					
				}
			}
		break;
		case DETECT_MAG:
			if(!detectMag()){
				dequeue(DXL_SORT-5);
				dequeue(DXL_SORT-5);
				dequeue(DXL_SORT-5);
				dequeue(DXL_SORT-5);
				sortState = SORT_GEO_STATE;
				break;
			}
			if(isQueueEmpty(DXL_SORT-5)){
				sortState = SORT_NEB_STATE;
        break;
			}
		break;
		case SORT_GEO_STATE:
			addPositionToQueue(DXL_TILT, TILT_DOWN_POS);
			addPositionToQueue(DXL_SORT, SORT_GEO);
			geo_count++;
			sortState = CENTER;
		break;
		case SORT_NEB_STATE:
			addPositionToQueue(DXL_TILT, TILT_DOWN_POS);
			addPositionToQueue(DXL_SORT, SORT_NEB-25);
			neb_count++;
			sortState = CENTER;
		break;
		case CENTER:
			if(isQueueEmpty(DXL_SORT-5)){
				addPositionToQueue(DXL_SORT, SORT_CENTER);
				addPositionToQueue(DXL_TILT, TILT_UP_POS+50);
				
				sortState = CENTERED;
			}
		break;
		case CENTERED:
      
			if(isQueueEmpty(DXL_SORT-5) && isQueueEmpty(DXL_DUMP_L-5) && isQueueEmpty(DXL_DUMP_R-5)){
				// if(geo_count % 4 == 0 && geo_count != 0){
					// addPositionToQueue(DXL_DUMP_R,DUMP_R_DOWN_POS-300);
					// addPositionToQueue(DXL_DUMP_R,DUMP_R_DOWN_POS);
				// }
				// if(neb_count % 4 == 0 && neb_count != 0){
					// addPositionToQueue(DXL_DUMP_L,DUMP_L_DOWN_POS-300);
					// addPositionToQueue(DXL_DUMP_L,DUMP_L_DOWN_POS);
				// }
				sortState = DETECT_PART_START_TIMER;
			}
		break;
		case DOWN:
			addPositionToQueue(DXL_TILT, TILT_DOWN_POS);
			pubScoopDone(true);
			sortState = DOWN_DONE;
		break;
		case DOWN_DONE:
			if(isQueueEmpty(DXL_TILT-5)){
				sortState = IDLE;
				pubScoopDone(false);
				sort_done = true;
			}
		break;
		case GRAB_UP:
			if(isQueueEmpty(DXL_SCOOP_R-5) && isQueueEmpty(DXL_SCOOP_L-5)){
				addPositionToQueue(DXL_TILT, TILT_GRAB_UP);
				sortState = GRAB_UP_IDLE;
			}
		break;
		case GRAB_UP_IDLE:
			if(!tilt_grab_up){
				sortState = DOWN;
			}
		break;
	}
}

void beaconStateMachine(){
	switch (beaconState) {
		case IDLE:
			if(place_beacon){
				beaconState = DOWN;
			}
		break;
		case DOWN:
			addPositionToQueue(DXL_BEACON, 1055);
			beaconState = DOWN_DONE;
		break;
		case DOWN_DONE:
			if(isQueueEmpty(DXL_BEACON-5)){
				actionDone(true);
				beaconState = UP_RQST;
			}
			
		break;
		case UP_RQST:
			if(!place_beacon){
				addPositionToQueue(DXL_BEACON, 0);
				beaconState = UP;
			}
		
		break;
		case UP:
			if(isQueueEmpty(DXL_BEACON-5)){
				beaconState = IDLE;
			}
		break;
  }
	
	
}
void dumpStateMachine(){
	switch (dumpState) {
		case IDLE:
			if(dump_geo || dump_neo){
				dumpState = UP;
			}
		break;
		case UP:
			if(dump_geo){
				dxl_wb.goalPosition(DXL_DUMP_R, DUMP_R_UP_POS);
				delay(500);
				dumpState = UP_SHAKE;
			}
			if(dump_neo){
				dxl_wb.goalPosition(DXL_DUMP_L,DUMP_L_UP_POS);
				delay(500);
				dumpState = UP_SHAKE;
			}
		break;
		case UP_SHAKE:
			if(dump_geo){
				// addPositionToQueue(DXL_DUMP_R,DUMP_R_UP_POS-100,100);
				// addPositionToQueue(DXL_DUMP_R,DUMP_R_UP_POS+100,100);
				// addPositionToQueue(DXL_DUMP_R,DUMP_R_UP_POS,100);
				//addPositionToQueue(DXL_DUMP_R,DUMP_R_DOWN_POS);
				dxl_wb.goalPosition(DXL_DUMP_R,DUMP_R_DOWN_POS);
				dumpState = DOWN_DONE;
			}
			if(dump_neo){
				// addPositionToQueue(DXL_DUMP_L,DUMP_L_UP_POS+100,100);
				// addPositionToQueue(DXL_DUMP_L,DUMP_L_UP_POS-100,100);
				// addPositionToQueue(DXL_DUMP_L,DUMP_L_UP_POS,100);
				//addPositionToQueue(DXL_DUMP_L,DUMP_L_DOWN_POS);
				dxl_wb.goalPosition(DXL_DUMP_L,DUMP_L_DOWN_POS);
				dumpState = DOWN_DONE;
			}
		break;
		case DOWN:
			// if(dump_neo){
				// addPositionToQueue(DXL_DUMP_L,DUMP_L_DOWN_POS);
				// dumpState = DOWN_DONE;
			// }
			// if(dump_geo){
				// addPositionToQueue(DXL_DUMP_R,DUMP_R_DOWN_POS);
				// dumpState = DOWN_DONE;
			// }
		break;
		case DOWN_DONE:
			if(isQueueEmpty(DXL_DUMP_R-5) && isQueueEmpty(DXL_DUMP_L-5)){
				//addPositionToQueue(DXL_DUMP_L,DUMP_L_DOWN_POS);
				//addPositionToQueue(DXL_DUMP_R,DUMP_R_DOWN_POS);
				actionDone(true);
				if(dump_neo)dump_neo = false;
				if(dump_geo)dump_geo = false;
				dumpState = IDLE;
			}
		break;
		
  }
	
	
}



void stateMachine(){
	sortStateMachine();
	scoopStateMachine();
	beaconStateMachine();
	dumpStateMachine();
}

