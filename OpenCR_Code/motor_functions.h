#ifndef MOTOR_FUNCTIONS_H
#define MOTOR_FUNCTIONS_H

#include "macros.h"
#include"ros_functions.h"
// Structures to hold each motor's state
typedef struct {
  int position;
  unsigned long waitTime;  // in milliseconds
} PositionCommand;

// Update the MotorState struct to include waiting variables
typedef struct {
  PositionCommand positionQueue[QUEUE_SIZE];
  int queueHead;
  int queueTail;
  int32_t desired_position;
  int32_t current_position;
  bool isMoving;
  bool isWaiting;          // Indicates if the motor is in waiting state
  unsigned long waitStartTime; // When the waiting started
  unsigned long waitTime; 
} MotorState;

// Function declarations
void intiDyn();
void initMotor(uint8_t motor_id, uint8_t control_mode);
void setMode(uint8_t motor_id, uint8_t control_mode);
void sortInit();
void setupWheels();
void status();
void homeAll();
void initSort();


void updateMotors();
void initMotorQueue();
void addPositionToQueue(uint8_t motor_id, int position, unsigned long waitTime);
void addPositionToQueue(uint8_t motor_id, int position);
bool isQueueEmpty(int motor_index);
PositionCommand dequeue(int motor_index);

void setPos(uint8_t motor_id, int position);
void getPos(uint8_t motor_id, int32_t &pos);
bool reachedGoal(int motor_index);

void scoopStateMachine();
void sortStateMachine();
void beaconStateMachine();

void stateMachine();






#endif // MOTOR_FUNCTIONS_H