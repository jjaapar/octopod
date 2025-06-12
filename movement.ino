#include <ESP32Servo.h>
#include <math.h>

// Constants for leg dimensions (in mm)
#define COXA_LENGTH 30.0    // Length of coxa (hip joint)
#define FEMUR_LENGTH 60.0   // Length of femur (thigh)
#define TIBIA_LENGTH 80.0   // Length of tibia (shin)

// Servo limits (in degrees)
#define SERVO_MIN 0
#define SERVO_MAX 180
#define SERVO_CENTER 90

// Robot configuration
#define NUM_LEGS 8
#define SERVOS_PER_LEG 3
#define TOTAL_SERVOS (NUM_LEGS * SERVOS_PER_LEG)

// Servo pin assignments for ESP32
const int servoPins[NUM_LEGS][SERVOS_PER_LEG] = {
  {2, 4, 5},    // Leg 0 (Front Right): Coxa, Femur, Tibia
  {18, 19, 21}, // Leg 1 (Mid Right)
  {22, 23, 25}, // Leg 2 (Rear Right)
  {26, 27, 14}, // Leg 3 (Rear Left)
  {12, 13, 15}, // Leg 4 (Mid Left)
  {16, 17, 32}, // Leg 5 (Front Left)
  {33, 34, 35}, // Leg 6 (Additional Front Right)
  {36, 37, 38}  // Leg 7 (Additional Front Left)
};

// Servo objects
Servo legServos[NUM_LEGS][SERVOS_PER_LEG];

// Leg position structure
struct LegPosition {
  float x, y, z;
};

// Joint angles structure
struct JointAngles {
  float coxa, femur, tibia;
};

// Default leg positions (relative to body center)
const LegPosition defaultPositions[NUM_LEGS] = {
  {100, -60, -80},  // Leg 0 (Front Right)
  {60, -100, -80},  // Leg 1 (Mid Right)
  {-60, -100, -80}, // Leg 2 (Rear Right)
  {-100, -60, -80}, // Leg 3 (Rear Left)
  {-100, 60, -80},  // Leg 4 (Mid Left)
  {-60, 100, -80},  // Leg 5 (Front Left)
  {60, 100, -80},   // Leg 6 (Additional Front Right)
  {100, 60, -80}    // Leg 7 (Additional Front Left)
};

// Current leg positions
LegPosition currentPositions[NUM_LEGS];

// Gait parameters
float gaitPhase = 0.0;
float stepHeight = 30.0;
float stepLength = 40.0;
bool movingForward = true;

// Movement state
enum MovementState {
  STANDING,
  WALKING_FORWARD,
  WALKING_BACKWARD,
  TURNING_LEFT,
  TURNING_RIGHT,
  CUSTOM_GAIT
};

MovementState currentState = STANDING;

void setup() {
  Serial.begin(115200);
  Serial.println("ESP32 Octopod Spider Robot Initializing...");
  
  // Initialize servos
  initializeServos();
  
  // Set initial positions
  for (int i = 0; i < NUM_LEGS; i++) {
    currentPositions[i] = defaultPositions[i];
  }
  
  // Move to home position
  moveToHomePosition();
  
  Serial.println("Spider robot ready!");
  Serial.println("Commands: 'f' - forward, 'b' - backward, 'l' - left, 'r' - right, 's' - stop");
}

void loop() {
  // Check for serial commands
  if (Serial.available()) {
    char command = Serial.read();
    handleCommand(command);
  }
  
  // Update movement based on current state
  updateMovement();
  
  // Small delay for smooth operation
  delay(20);
}

void initializeServos() {
  for (int leg = 0; leg < NUM_LEGS; leg++) {
    for (int joint = 0; joint < SERVOS_PER_LEG; joint++) {
      legServos[leg][joint].attach(servoPins[leg][joint]);
      legServos[leg][joint].write(SERVO_CENTER);
    }
  }
  delay(1000); // Allow servos to reach position
}

void handleCommand(char command) {
  switch (command) {
    case 'f':
    case 'F':
      currentState = WALKING_FORWARD;
      Serial.println("Walking forward");
      break;
    case 'b':
    case 'B':
      currentState = WALKING_BACKWARD;
      Serial.println("Walking backward");
      break;
    case 'l':
    case 'L':
      currentState = TURNING_LEFT;
      Serial.println("Turning left");
      break;
    case 'r':
    case 'R':
      currentState = TURNING_RIGHT;
      Serial.println("Turning right");
      break;
    case 's':
    case 'S':
      currentState = STANDING;
      Serial.println("Standing");
      break;
    default:
      Serial.println("Unknown command");
      break;
  }
}

void updateMovement() {
  switch (currentState) {
    case STANDING:
      standStill();
      break;
    case WALKING_FORWARD:
      walkForward();
      break;
    case WALKING_BACKWARD:
      walkBackward();
      break;
    case TURNING_LEFT:
      turnLeft();
      break;
    case TURNING_RIGHT:
      turnRight();
      break;
    case CUSTOM_GAIT:
      customGait();
      break;
  }
}

void standStill() {
  // Maintain default positions
  for (int i = 0; i < NUM_LEGS; i++) {
    currentPositions[i] = defaultPositions[i];
  }
  updateAllLegs();
}

void walkForward() {
  // Octopod gait: alternating tetrapod pattern
  // Group 1: legs 0, 2, 4, 6 move together
  // Group 2: legs 1, 3, 5, 7 move together
  
  gaitPhase += 0.1;
  if (gaitPhase >= 2 * PI) {
    gaitPhase = 0;
  }
  
  for (int leg = 0; leg < NUM_LEGS; leg++) {
    float legPhase = gaitPhase;
    if (leg % 2 == 1) {
      legPhase += PI; // Offset alternate legs by 180 degrees
    }
    
    float x_offset = stepLength * sin(legPhase) * 0.5;
    float z_offset = 0;
    
    // Lift leg during stance phase
    if (sin(legPhase) > 0) {
      z_offset = stepHeight * sin(legPhase);
    }
    
    currentPositions[leg].x = defaultPositions[leg].x + x_offset;
    currentPositions[leg].y = defaultPositions[leg].y;
    currentPositions[leg].z = defaultPositions[leg].z + z_offset;
  }
  
  updateAllLegs();
}

void walkBackward() {
  gaitPhase += 0.1;
  if (gaitPhase >= 2 * PI) {
    gaitPhase = 0;
  }
  
  for (int leg = 0; leg < NUM_LEGS; leg++) {
    float legPhase = gaitPhase;
    if (leg % 2 == 1) {
      legPhase += PI;
    }
    
    float x_offset = -stepLength * sin(legPhase) * 0.5;
    float z_offset = 0;
    
    if (sin(legPhase) > 0) {
      z_offset = stepHeight * sin(legPhase);
    }
    
    currentPositions[leg].x = defaultPositions[leg].x + x_offset;
    currentPositions[leg].y = defaultPositions[leg].y;
    currentPositions[leg].z = defaultPositions[leg].z + z_offset;
  }
  
  updateAllLegs();
}

void turnLeft() {
  gaitPhase += 0.08;
  if (gaitPhase >= 2 * PI) {
    gaitPhase = 0;
  }
  
  for (int leg = 0; leg < NUM_LEGS; leg++) {
    float legPhase = gaitPhase;
    if (leg % 2 == 1) {
      legPhase += PI;
    }
    
    // Different movement for left and right sides
    float direction = (leg < 4) ? 1 : -1; // Right side forward, left side backward
    float x_offset = direction * stepLength * sin(legPhase) * 0.3;
    float z_offset = 0;
    
    if (sin(legPhase) > 0) {
      z_offset = stepHeight * sin(legPhase);
    }
    
    currentPositions[leg].x = defaultPositions[leg].x + x_offset;
    currentPositions[leg].y = defaultPositions[leg].y;
    currentPositions[leg].z = defaultPositions[leg].z + z_offset;
  }
  
  updateAllLegs();
}

void turnRight() {
  gaitPhase += 0.08;
  if (gaitPhase >= 2 * PI) {
    gaitPhase = 0;
  }
  
  for (int leg = 0; leg < NUM_LEGS; leg++) {
    float legPhase = gaitPhase;
    if (leg % 2 == 1) {
      legPhase += PI;
    }
    
    // Different movement for left and right sides
    float direction = (leg < 4) ? -1 : 1; // Right side backward, left side forward
    float x_offset = direction * stepLength * sin(legPhase) * 0.3;
    float z_offset = 0;
    
    if (sin(legPhase) > 0) {
      z_offset = stepHeight * sin(legPhase);
    }
    
    currentPositions[leg].x = defaultPositions[leg].x + x_offset;
    currentPositions[leg].y = defaultPositions[leg].y;
    currentPositions[leg].z = defaultPositions[leg].z + z_offset;
  }
  
  updateAllLegs();
}

void customGait() {
  // Wave gait - each leg moves in sequence
  gaitPhase += 0.05;
  if (gaitPhase >= 2 * PI) {
    gaitPhase = 0;
  }
  
  for (int leg = 0; leg < NUM_LEGS; leg++) {
    float legPhase = gaitPhase + (leg * PI / 4); // Offset each leg by 45 degrees
    
    float x_offset = stepLength * sin(legPhase) * 0.5;
    float z_offset = 0;
    
    if (sin(legPhase) > 0) {
      z_offset = stepHeight * sin(legPhase);
    }
    
    currentPositions[leg].x = defaultPositions[leg].x + x_offset;
    currentPositions[leg].y = defaultPositions[leg].y;
    currentPositions[leg].z = defaultPositions[leg].z + z_offset;
  }
  
  updateAllLegs();
}

void updateAllLegs() {
  for (int leg = 0; leg < NUM_LEGS; leg++) {
    JointAngles angles = calculateInverseKinematics(currentPositions[leg], leg);
    moveServo(leg, 0, angles.coxa);   // Coxa
    moveServo(leg, 1, angles.femur);  // Femur
    moveServo(leg, 2, angles.tibia);  // Tibia
  }
}

JointAngles calculateInverseKinematics(LegPosition pos, int legIndex) {
  JointAngles angles;
  
  // Calculate coxa angle (horizontal rotation)
  angles.coxa = atan2(pos.y, pos.x) * 180.0 / PI;
  
  // Calculate distance from coxa joint to foot
  float horizontal_dist = sqrt(pos.x * pos.x + pos.y * pos.y) - COXA_LENGTH;
  float vertical_dist = -pos.z; // Negative because z is downward
  
  // Distance from femur joint to foot
  float leg_length = sqrt(horizontal_dist * horizontal_dist + vertical_dist * vertical_dist);
  
  // Check if position is reachable
  if (leg_length > (FEMUR_LENGTH + TIBIA_LENGTH)) {
    leg_length = FEMUR_LENGTH + TIBIA_LENGTH - 5; // Slightly less than max reach
  }
  
  // Calculate femur angle using law of cosines
  float cos_femur = (FEMUR_LENGTH * FEMUR_LENGTH + leg_length * leg_length - TIBIA_LENGTH * TIBIA_LENGTH) / 
                    (2 * FEMUR_LENGTH * leg_length);
  cos_femur = constrain(cos_femur, -1.0, 1.0);
  
  float femur_angle = acos(cos_femur) * 180.0 / PI;
  float ground_angle = atan2(vertical_dist, horizontal_dist) * 180.0 / PI;
  angles.femur = femur_angle + ground_angle;
  
  // Calculate tibia angle
  float cos_tibia = (FEMUR_LENGTH * FEMUR_LENGTH + TIBIA_LENGTH * TIBIA_LENGTH - leg_length * leg_length) / 
                    (2 * FEMUR_LENGTH * TIBIA_LENGTH);
  cos_tibia = constrain(cos_tibia, -1.0, 1.0);
  angles.tibia = 180.0 - (acos(cos_tibia) * 180.0 / PI);
  
  // Apply leg-specific offsets and constraints
  angles.coxa = constrainServoAngle(angles.coxa + getCoxaOffset(legIndex));
  angles.femur = constrainServoAngle(angles.femur + getFemurOffset(legIndex));
  angles.tibia = constrainServoAngle(angles.tibia + getTibiaOffset(legIndex));
  
  return angles;
}

float getCoxaOffset(int legIndex) {
  // Leg-specific offsets to account for mounting orientation
  float offsets[NUM_LEGS] = {0, 45, 90, 135, 180, 225, 270, 315};
  return offsets[legIndex];
}

float getFemurOffset(int legIndex) {
  // Adjust for servo mounting orientation
  return (legIndex < 4) ? 90 : 90; // Right side vs left side
}

float getTibiaOffset(int legIndex) {
  // Adjust for servo mounting orientation
  return (legIndex < 4) ? 90 : 90;
}

float constrainServoAngle(float angle) {
  return constrain(angle, SERVO_MIN, SERVO_MAX);
}

void moveServo(int leg, int joint, float angle) {
  int servoAngle = (int)angle;
  servoAngle = constrain(servoAngle, SERVO_MIN, SERVO_MAX);
  legServos[leg][joint].write(servoAngle);
}

void moveToHomePosition() {
  Serial.println("Moving to home position...");
  
  // Calculate angles for default positions
  for (int leg = 0; leg < NUM_LEGS; leg++) {
    JointAngles angles = calculateInverseKinematics(defaultPositions[leg], leg);
    moveServo(leg, 0, angles.coxa);
    moveServo(leg, 1, angles.femur);
    moveServo(leg, 2, angles.tibia);
  }
  
  delay(2000); // Allow time to reach position
  Serial.println("Home position reached.");
}

// Utility function to print current leg positions (for debugging)
void printLegPositions() {
  for (int i = 0; i < NUM_LEGS; i++) {
    Serial.print("Leg ");
    Serial.print(i);
    Serial.print(": X=");
    Serial.print(currentPositions[i].x);
    Serial.print(" Y=");
    Serial.print(currentPositions[i].y);
    Serial.print(" Z=");
    Serial.println(currentPositions[i].z);
  }
}
