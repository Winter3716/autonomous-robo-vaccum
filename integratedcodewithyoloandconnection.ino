// ============================================================
// KUCHO BOT - COMPLETE SYSTEM + MAPPING
// ESP32 Arduino Core 3.x
//
// HARDWARE:
//  - HC-SR04 x3
//  - TCRT5000
//  - L298N
//  - Relay + suction motor
//  - ESP32-CAM / YOLO via HTTP
//
// MAPPING:
//  - NO MPU6050
//  - NO wheel encoders
//  - Position estimated from motor runtime
//  - 90-degree turns update heading
//
// HTTP:
//  /status -> robot status
//  /map    -> mapping data
//  /yolo   -> receives YOLO detection
// ============================================================


// ============================================================
// BLYNK
// ============================================================

#define BLYNK_TEMPLATE_ID "TMPL6nvFalc3N"
#define BLYNK_TEMPLATE_NAME "Quickstart Template"
#define BLYNK_AUTH_TOKEN "aqVZ1jrAua9PsErxIQN7HG5RIxIZzUcB"

#include <Arduino.h>
#include <WiFi.h>
#include <WebServer.h>
#include <BlynkSimpleEsp32.h>
#include <math.h>


// ============================================================
// PIN DEFINITIONS
// ============================================================

// ---------------- ULTRASONIC ----------------

#define FRONT_TRIG 26
#define FRONT_ECHO 25

#define LEFT_TRIG 14
#define LEFT_ECHO 27

#define RIGHT_TRIG 13
#define RIGHT_ECHO 12


// ---------------- TCRT5000 ----------------

#define TCRT_DO 23
#define TCRT_AO 34


// ---------------- RELAY ----------------

#define RELAY_PIN 4


// ---------------- L298N ----------------

// LEFT MOTOR
#define ENA 32
#define IN1 18
#define IN2 19

// RIGHT MOTOR
#define ENB 33
#define IN3 21
#define IN4 22


// ============================================================
// WIFI
// ============================================================


char ssid[] = "itswaifaiiiu";
char pass[] = "123456789";

BlynkTimer timer;

WebServer server(80);


// ============================================================
// ULTRASONIC VARIABLES
// ============================================================

int frontDistance = -1;
int leftDistance = -1;
int rightDistance = -1;


// ============================================================
// TCRT VARIABLES
// ============================================================

int floorReflection = 0;

String floorType = "Unknown";

bool edgeDetected = false;


// ============================================================
// ROBOT VARIABLES
// ============================================================

bool obstacleDetected = false;

bool suctionStatus = false;

bool robotPower = true;

String robotStatus = "Idle";

String mappingStatus = "Mapping";

int currentSpeed = 0;

int wifiSignal = 0;


// ============================================================
// YOLO VARIABLES
// ============================================================

String yoloObject = "None";

float yoloConfidence = 0.0;

int yoloX = 0;
int yoloY = 0;
int yoloWidth = 0;
int yoloHeight = 0;

unsigned long lastYOLOTime = 0;


// ============================================================
// MOTOR PWM
// ============================================================

#define PWM_FREQ 1000
#define PWM_RESOLUTION 8


// ============================================================
// FLOOR SPEEDS
// ============================================================

#define SPEED_STARTUP 255

#define SPEED_CARPET 128

#define SPEED_CEMENT 191

#define SPEED_SMOOTH 255

#define SPEED_UNKNOWN 128


// ============================================================
// MOTOR CALIBRATION
// ============================================================

// Left motor is faster.

#define LEFT_MOTOR_CORRECTION 0.75


// ============================================================
// NAVIGATION
// ============================================================

#define OBSTACLE_DISTANCE 5

#define SIDE_BLOCK_DISTANCE 5

#define REVERSE_TIME 700

#define TURN_90_TIME 650


// ============================================================
// MAPPING SETTINGS
// ============================================================

// Virtual map size:
// 500 cm x 500 cm

#define MAP_WIDTH_CM 500

#define MAP_HEIGHT_CM 500

// Each grid cell = 5 cm

#define CELL_SIZE_CM 5

#define GRID_WIDTH 100

#define GRID_HEIGHT 100


// Occupancy values

#define CELL_UNKNOWN 0
#define CELL_FREE 1
#define CELL_OCCUPIED 2


// ============================================================
// OCCUPANCY GRID
// ============================================================

uint8_t occupancyGrid[GRID_HEIGHT][GRID_WIDTH];


// ============================================================
// ROBOT POSITION
// ============================================================

// Robot starts at map center.

float robotX = 250.0;

float robotY = 250.0;

// 0° = +X direction

float robotHeading = 0.0;


// ============================================================
// MOVEMENT SPEED ESTIMATES
// ============================================================

// IMPORTANT:
//
// These are INITIAL estimates.
// We will calibrate these experimentally later.
//
// Approximate forward speeds.

#define SPEED_CM_S_CARPET 8.0
#define SPEED_CM_S_CEMENT 13.0
#define SPEED_CM_S_SMOOTH 18.0
#define SPEED_CM_S_UNKNOWN 8.0


// Time tracking

unsigned long movementStartTime = 0;

bool movementTracking = false;


// ============================================================
// FLOOR DETECTION
// ============================================================

String detectFloorType(int reflection)
{
  if (reflection >= 2300)
  {
    return "Smooth Floor";
  }

  else if (reflection >= 300 && reflection <= 1500)
  {
    return "Cement Floor";
  }

  else if (reflection >= 100 && reflection < 300)
  {
    return "Carpet";
  }

  else
  {
    return "Unknown";
  }
}


// ============================================================
// TCRT AVERAGE
// ============================================================

int readReflection()
{
  long sum = 0;

  for (int i = 0; i < 10; i++)
  {
    sum += analogRead(TCRT_AO);

    delay(5);
  }

  return sum / 10;
}


// ============================================================
// READ TCRT
// ============================================================

void readTCRT()
{
  edgeDetected =
    (digitalRead(TCRT_DO) == HIGH);

  floorReflection =
    readReflection();

  floorType =
    detectFloorType(floorReflection);
}


// ============================================================
// ULTRASONIC
// ============================================================

int readUltrasonic(int trigPin, int echoPin)
{
  digitalWrite(trigPin, LOW);

  delayMicroseconds(2);

  digitalWrite(trigPin, HIGH);

  delayMicroseconds(10);

  digitalWrite(trigPin, LOW);

  long duration =
    pulseIn(
      echoPin,
      HIGH,
      30000
    );

  if (duration == 0)
  {
    return -1;
  }

  float distance =
    duration * 0.0343 / 2.0;

  if (distance >= 2 &&
      distance <= 400)
  {
    return (int)distance;
  }

  return -1;
}


// ============================================================
// READ ALL ULTRASONICS
// ============================================================

void readUltrasonicSensors()
{
  frontDistance =
    readUltrasonic(
      FRONT_TRIG,
      FRONT_ECHO
    );

  delay(20);

  leftDistance =
    readUltrasonic(
      LEFT_TRIG,
      LEFT_ECHO
    );

  delay(20);

  rightDistance =
    readUltrasonic(
      RIGHT_TRIG,
      RIGHT_ECHO
    );

  delay(20);
}


// ============================================================
// SUCTION
//
// NC CONFIGURATION:
//
// HIGH = relay released
//      = COM connected to NC
//      = suction ON
//
// LOW = suction OFF
// ============================================================

void suctionON()
{
  digitalWrite(
    RELAY_PIN,
    HIGH
  );

  suctionStatus = true;
}


void suctionOFF()
{
  digitalWrite(
    RELAY_PIN,
    LOW
  );

  suctionStatus = false;
}


// ============================================================
// FLOOR SPEED
// ============================================================

int getFloorSpeed()
{
  if (floorType == "Smooth Floor")
  {
    return SPEED_SMOOTH;
  }

  else if (floorType == "Cement Floor")
  {
    return SPEED_CEMENT;
  }

  else if (floorType == "Carpet")
  {
    return SPEED_CARPET;
  }

  return SPEED_UNKNOWN;
}


// ============================================================
// ESTIMATED MOVEMENT SPEED
// ============================================================

float getEstimatedSpeedCmS()
{
  if (floorType == "Smooth Floor")
  {
    return SPEED_CM_S_SMOOTH;
  }

  else if (floorType == "Cement Floor")
  {
    return SPEED_CM_S_CEMENT;
  }

  else if (floorType == "Carpet")
  {
    return SPEED_CM_S_CARPET;
  }

  return SPEED_CM_S_UNKNOWN;
}


// ============================================================
// UPDATE ROBOT POSITION
//
// Uses motor runtime.
//
// NO MPU.
// NO ENCODER.
// ============================================================

void updateEstimatedPosition()
{
  if (!movementTracking)
  {
    return;
  }

  unsigned long now =
    millis();

  unsigned long elapsed =
    now - movementStartTime;

  if (elapsed == 0)
  {
    return;
  }

  float elapsedSeconds =
    elapsed / 1000.0;

  float speed =
    getEstimatedSpeedCmS();

  float distance =
    speed * elapsedSeconds;

  float angle =
    robotHeading *
    PI / 180.0;

  robotX +=
    distance * cos(angle);

  robotY +=
    distance * sin(angle);

  // Keep inside map

  robotX =
    constrain(
      robotX,
      0,
      MAP_WIDTH_CM - 1
    );

  robotY =
    constrain(
      robotY,
      0,
      MAP_HEIGHT_CM - 1
    );

  movementStartTime = now;
}


// ============================================================
// START MOVEMENT TRACKING
// ============================================================

void startMovementTracking()
{
  if (!movementTracking)
  {
    movementStartTime =
      millis();

    movementTracking = true;
  }
}


// ============================================================
// STOP MOVEMENT TRACKING
// ============================================================

void stopMovementTracking()
{
  if (movementTracking)
  {
    updateEstimatedPosition();

    movementTracking = false;
  }
}


// ============================================================
// UPDATE HEADING
// ============================================================

void updateHeading(float change)
{
  robotHeading += change;

  while (robotHeading < 0)
  {
    robotHeading += 360;
  }

  while (robotHeading >= 360)
  {
    robotHeading -= 360;
  }

  Serial.print("Heading: ");

  Serial.print(robotHeading);

  Serial.println(" degrees");
}


// ============================================================
// MOTOR SPEED
// ============================================================

void setMotorSpeed(int speed)
{
  currentSpeed = speed;

  int leftSpeed =
    speed * LEFT_MOTOR_CORRECTION;

  leftSpeed =
    constrain(
      leftSpeed,
      0,
      255
    );

  speed =
    constrain(
      speed,
      0,
      255
    );

  ledcWrite(
    ENA,
    leftSpeed
  );

  ledcWrite(
    ENB,
    speed
  );
}


// ============================================================
// RAW MOTOR SPEED
// ============================================================

void setRawMotorSpeed(int speed)
{
  speed =
    constrain(
      speed,
      0,
      255
    );

  int leftSpeed =
    speed * LEFT_MOTOR_CORRECTION;

  leftSpeed =
    constrain(
      leftSpeed,
      0,
      255
    );

  ledcWrite(
    ENA,
    leftSpeed
  );

  ledcWrite(
    ENB,
    speed
  );
}


// ============================================================
// FORWARD
// ============================================================

void forward()
{
  digitalWrite(
    IN1,
    HIGH
  );

  digitalWrite(
    IN2,
    LOW
  );

  digitalWrite(
    IN3,
    HIGH
  );

  digitalWrite(
    IN4,
    LOW
  );

  startMovementTracking();
}


// ============================================================
// BACKWARD
// ============================================================

void backward()
{
  stopMovementTracking();

  digitalWrite(
    IN1,
    LOW
  );

  digitalWrite(
    IN2,
    HIGH
  );

  digitalWrite(
    IN3,
    LOW
  );

  digitalWrite(
    IN4,
    HIGH
  );
}


// ============================================================
// LEFT TURN
// ============================================================

void turnLeft()
{
  stopMovementTracking();

  digitalWrite(
    IN1,
    LOW
  );

  digitalWrite(
    IN2,
    HIGH
  );

  digitalWrite(
    IN3,
    HIGH
  );

  digitalWrite(
    IN4,
    LOW
  );
}


// ============================================================
// RIGHT TURN
// ============================================================

void turnRight()
{
  stopMovementTracking();

  digitalWrite(
    IN1,
    HIGH
  );

  digitalWrite(
    IN2,
    LOW
  );

  digitalWrite(
    IN3,
    LOW
  );

  digitalWrite(
    IN4,
    HIGH
  );
}


// ============================================================
// STOP
// ============================================================

void stopMotors()
{
  stopMovementTracking();

  digitalWrite(
    IN1,
    LOW
  );

  digitalWrite(
    IN2,
    LOW
  );

  digitalWrite(
    IN3,
    LOW
  );

  digitalWrite(
    IN4,
    LOW
  );

  ledcWrite(
    ENA,
    0
  );

  ledcWrite(
    ENB,
    0
  );

  currentSpeed = 0;
}


// ============================================================
// MARK GRID CELL
// ============================================================

void setGridCell(
  int gx,
  int gy,
  uint8_t value
)
{
  if (
    gx >= 0 &&
    gx < GRID_WIDTH &&
    gy >= 0 &&
    gy < GRID_HEIGHT
  )
  {
    occupancyGrid[gy][gx] =
      value;
  }
}


// ============================================================
// MARK FREE CELLS BETWEEN ROBOT
// AND OBSTACLE
// ============================================================

void mapRay(
  float distance,
  float angleDegrees
)
{
  if (distance <= 0)
  {
    return;
  }

  float angle =
    angleDegrees *
    PI / 180.0;

  // Sample every 5 cm

  for (
    float d = CELL_SIZE_CM;
    d < distance;
    d += CELL_SIZE_CM
  )
  {
    float px =
      robotX +
      d * cos(angle);

    float py =
      robotY +
      d * sin(angle);

    int gx =
      (int)(px /
      CELL_SIZE_CM);

    int gy =
      (int)(py /
      CELL_SIZE_CM);

    setGridCell(
      gx,
      gy,
      CELL_FREE
    );
  }


  // Mark obstacle at end

  float ox =
    robotX +
    distance * cos(angle);

  float oy =
    robotY +
    distance * sin(angle);

  int obstacleGX =
    (int)(ox /
    CELL_SIZE_CM);

  int obstacleGY =
    (int)(oy /
    CELL_SIZE_CM);

  setGridCell(
    obstacleGX,
    obstacleGY,
    CELL_OCCUPIED
  );
}


// ============================================================
// UPDATE OCCUPANCY GRID
// ============================================================

void updateMapping()
{
  // Update current robot position first

  updateEstimatedPosition();


  // Front direction

  mapRay(
    frontDistance,
    robotHeading
  );


  // Left direction

  mapRay(
    leftDistance,
    robotHeading - 90
  );


  // Right direction

  mapRay(
    rightDistance,
    robotHeading + 90
  );
}


// ============================================================
// TURN LEFT 90
// ============================================================

void rotateLeft90()
{
  Serial.println(
    "Turning LEFT 90 degrees..."
  );

  robotStatus =
    "Turning Left";

  turnLeft();

  setRawMotorSpeed(
    SPEED_CEMENT
  );

  delay(
    TURN_90_TIME
  );

  stopMotors();

  // Update mapping heading

  updateHeading(-90);

  delay(200);
}


// ============================================================
// TURN RIGHT 90
// ============================================================

void rotateRight90()
{
  Serial.println(
    "Turning RIGHT 90 degrees..."
  );

  robotStatus =
    "Turning Right";

  turnRight();

  setRawMotorSpeed(
    SPEED_CEMENT
  );

  delay(
    TURN_90_TIME
  );

  stopMotors();

  // Update mapping heading

  updateHeading(90);

  delay(200);
}


// ============================================================
// REVERSE
// ============================================================

void reverseRobot()
{
  Serial.println(
    "Both sides blocked - REVERSING"
  );

  robotStatus =
    "Reversing";

  backward();

  setRawMotorSpeed(
    SPEED_CEMENT
  );

  unsigned long start =
    millis();

  while (
    millis() - start <
    REVERSE_TIME
  )
  {
    // Estimate reverse movement

    float speed =
      getEstimatedSpeedCmS();

    float dt =
      0.01;

    float angle =
      robotHeading *
      PI / 180.0;

    robotX -=
      speed *
      dt *
      cos(angle);

    robotY -=
      speed *
      dt *
      sin(angle);

    robotX =
      constrain(
        robotX,
        0,
        MAP_WIDTH_CM - 1
      );

    robotY =
      constrain(
        robotY,
        0,
        MAP_HEIGHT_CM - 1
      );

    delay(10);
  }

  stopMotors();

  delay(200);
}


// ============================================================
// CHECK OBSTACLE
// ============================================================

void checkObstacle()
{
  if (
    frontDistance > 0 &&
    frontDistance <=
    OBSTACLE_DISTANCE
  )
  {
    obstacleDetected = true;
  }
  else
  {
    obstacleDetected = false;
  }
}


// ============================================================
// HANDLE OBSTACLE
// ============================================================

void handleObstacle()
{
  stopMotors();

  Serial.println();
  Serial.println(
    "***************************************"
  );

  Serial.println(
    "       OBSTACLE DETECTED"
  );

  Serial.println(
    "***************************************"
  );


  bool leftFree =
    (
      leftDistance == -1 ||
      leftDistance >
      SIDE_BLOCK_DISTANCE
    );


  bool rightFree =
    (
      rightDistance == -1 ||
      rightDistance >
      SIDE_BLOCK_DISTANCE
    );


  // Both sides free

  if (
    leftFree &&
    rightFree
  )
  {
    if (
      leftDistance == -1 &&
      rightDistance == -1
    )
    {
      rotateRight90();
    }

    else if (
      leftDistance == -1
    )
    {
      rotateLeft90();
    }

    else if (
      rightDistance == -1
    )
    {
      rotateRight90();
    }

    else if (
      leftDistance >
      rightDistance
    )
    {
      rotateLeft90();
    }

    else
    {
      rotateRight90();
    }

    return;
  }


  // Left free

  if (
    leftFree &&
    !rightFree
  )
  {
    rotateLeft90();

    return;
  }


  // Right free

  if (
    !leftFree &&
    rightFree
  )
  {
    rotateRight90();

    return;
  }


  // Both blocked

  reverseRobot();

  readUltrasonicSensors();


  bool leftAvailable =
    (
      leftDistance == -1 ||
      leftDistance >
      SIDE_BLOCK_DISTANCE
    );


  bool rightAvailable =
    (
      rightDistance == -1 ||
      rightDistance >
      SIDE_BLOCK_DISTANCE
    );


  if (
    leftAvailable &&
    !rightAvailable
  )
  {
    rotateLeft90();
  }

  else if (
    !leftAvailable &&
    rightAvailable
  )
  {
    rotateRight90();
  }

  else if (
    leftAvailable &&
    rightAvailable
  )
  {
    if (
      leftDistance >
      rightDistance
    )
    {
      rotateLeft90();
    }
    else
    {
      rotateRight90();
    }
  }

  else
  {
    rotateRight90();

    rotateRight90();
  }
}


// ============================================================
// UPDATE ROBOT MOVEMENT
// ============================================================

void updateRobotMovement()
{
  // ----------------------------------------------------------
  // EDGE
  // ----------------------------------------------------------

  if (edgeDetected)
  {
    stopMotors();

    robotStatus =
      "EDGE DETECTED";

    Serial.println(
      "!!! EDGE DETECTED !!!"
    );

    backward();

    setRawMotorSpeed(
      SPEED_CEMENT
    );

    delay(400);

    stopMotors();

    delay(200);

    return;
  }


  // ----------------------------------------------------------
  // OBSTACLE
  // ----------------------------------------------------------

  if (obstacleDetected)
  {
    handleObstacle();

    return;
  }


  // ----------------------------------------------------------
  // NORMAL MOVEMENT
  // ----------------------------------------------------------

  int requiredSpeed =
    getFloorSpeed();

  forward();

  setMotorSpeed(
    requiredSpeed
  );

  robotStatus =
    "Moving Forward";
}


// ============================================================
// INITIALIZE MAP
// ============================================================

void initializeMap()
{
  for (
    int y = 0;
    y < GRID_HEIGHT;
    y++
  )
  {
    for (
      int x = 0;
      x < GRID_WIDTH;
      x++
    )
    {
      occupancyGrid[y][x] =
        CELL_UNKNOWN;
    }
  }

  // Mark starting cell free

  int gx =
    (int)(
      robotX /
      CELL_SIZE_CM
    );

  int gy =
    (int)(
      robotY /
      CELL_SIZE_CM
    );

  setGridCell(
    gx,
    gy,
    CELL_FREE
  );
}


// ============================================================
// HTTP STATUS
// ============================================================

void handleStatus()
{
  String json = "{";

  json += "\"robot\":\"KUCHO BOT\",";

  json +=
    "\"status\":\"" +
    robotStatus +
    "\",";

  json +=
    "\"floor\":\"" +
    floorType +
    "\",";

  json +=
    "\"front\":" +
    String(frontDistance) +
    ",";

  json +=
    "\"left\":" +
    String(leftDistance) +
    ",";

  json +=
    "\"right\":" +
    String(rightDistance) +
    ",";

  json +=
    "\"obstacle\":" +
    String(
      obstacleDetected
      ? "true"
      : "false"
    );

  json += "}";

  server.send(
    200,
    "application/json",
    json
  );
}


// ============================================================
// HTTP MAP DATA
// ============================================================

void handleMap()
{
  updateEstimatedPosition();

  String json = "{";

  json +=
    "\"x\":" +
    String(robotX, 1) +
    ",";

  json +=
    "\"y\":" +
    String(robotY, 1) +
    ",";

  json +=
    "\"heading\":" +
    String(robotHeading, 1) +
    ",";

  json +=
    "\"front\":" +
    String(frontDistance) +
    ",";

  json +=
    "\"left\":" +
    String(leftDistance) +
    ",";

  json +=
    "\"right\":" +
    String(rightDistance) +
    ",";

  json +=
    "\"floor\":\"" +
    floorType +
    "\",";

  json +=
    "\"yolo\":\"" +
    yoloObject +
    "\",";

  json +=
    "\"confidence\":" +
    String(yoloConfidence, 2);

  json += "}";

  server.send(
    200,
    "application/json",
    json
  );
}


// ============================================================
// HTTP YOLO RECEIVER
//
// Python sends:
//
// POST /yolo
//
// object=person
// confidence=0.85
// x=20
// y=30
// width=100
// height=200
// ============================================================

void handleYOLO()
{
  if (
    server.hasArg("object")
  )
  {
    yoloObject =
      server.arg(
        "object"
      );
  }

  if (
    server.hasArg("confidence")
  )
  {
    yoloConfidence =
      server.arg(
        "confidence"
      ).toFloat();
  }

  if (
    server.hasArg("x")
  )
  {
    yoloX =
      server.arg(
        "x"
      ).toInt();
  }

  if (
    server.hasArg("y")
  )
  {
    yoloY =
      server.arg(
        "y"
      ).toInt();
  }

  if (
    server.hasArg("width")
  )
  {
    yoloWidth =
      server.arg(
        "width"
      ).toInt();
  }

  if (
    server.hasArg("height")
  )
  {
    yoloHeight =
      server.arg(
        "height"
      ).toInt();
  }


  lastYOLOTime =
    millis();


  Serial.println();
  Serial.println(
    "======================================"
  );

  Serial.println(
    "          YOLO DETECTION"
  );

  Serial.println(
    "======================================"
  );

  Serial.print(
    "Object     : "
  );

  Serial.println(
    yoloObject
  );

  Serial.print(
    "Confidence : "
  );

  Serial.println(
    yoloConfidence
  );

  Serial.print(
    "X          : "
  );

  Serial.println(
    yoloX
  );

  Serial.print(
    "Y          : "
  );

  Serial.println(
    yoloY
  );

  Serial.print(
    "Width      : "
  );

  Serial.println(
    yoloWidth
  );

  Serial.print(
    "Height     : "
  );

  Serial.println(
    yoloHeight
  );


  server.send(
    200,
    "text/plain",
    "YOLO data received"
  );
}


// ============================================================
// HTTP ROOT
// ============================================================

void handleRoot()
{
  String message;

  message +=
    "KUCHO BOT HTTP SERVER\n\n";

  message +=
    "Endpoints:\n";

  message +=
    "/status\n";

  message +=
    "/map\n";

  message +=
    "/yolo\n";

  server.send(
    200,
    "text/plain",
    message
  );
}


// ============================================================
// START HTTP SERVER
// ============================================================

void startHTTPServer()
{
  server.on(
    "/",
    HTTP_GET,
    handleRoot
  );

  server.on(
    "/status",
    HTTP_GET,
    handleStatus
  );

  server.on(
    "/map",
    HTTP_GET,
    handleMap
  );

  server.on(
    "/yolo",
    HTTP_POST,
    handleYOLO
  );

  server.begin();

  Serial.println();
  Serial.println(
    "HTTP server started!"
  );

  Serial.print(
    "Status URL: http://"
  );

  Serial.print(
    WiFi.localIP()
  );

  Serial.println(
    "/status"
  );

  Serial.print(
    "Map URL: http://"
  );

  Serial.print(
    WiFi.localIP()
  );

  Serial.println(
    "/map"
  );

  Serial.print(
    "YOLO URL: http://"
  );

  Serial.print(
    WiFi.localIP()
  );

  Serial.println(
    "/yolo"
  );
}


// ============================================================
// BLYNK DATA
// ============================================================

void sendData()
{
  wifiSignal =
    WiFi.RSSI();

  Blynk.virtualWrite(
    V0,
    robotPower
  );

  Blynk.virtualWrite(
    V1,
    100
  );

  Blynk.virtualWrite(
    V2,
    wifiSignal
  );

  Blynk.virtualWrite(
    V3,
    obstacleDetected
  );

  Blynk.virtualWrite(
    V4,
    floorType
  );

  Blynk.virtualWrite(
    V5,
    robotStatus
  );

  Blynk.virtualWrite(
    V6,
    suctionStatus
  );

  Blynk.virtualWrite(
    V7,
    mappingStatus
  );
}


// ============================================================
// SETUP
// ============================================================

void setup()
{
  Serial.begin(
    115200
  );

  delay(1000);


  Serial.println();
  Serial.println(
    "======================================"
  );

  Serial.println(
    "      KUCHO BOT - MAPPING SYSTEM"
  );

  Serial.println(
    "======================================"
  );


  // ==========================================================
  // ULTRASONIC
  // ==========================================================

  pinMode(
    FRONT_TRIG,
    OUTPUT
  );

  pinMode(
    FRONT_ECHO,
    INPUT
  );

  pinMode(
    LEFT_TRIG,
    OUTPUT
  );

  pinMode(
    LEFT_ECHO,
    INPUT
  );

  pinMode(
    RIGHT_TRIG,
    OUTPUT
  );

  pinMode(
    RIGHT_ECHO,
    INPUT
  );


  Serial.println(
    "Ultrasonic sensors initialized"
  );


  // ==========================================================
  // TCRT
  // ==========================================================

  pinMode(
    TCRT_AO,
    INPUT
  );

  pinMode(
    TCRT_DO,
    INPUT
  );


  Serial.println(
    "TCRT5000 initialized"
  );


  // ==========================================================
  // RELAY
  // ==========================================================

  pinMode(
    RELAY_PIN,
    OUTPUT
  );


  // NC:
  // HIGH = suction ON

  digitalWrite(
    RELAY_PIN,
    HIGH
  );

  suctionStatus = true;


  Serial.println(
    "Relay initialized"
  );

  Serial.println(
    "Suction: ON"
  );


  // ==========================================================
  // MOTOR PINS
  // ==========================================================

  pinMode(
    IN1,
    OUTPUT
  );

  pinMode(
    IN2,
    OUTPUT
  );

  pinMode(
    IN3,
    OUTPUT
  );

  pinMode(
    IN4,
    OUTPUT
  );


  // ==========================================================
  // PWM
  // ==========================================================

  bool leftPWM =
    ledcAttach(
      ENA,
      PWM_FREQ,
      PWM_RESOLUTION
    );

  bool rightPWM =
    ledcAttach(
      ENB,
      PWM_FREQ,
      PWM_RESOLUTION
    );


  if (
    leftPWM &&
    rightPWM
  )
  {
    Serial.println(
      "Motor PWM initialized"
    );
  }
  else
  {
    Serial.println(
      "ERROR: PWM initialization failed!"
    );
  }


  stopMotors();


  // ==========================================================
  // INITIALIZE MAP
  // ==========================================================

  initializeMap();


  Serial.println(
    "Occupancy grid initialized"
  );

  Serial.print(
    "Map size: "
  );

  Serial.print(
    MAP_WIDTH_CM
  );

  Serial.print(
    " x "
  );

  Serial.print(
    MAP_HEIGHT_CM
  );

  Serial.println(
    " cm"
  );

  Serial.print(
    "Cell size: "
  );

  Serial.print(
    CELL_SIZE_CM
  );

  Serial.println(
    " cm"
  );


  // ==========================================================
  // WIFI
  // ==========================================================

  Serial.println();

  Serial.println(
    "Connecting to WiFi..."
  );

  WiFi.mode(
    WIFI_STA
  );

  WiFi.setSleep(
    false
  );

  WiFi.begin(
    ssid,
    pass
  );


  Serial.print(
    "Connecting"
  );


  while (
    WiFi.status() !=
    WL_CONNECTED
  )
  {
    delay(500);

    Serial.print(".");
  }


  Serial.println();

  Serial.println(
    "WiFi connected!"
  );


  Serial.print(
    "IP Address: "
  );

  Serial.println(
    WiFi.localIP()
  );


  // ==========================================================
  // HTTP SERVER
  // ==========================================================

  startHTTPServer();


  // ==========================================================
  // BLYNK
  // ==========================================================

  Serial.println(
    "Connecting to Blynk..."
  );


  Blynk.config(
    BLYNK_AUTH_TOKEN
  );


  if (
    Blynk.connect(10000)
  )
  {
    Serial.println(
      "Blynk connected!"
    );
  }
  else
  {
    Serial.println(
      "Blynk connection FAILED!"
    );
  }


  timer.setInterval(
    2000L,
    sendData
  );


  // ==========================================================
  // STARTUP BOOST
  // ==========================================================

  Serial.println();

  Serial.println(
    "Starting motor startup boost..."
  );


  forward();

  setMotorSpeed(
    SPEED_STARTUP
  );

  delay(500);

  stopMotors();

  delay(300);


  // ==========================================================
  // READY
  // ==========================================================

  suctionON();

  mappingStatus =
    "Mapping";


  Serial.println();

  Serial.println(
    "======================================"
  );

  Serial.println(
    "          SYSTEM READY"
  );

  Serial.println(
    "======================================"
  );

  Serial.println(
    "MPU6050: DISABLED"
  );

  Serial.println(
    "Mapping: ENABLED"
  );

  Serial.println(
    "Obstacle threshold: 5 cm"
  );

  Serial.println(
    "======================================"
  );
}


// ============================================================
// MAIN LOOP
// ============================================================

void loop()
{
  // ==========================================================
  // BLYNK
  // ==========================================================

  Blynk.run();

  timer.run();


  // ==========================================================
  // HTTP
  // ==========================================================

  server.handleClient();


  // ==========================================================
  // SENSOR READINGS
  // ==========================================================

  readUltrasonicSensors();

  readTCRT();


  // ==========================================================
  // OBSTACLE
  // ==========================================================

  checkObstacle();


  // ==========================================================
  // UPDATE MAP
  // ==========================================================

  updateMapping();


  // ==========================================================
  // DISPLAY
  // ==========================================================

  Serial.println();
  Serial.println(
    "======================================"
  );

  Serial.print(
    "Robot X       : "
  );

  Serial.print(
    robotX,
    1
  );

  Serial.println(
    " cm"
  );


  Serial.print(
    "Robot Y       : "
  );

  Serial.print(
    robotY,
    1
  );

  Serial.println(
    " cm"
  );


  Serial.print(
    "Heading       : "
  );

  Serial.print(
    robotHeading,
    1
  );

  Serial.println(
    " deg"
  );


  Serial.print(
    "Front         : "
  );

  Serial.print(
    frontDistance
  );

  Serial.println(
    " cm"
  );


  Serial.print(
    "Left          : "
  );

  Serial.print(
    leftDistance
  );

  Serial.println(
    " cm"
  );


  Serial.print(
    "Right         : "
  );

  Serial.print(
    rightDistance
  );

  Serial.println(
    " cm"
  );


  Serial.print(
    "YOLO Object   : "
  );

  Serial.println(
    yoloObject
  );


  Serial.println(
    "======================================"
  );


  // ==========================================================
  // NAVIGATION
  // ==========================================================

  updateRobotMovement();


  // ==========================================================
  // SUCTION ALWAYS ON
  // ==========================================================

  suctionON();


  delay(300);
}