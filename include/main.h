#include "globals.h"
#include <SPI.h>
#include <max6675.h>
#include <ArduinoJson.h>
#include <pid.h>

// #include <SPI.h>
// #include <ArdunoJson.h>


#define BAUDRATE 57600

// Hot Tub Min and Max Temperature range
#define MAX_ALLOWED_TEMP 45
#define MIN_ALLOWED_TEMP 0
#define HYSTERISIS 3

// Timing Constants
#define START_UP_DELAY 3500
#define READ_TIME_DELAY 500
#define STATUS_UPDATE_TIME 200

// Buffer Size for UART
#define MAX_BUFFER_SIZE 200

// Hot Tub Structure Definition
static struct HotTubStruct {
  // Hot Tub Variables
  float currentTemp = 20;
  float setTemp = 20;
  float airTemp = 0;
  bool HeaterOn = false;
  bool pumpLowSpeedOn = false;
  bool pumpHighSpeedOn = false;
  // Variable to signify temperature went below setpoint
  bool belowSetpoint = true;
  // Variable to signify temperature range window below setpoint
  int hysterisis = HYSTERISIS;
} hotTub;
