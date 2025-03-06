#include <Arduino.h>
#include "globals.h"
#include "ArduinoJson-v6.19.4.h" 
#include "pid.h"
#include <SPI.h>


#define BAUDRATE 57600

// Hot Tub Min and Max Temperature range
#define MAX_ALLOWED_TEMP 45
#define MIN_ALLOWED_TEMP 0
#define HYSTERISIS 3

// Timing Constants
#define START_UP_DELAY 3500
#define READ_TIME_DELAY 500
#define STATUS_UPDATE_TIME 200
