#include "globals.h"
#include <SPI.h>
#include <max6675.h>
#include <ArduinoJson.h>
#include <pid.h>

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

  // Serial USB Variables
  uint8_t SerialTxMsg[MAX_BUFFER_SIZE];
  uint8_t SerialRxMsg[MAX_BUFFER_SIZE];
  int SerialTxMsgLength = 0;
  int SerialRxMsgLength = 0;
  bool SerialTxMsgReady = false;
  bool SerialRxMsgReady = false;
  
    // Serial1 Variables
  uint8_t Serial1TxMsg[MAX_BUFFER_SIZE];
  uint8_t Serial1RxMsg[MAX_BUFFER_SIZE];
  int Serial1TxMsgLength = 0;
  int Serial1RxMsgLength = 0;
  bool Serial1TxMsgReady = false;
  bool Serial1RxMsgReady = false;
  
  // Serial2 Variables
  uint8_t Serial2TxMsg[MAX_BUFFER_SIZE];
  uint8_t Serial2RxMsg[MAX_BUFFER_SIZE];
  int Serial2TxMsgLength = 0;
  int Serial2RxMsgLength = 0;
  bool Serial2TxMsgReady = false;
  bool Serial2RxMsgReady = false;

  // Variable to signify temperature went below setpoint
  bool belowSetpoint = true;
  // Variable to signify temperature range window below setpoint
  int hysterisis = HYSTERISIS;
} hotTub;
