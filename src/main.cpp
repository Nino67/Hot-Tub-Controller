/**************************************************************************/
/**************************************************************************/
/**
 * @file main.cpp
 * @brief This file contains the main application logic 
 *        for the Hot Tub Controller.
 *
 * @author Gaetano (Nino) Ricca. 
 *                 email gricca1967@gmail.com  
 * @version 1.1.0
 * @date 2021-09-30
 *  
 * The Hot Tub Controller is designed to manage and monitor various 
 * aspects of a hot tub, including temperature regulation, pump control, 
 * and status communication.
 *
 * Key functionalities include:
 *   - Reading temperature from MAX6675 thermocouples.
 *   - Controlling relays for heater and pump activation.
 *   - Implementing a basic temperature control algorithm.
 *   - Communicating status and receiving commands via UART serial ports
 *      (Serial, Serial1, Serial2).
 *   - Handling incoming JSON messages to update hot tub settings.
 *   - Providing safety mechanisms such as temperature range checks.
 *
 * The code is structured with clear function declarations and 
 * comments to facilitate understanding and maintenance. It utilizes 
 * the Arduino framework for STM32 microcontrollers.
 */
/**************************************************************************/
/**************************************************************************/


/**************************************************************************/
/**** Include Files *******************************************************/
/**************************************************************************/
#include "main.h"
/**************************************************************************/


/**************************************************************************/
/**** Function Declarations ***********************************************/
/**************************************************************************/
void setup();
void loop();
void toggleLED(void);
void sendHotTubStatus(void);
void updateHotTubStatus(char*);

void updatePumpStatus(HotTubStruct&);
bool checkSerialForData(HotTubStruct&);
bool checkSerial1ForData(HotTubStruct&);
bool checkSerial2ForData(HotTubStruct&);
bool checkIfHotTubUpdateReceived(HotTubStruct&);
void runBasicTemperatureControl(HotTubStruct&);

void heaterRelayOn(void);           // Relay 1
void heaterRelayOff(void);				  // Relay 1
bool heaterRelayStatus(void);			  // Relay 1
void pumpHighOn(void);					    // Relay 2
void pumpHighOff(void);					    // Relay 2
bool pumpHighStatus(void);				  // Relay 2
void pumpLowOn(void);					      // Relay 3
void pumpLowOff(void);					    // Relay 3
bool pumpLowStatus(void);				    // Relay 3
void Relay4on(void);						    // Relay 4
void Relay4off(void);					      // Relay 4
bool Relay4status (void);				    // Relay 4
void RelaysAllOff(void);
/**************************************************************************/


/**************************************************************************/
/**** Global and Constant Declarations ************************************/
/**************************************************************************/
// Define PID I/O Variables we'll be connecting to
double Setpoint, Input, Output;

// Specify the links and initial PID tuning parameters
double Kp = 2, Ki = 5, Kd = 1;
PID temperaturePID(&Input, &Output, &Setpoint, Kp, Ki, Kd, DIRECT);

// Create an instance of the MAX6675 library
MAX6675 thermocouple1(THERMO_CLK_Pin, THERMO_CS_1_Pin, THERMO_SO_Pin);
MAX6675 thermocouple2(THERMO_CLK_Pin, THERMO_CS_2_Pin, THERMO_SO_Pin);

// Define Serial1 manually (Serial2 is defined in the Arduino framework)
HardwareSerial Serial1(PA10, PA9); // RX, TX for USART1
// HardwareSerial Serial2(PA3, PA2); // RX, TX for USART2

// Initialize Main loop timing variables
static uint32_t currentTime = 0;
static uint32_t lastStatusUpdate = 0;
static uint32_t lastReadUpdate = 0;
/**************************************************************************/


/***************************************************************************/
/**
 * @brief   Function to setup the initial state of the system
 *
 */
void setup()
{
  // Initialize the LED pin as an output
  pinMode(LED_Pin, OUTPUT);

  // Initialize the relay pins as outputs
  pinMode(RELAY_1_Pin, OUTPUT);
  pinMode(RELAY_2_Pin, OUTPUT);
  pinMode(RELAY_3_Pin, OUTPUT);
  pinMode(RELAY_4_Pin, OUTPUT);

  // Set the relay pins to LOW (OFF)
  digitalWrite(RELAY_1_Pin, LOW);
  digitalWrite(RELAY_2_Pin, LOW);
  digitalWrite(RELAY_3_Pin, LOW);
  digitalWrite(RELAY_4_Pin, LOW);


  // Initialize SPI2
  SPI.begin();

  // Initialize UART1
  Serial1.begin(BAUDRATE); // Set the baud rate as needed
  // Initialize UART2
  Serial2.begin(BAUDRATE); // Set the baud rate as needed
  // Initialize USB serial communication for debugging
  Serial.begin(BAUDRATE);

  // delay a second for everything to stabalize
  delay(1000);

} // end of setup() function
/***************************************************************************/


/***************************************************************************/
/**
 * @brief   Main loop function to run the hot tub controller
 *
 */
void loop()
{
  // Reset Main loop timing variables
  currentTime = millis();
  lastStatusUpdate = currentTime;
  lastReadUpdate = lastStatusUpdate;

  // Main endless loop
  while (true) {
    // Get the current time
    currentTime = millis();

    // Check if Status Update time elapsed (default 200ms)
    if ((currentTime - lastStatusUpdate) >= STATUS_UPDATE_TIME) {
      // Check if a update was received before hardware status sent
      checkIfHotTubUpdateReceived(hotTub);
      // Transmit Hot Tub hardware status
      sendHotTubStatus();
      // Update the Hardware status 
      updatePumpStatus(hotTub);
      // Run the basic temperature control algorithm
      runBasicTemperatureControl(hotTub);
      // signal to user that hot tub data packet sent
      toggleLED();
      // Reset the status update loop counter
      lastStatusUpdate = millis();
    } // end of if((currentTime - lastStatusUpdate) >= STATUS_UPDATE_TIME)

    // Check if thermocouple read time elapsed (default 500ms)
    if ((currentTime - lastReadUpdate) >= READ_TIME_DELAY) {
      // Read the temperature from the MAX6675
      hotTub.currentTemp = thermocouple1.readFahrenheit();
      // Reset the thermocouple read loop counter
      lastReadUpdate = millis();
    } // end of if((currentTime - lastReadUpdate) >= READ_TIME_DELAY)
  } // end of while endless loop
} // end of loop() function
/***************************************************************************/


/***************************************************************************/
/**
 * @brief Function to check if a message was received from the hot tub 
 * and update the status accordingly
 * 
 * @param hot_tub   The hot tub struct to store the incoming data
 * @return true   If a message was received
 * @return false  If a message was not received
 */
bool checkIfHotTubUpdateReceived(HotTubStruct &hot_tub)
{
  bool hotTubUpdateReceived = false;

  // Check for incoming data on the USB_UART
  if (checkSerialForData(hot_tub)) {
    updateHotTubStatus((char *)hot_tub.SerialRxMsg);
    // sendHotTubStatus();
    hot_tub.SerialRxMsgLength = 0;
    hot_tub.SerialRxMsgReady = false;
    hotTubUpdateReceived = true;
    return hotTubUpdateReceived;
  } // end of if(checkSerialForData(hot_tub))

  // Check for incoming data on Serial1
  if (checkSerial1ForData(hot_tub)) {
    updateHotTubStatus((char *)hot_tub.Serial1RxMsg);
    // sendHotTubStatus();
    hot_tub.Serial1RxMsgLength = 0;
    hot_tub.Serial1RxMsgReady = false;
    hotTubUpdateReceived = true;
    return hotTubUpdateReceived;
  } // end of if(checkSerial1ForData(hot_tub))

  // Check for incoming data on Serial2
  if (checkSerial2ForData(hot_tub)) {
    updateHotTubStatus((char *)hot_tub.Serial2RxMsg);
    // sendHotTubStatus();
    hot_tub.Serial2RxMsgLength = 0;
    hot_tub.Serial2RxMsgReady = false;
    hotTubUpdateReceived = true;
    return hotTubUpdateReceived;
  } // end of if(checkSerial2ForData(hot_tub))
  return hotTubUpdateReceived;
} // end of checkIfHotTubUpdateReceived() function
/***************************************************************************/


/***************************************************************************/
/**
 * @brief  Function to toggle the LED on the board
 *
 */
void toggleLED()
{
  static bool ledState = LOW;      // Keep track of the LED state
  ledState = !ledState;            // Toggle the state
  digitalWrite(LED_Pin, ledState); // Set the LED to the new state
} // end of toggleLED() function
/***************************************************************************/


/***************************************************************************/
/**
 * @brief Function to send the current hot tub status over UARTS
 *
 */
void sendHotTubStatus()
{
  // Create a JSON object to store the status data
  JsonDocument doc;

  // Buffer to hold the formatted temperature string
  char tempStr[10];

  // Convert float to string with 1 decimal place
  dtostrf(hotTub.currentTemp, 4, 1, tempStr);

  doc["temp"] = tempStr;
  doc["sTemp"] = hotTub.setTemp;
  doc["heat"] = hotTub.HeaterOn;
  doc["pLo"] = hotTub.pumpLowSpeedOn;
  doc["pHi"] = hotTub.pumpHighSpeedOn;

  // Serialize the JSON object to a string
  String output;
  serializeJson(doc, output);

  // Transmit the JSON string over Serial
  Serial.println(output);
  Serial1.println(output);
  Serial2.println(output);
} // end of sendHotTubStatus() function
/***************************************************************************/


/***************************************************************************/
/**
 * @brief  Function to update the hot tub status based on the incoming message
 *
 * @param msg The incoming Json message used to update the hot tub status
 */
void updateHotTubStatus(char *msg)
{
  JsonDocument doc;
  deserializeJson(doc, msg);

  // Check if the temperature is a float
  if (doc["temp"].is<float>()) {
    // Convert the string representation of the float to a float
    float tempValue = doc["temp"].as<float>();
    // Buffer to hold the formatted temperature string
    char tempStr[10];
    // copy the recieved variable to the buffer
    strcpy(tempStr, doc["temp"]);
    // Convert float to string with 1 decimal place
    dtostrf(tempValue, 4, 1, tempStr);
    // Update the hotTub struct with the formatted float value
    hotTub.currentTemp = atof(tempStr);
  }

  // Check if the set temperature is an integer
  if (doc["sTemp"].is<int>()) {
    // Update the hotTub struct with the set temperature
    hotTub.setTemp = doc["sTemp"];
  }

  // Check if the heater status is a boolean
  if (doc["heat"].is<bool>()) {
    // Update the hotTub struct with the heater status
    hotTub.HeaterOn = doc["heat"];
  }

  // Check if the low speed pump status is a boolean
  if (doc["pLo"].is<bool>()) {
    // Update the hotTub struct with the low speed pump status
    hotTub.pumpLowSpeedOn = doc["pLo"];
  }

  // Check if the high speed pump status is a boolean
  if (doc["pHi"].is<bool>()) {
    // Update the hotTub struct with the high speed pump status
    hotTub.pumpHighSpeedOn = doc["pHi"];
  }
} // end of updateHotTubStatus() function
/***************************************************************************/

/**
 * @brief  Function to update the pump status based on the current Hot Tub struct
 * 
 * @param hot_tub   The hot tub struct data to update pump status 
 */
void updatePumpStatus(HotTubStruct &hot_tub) {
  // Check to see if the heater status has changed
  if (hot_tub.pumpLowSpeedOn != pumpLowStatus()) {
    // If the pumpLowSpeedOn is true and the low speed pump relay is off
    if (hot_tub.pumpLowSpeedOn) {
      // Turn off the high speed pump
      pumpHighOff();
      // Turn on the low speed pump
      pumpLowOn();
    } else {
      // Turn off the low speed pump
      pumpLowOff();
    }
  } // end of if(hot_tub.pumpLowSpeedOn != pumpLowStatus())
  
  // Check to see if the high speed pump status has changed
  if (hot_tub.pumpHighSpeedOn != pumpHighStatus()) {
    // If the pumpHighSpeedOn is true and the high speed pump relay is off
    if (hot_tub.pumpHighSpeedOn) {
      // Turn off the low speed pump
      pumpLowOff();
      // Turn on the high speed pump  
      pumpHighOn();
    } else {
      // Turn off the high speed pump
      pumpHighOff();
    } // end of if(hot_tub.pumpHighSpeedOn != pumpHighStatus())
  } // end of if(hot_tub.pumpHighSpeedOn != pumpHighStatus())
} // end of updatePumpStatus() function
/***************************************************************************/


/***************************************************************************/
/**
 * @brief runBasicTemperatureControl()
 * @param hot_Tub   The hot tub struct to store the incoming data
 * @retval None
 */
void runBasicTemperatureControl(HotTubStruct &hot_tub)
{
  // If temperature reading is out of range set heater off and return (safety check)
  if((hot_tub.currentTemp > MIN_ALLOWED_TEMP) && (hot_tub.currentTemp < MAX_ALLOWED_TEMP))
  {
    
    // Check if the user requested the heater on
    if (hot_tub.HeaterOn) 
    {
      // Check to see if temperature is above the user setpoint
      if(hot_tub.currentTemp > hot_tub.setTemp)
      {
        // Set belowSetpoint flag to false to indicate above setpoint
        hot_tub.belowSetpoint = false;
        // Turn off the heater relay
        heaterRelayOff();
        // Check if pump is under user control if not turn off the low speed pump
        if (!(hot_tub.pumpHighSpeedOn || hot_tub.pumpLowSpeedOn))
        {
          // Turn off the low speed pump under heater control
          pumpLowOff();
        } // end of if(!(hot_tub.pumpHighSpeedOn || hot_tub.pumpLowSpeedOn))
      } // end of if(hot_tub.currentTemp > hot_tub.setTemp)
      
      // Check to see if temperature is below (setpoint - hysterisis) 
      // if yes set flag for heater turn on
      if(hot_tub.currentTemp < (hot_tub.setTemp - hot_tub.hysterisis))
      {
        // Set belowSetpoint flag to true
        hotTub.belowSetpoint = true;
      } // end of if(hot_tub.currentTemp < (hot_tub.setTemp - hot_tub.hysterisis))

      // Check to see if belowSetpoint is true and temperature is below setpoint - hysterisis
      if(hot_tub.belowSetpoint && (hot_tub.currentTemp < (hot_tub.setTemp - hot_tub.hysterisis)))
      {
        if(hot_tub.pumpHighSpeedOn || hot_tub.pumpLowSpeedOn) 
        {
          // Check if the high speed pump is on
          if(hot_tub.pumpHighSpeedOn)
          {
            // Turn on the high speed pump
            pumpHighOn();
          }
          else
          {
            // Turn on the low speed pump
            pumpLowOn();
          } // end of if(hot_tub.pumpHighSpeedOn)
          // Turn on the heater relay
          heaterRelayOn();
        }
        else 
        {
          // Turn on the low speed pump
          pumpLowOn();
          // Turn on the heater relay
          heaterRelayOn();
        } // end of if(hot_tub.pumpHighSpeedOn || hot_tub.pumpLowSpeedOn)
      } // end of if(hot_tub.belowSetpoint && (hot_tub.currentTemp < (hot_tub.setTemp - hot_tub.hysterisis)))
    } // end of if (hot_tub.HeaterOn) 
  }
  else
  {
	  // Out of range temperature shut down heater and return
	  hot_tub.HeaterOn = false;
	  heaterRelayOff();
  }
} // end of runBasicTemperatureControl() function
/***************************************************************************/


/***************************************************************************/
/**
 * @brief Function to check for incoming serial data on USB_UART
 *
 * @param hot_tub  The hot tub struct to store the incoming data
 * @return true  If a message is ready
 * @return false  If a message is not ready
 */
bool checkSerialForData(HotTubStruct &hot_tub)
{
  while (Serial.available() > 0)
  {
    char c = Serial.read();
    if (hot_tub.SerialRxMsgLength < MAX_BUFFER_SIZE)
    {
      hot_tub.SerialRxMsg[hot_tub.SerialRxMsgLength++] = c;
    }
    else
    {
      // Handle overflow, reset the buffer, flush the uart and reset msg flag
      Serial.flush();
      hot_tub.SerialRxMsgLength = 0;
      hot_tub.SerialRxMsgReady = false;
    }
    // Check for the end of the message (newline character)
    if (c == '\n')
    {
      hot_tub.SerialRxMsgReady = true;
      break;
    }
  } // end of while loop
  return hot_tub.SerialRxMsgReady;
} // end of checkSerialForData() function
/***************************************************************************/


/***************************************************************************/
/**
 * @brief   Function to check for incoming serial data on Serial1
 * 
 * @param hot_tub   The hot tub struct to store the incoming data
 * @return true   If a message is ready 
 * @return false  If a message is not ready
 */
bool checkSerial1ForData(HotTubStruct &hot_tub)
{
  while (Serial1.available() > 0)
  {
    char c = Serial1.read();
    if (hot_tub.Serial1RxMsgLength < MAX_BUFFER_SIZE)
    {
      hot_tub.Serial1RxMsg[hot_tub.Serial1RxMsgLength++] = c;
    }
    else
    {
      // Handle overflow, reset the buffer, flush the uart and reset msg flag
      Serial1.flush();
      hot_tub.Serial1RxMsgLength = 0;
      hot_tub.Serial1RxMsgReady = false;
    }
    // Check for the end of the message (newline character)
    if (c == '\n')
    {
      hot_tub.Serial1RxMsgReady = true;
      break;
    }
  } // end of while loop
  return hot_tub.Serial1RxMsgReady;
} // end of checkSerial1ForData() function
/***************************************************************************/


/***************************************************************************/
/**
 * @brief  Function to check for incoming serial data on Serial2
 *
 * @param hot_tub The hot tub struct to store the incoming data
 * @return true If a message is ready
 * @return false  If a message is not ready
 */
bool checkSerial2ForData(HotTubStruct &hot_tub)
{
  while (Serial2.available() > 0)
  {
    char c = Serial2.read();
    if (hot_tub.Serial2RxMsgLength < MAX_BUFFER_SIZE)
    {
      hot_tub.Serial2RxMsg[hot_tub.Serial2RxMsgLength++] = c;
    }
    else
    {
      // Handle overflow, reset the buffer, flush the uart and reset msg flag
      Serial2.flush();
      hot_tub.Serial2RxMsgLength = 0;
      hot_tub.Serial2RxMsgReady = false;
    }
    // Check for the end of the message (newline character)
    if (c == '\n')
    {
      hot_tub.Serial2RxMsgReady = true;
      break;
    }
  } // end of while loop
  return hot_tub.Serial2RxMsgReady;
} // end of checkSerial2ForData() function
/***************************************************************************/


/**
 * @brief   Function to turn on the heater relay
 * 
 */
void heaterRelayOn(void) {
  digitalWrite(RELAY_1_Pin, HIGH);
} // end of heaterRelayOn() function
/**************************************************************************/


/**************************************************************************/
/**
 * @brief   Function to turn off the heater relay
 * 
 */
void heaterRelayOff(void) {
  digitalWrite(RELAY_1_Pin, LOW);
}  
/**************************************************************************/


/**************************************************************************/
/**
 * @brief  Function to check the status of the heater relay
 * 
 * @return true  If the relay is on  
 * @return false If the relay is off
 */
bool heaterRelayStatus(void) {
  return digitalRead(RELAY_1_Pin) == HIGH;
} // end of heaterRelayStatus() function
/**************************************************************************/


/**************************************************************************/
/**
 * @brief   Function to turn on the high speed pump relay
 * 
 */
void pumpHighOn(void) {
  digitalWrite(RELAY_2_Pin, HIGH);
} // end of pumpHighOn() function
/**************************************************************************/


/**************************************************************************/
/**
 * @brief   Function to turn off the high speed pump relay
 * 
 */
void pumpHighOff(void) {
  digitalWrite(RELAY_2_Pin, LOW);
} // end of pumpHighOff() function
/**************************************************************************/


/**************************************************************************/
/**
 * @brief   Function to check the status of the high speed pump relay
 * 
 * @return true   If the relay is on
 * @return false  If the relay is off
 */
bool pumpHighStatus(void) {
  return digitalRead(RELAY_2_Pin) == HIGH;
} // end of pumpHighStatus() function  
/**************************************************************************/


/**************************************************************************/
/**
 * @brief   Function to turn on the low speed pump relay
 * 
 */
void pumpLowOn(void) {
  digitalWrite(RELAY_3_Pin, HIGH);
} // end of pumpLowOn() function
/**************************************************************************/


/**************************************************************************/
/**
 * @brief   Function to turn off the low speed pump relay
 * 
 */
void pumpLowOff(void) {
  digitalWrite(RELAY_3_Pin, LOW);
} // end of pumpLowOff() function
/**************************************************************************/


/**************************************************************************/
/**
 * @brief   Function to check the status of the low speed pump relay
 * 
 * @return true   If the relay is on
 * @return false  If the relay is off
 */
bool pumpLowStatus(void) {
  return digitalRead(RELAY_3_Pin) == HIGH;
} // end of pumpLowStatus() function
/**************************************************************************/


/**************************************************************************/
/**
 * @brief   Function to turn on the relay 4
 * 
 */
void Relay4on(void) {
  digitalWrite(RELAY_4_Pin, HIGH);
} // end of Relay4on() function
/**************************************************************************/


/**************************************************************************/
/**
 * @brief   Function to turn off the relay 4
 * 
 */
void Relay4off(void) {
  digitalWrite(RELAY_4_Pin, LOW);
} // end of Relay4off() function
/**************************************************************************/


/**************************************************************************/
/**
 * @brief   Function to check the status of relay 4
 * 
 * @return true   If the relay is on
 * @return false  If the relay is off
 */
bool Relay4status (void) {
  return digitalRead(RELAY_4_Pin) == HIGH;
} // end of Relay4status() function
/**************************************************************************/


/**************************************************************************/
/**
 * @brief   Function to turn off all the relays
 * 
 */
void RelaysAllOff(void) {
  digitalWrite(RELAY_1_Pin, LOW);
  digitalWrite(RELAY_2_Pin, LOW);
  digitalWrite(RELAY_3_Pin, LOW);
  digitalWrite(RELAY_4_Pin, LOW);
} // end of RelaysAllOff() function
/**************************************************************************/



