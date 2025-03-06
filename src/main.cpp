#include "main.h"

// function declarations 
void setup();
void loop();
void toggleLED();
void sendHotTubStatus();

// Define PID I/O Variables we'll be connecting to
double Setpoint, Input, Output;

// Specify the links and initial PID tuning parameters
double Kp = 2, Ki = 5, Kd = 1;
PID temperaturePID(&Input, &Output, &Setpoint, Kp, Ki, Kd, DIRECT);

// Define the pin numbers for the MAX6675
// const int thermocouple1_CS = PB12;
// const int thermocouple_CLK = PB13;
// const int thermocouple_SO = PB14;
// const int thermocouple2_CS = PB15;

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

void setup()
{
  // Initialize the LED pin as an output
  pinMode(LED_Pin, OUTPUT);

  // Initialize SPI2
  SPI.begin();

  // Initialize UART1
  Serial1.begin(9600); // Set the baud rate as needed

  // Initialize UART2
  Serial2.begin(9600); // Set the baud rate as needed

  // Initialize USB serial communication for debugging
  Serial.begin(9600);

  // Wait for MAX6675 to stabilize
  delay(500);

  // Test message
  Serial1.println("Hello from UART1");

  // Reset Main loop timing variables
  currentTime = millis();
  lastStatusUpdate = currentTime;
  lastReadUpdate = lastStatusUpdate;
  }

  void loop()
  {

    static int count = 0;
    currentTime = millis();

    // Check if Status Update time elapsed (default 200ms)
    if ((currentTime - lastStatusUpdate) >= STATUS_UPDATE_TIME)
    {
      // Transmit Hot Tub hardware status
      // transmitDataStructure();

      // signal to user that hot tub data packet sent
      // toggleLED();

      // Reset the status update loop counter
      lastStatusUpdate = millis();
    }

    // Check if thermocouple read time elapsed (default 500ms)
    if ((currentTime - lastReadUpdate) >= READ_TIME_DELAY)
    {
      // Read the temperature from the MAX6675
      hotTub.currentTemp = thermocouple1.readFahrenheit();

      // Print the temperature to the serial monitor
      Serial.print("Temperature: ");
      Serial.print(hotTub.currentTemp);
      Serial.println(" °F");
      Serial.println("\n\n");

      sendHotTubStatus();

      if (Serial1.available() > 0)
      {
        char c = Serial1.read();
        Serial2.write(c);
      }

      // // Test UART communication
      // while (Serial1.available() > 0) {
      //   char c = Serial1.read();
      //   Serial2.write(c);

      // }

      while (Serial2.available() > 0)
      {
        char c = Serial2.read();
        Serial.write(c);
      }

      // Toggle the LED
      toggleLED();

      Serial1.println("count = " + String(count));

      count++;

      // Reset the thermocouple read loop counter
      lastReadUpdate = millis();
    }
  }

  void toggleLED()
  {
    static bool ledState = LOW;      // Keep track of the LED state
    ledState = !ledState;            // Toggle the state
    digitalWrite(LED_Pin, ledState); // Set the LED to the new state
  }

  void sendHotTubStatus()
  {
    // Create a JSON object to store the status data
    // StaticJsonDocument<200> doc;
    JsonDocument doc;

    // Add the temperature data to the JSON object
    // doc["temperature"] = thermocouple1.readFahrenheit();


    doc["temp"] = hotTub.currentTemp;
    doc["sTemp"] = hotTub.setTemp;
    doc["heat"] = hotTub.HeaterOn;
    doc["pLo"] = hotTub.pumpLowSpeedOn;
    doc["pHi"] = hotTub.pumpHighSpeedOn;
  
  

    // Serialize the JSON object to a string
    String output;
    serializeJson(doc, output);

    // Transmit the JSON string over Serial
    Serial.println(output);
  }