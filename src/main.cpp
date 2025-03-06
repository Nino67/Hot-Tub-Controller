#include "main.h"
#include <SPI.h>
#include <max6675.h>

// put function declarations here:
void toggleLED();
float readThermocouple1();
float readThermocouple2();

  // Define PID I/O Variables we'll be connecting to
  double Setpoint, Input, Output;

  // Specify the links and initial PID tuning parameters
  double Kp = 2, Ki = 5, Kd = 1;
  PID temperaturePID(&Input, &Output, &Setpoint, Kp, Ki, Kd, DIRECT);

  // Define the pin numbers for the MAX6675
  const int thermo1_CS = PB12;
  const int thermo_CLK = PB13;
  const int thermo_SO = PB14;
  const int thermo2_CS = PB15;

  // Create an instance of the MAX6675 library
  MAX6675 thermocouple1(thermo_CLK, thermo1_CS, thermo_SO);
  MAX6675 thermocouple2(thermo_CLK, thermo2_CS, thermo_SO);

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
      double temperature = thermocouple1.readFahrenheit();

      // Print the temperature to the serial monitor
      Serial.print("Temperature: ");
      Serial.print(temperature);
      Serial.println(" °F");

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

  