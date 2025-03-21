# Hot Tub Controller

## Overview

- This project provides automatic temperature control and auto or manual pump control for a hot tub. It uses an STM32 Blue Pill microcontroller to manage temperature regulation, pump activation, and status communication.

- [Hot Tub Controller Image 1:](images/HotTub-Controller-1.png)
- [Hot Tub Controller Image 2:](images/HotTub-Controller-2.png)

### Description

The Hot Tub Controller utilizes a MAX6675 thermocouple amplifier and K-type thermocouple to monitor the Hot tub temperature. When the heater is enabled and the temperature falls below the setpoint plus the hysteresis value, the controller then checks if the pump is already on and running. If not, it enables the low-speed pump operation before engaging the heater. The system ensures the pump is enabled before heater activation. If the pump was manually enabled by the user the heater deactivation does not deactivate the pump. If pump activation was automatic then pump will turn off at end of heater cycle.

The controller communicates status updates approximately five times a second to a touchscreen display and a Bluetooth module. It allows temperature setting and manual pump control, which when enabled, overrides the automatic pump deactivation during heater cycles.

### Key Features

- **Temperature Monitoring:** Reads temperature using MAX6675 thermocouples.
- **Relay Control:** Controls relays for heater and pump activation.
- **Temperature Regulation:** Implements a basic temperature control algorithm with hysteresis.
- **UART Communication:** Communicates status and receives commands via UART serial ports (Serial, Serial1, Serial2).
- **JSON Messaging:** Handles incoming JSON messages to update hot tub settings.
- **Safety Mechanisms:** Provides temperature range checks to prevent overheating or freezing.
- **Automatic and Manual Pump Control:** Allows for both automatic pump control based on temperature and manual override.

## Hardware Requirements

- **Microcontroller:** STM32 Blue Pill [stm32 bluepill pinout](images/stm32_bluepill_pinout.png) and [stm32f103c8 datasheet](docs/stm32f103c8-1851025.pdf)

- **Thermocouple:** MAX6675 with K-type thermocouple [MAX6675 Module](images/MAX6675-Module-Pinout.png) and [max6675 datasheet](docs/max6675.pdf)

- **Relays:** [4 channel 5v relay module](images/4-channel-5v-relay-module2.png)
  - Relay 1: Heater Control
  - Relay 2: High-Speed Pump Control
  - Relay 3: Low-Speed Pump Control
  - Relay 4: Auxiliary (Specify function)
- **Other Components:**

  - Power Supply [LM2596S Buck Converter](images/LM2596S-Buck-Converter.png)

  - Touchscreen Display [2.4inch TFT LCD Display Module](images/TFT_LCD_Display_Module_and_UNO.png) and [TFT LCD Display Module Pinout](images/TFT_LCD_Display_Module_Pinout.png)

  - Bluetooth Module [HC-06 module](images/HC-06_Bluetooth_Module.png)

### Wiring Diagram

- [Hot Tub Controller Block Wiring Diagram.jpg](images/Hot-Tub-Sch.jpg)

- [Hot Tub Controller Block Wiring Diagram.pdf](docs/Hot-Tub-Sch.pdf)

## Software Requirements

- **Development Environment:** PlatformIO in Visual Studio Code
- **Arduino Libraries:**
  - `SPI` (Included with Arduino framework)
  - `MAX6675` (Adafruit version 1.1.2)
  - `ArduinoJson` (Benoit Blanchon version 6.19.4 )
- **Programming Language:** C++

## Installation and Setup

1.  **Install Visual Studio Code:** Download and install Visual Studio Code from the [official website](https://code.visualstudio.com/).
2.  **Install PlatformIO IDE:** Install the PlatformIO IDE extension in Visual Studio Code.
3.  **Clone the Project:** Clone the project from the repository (if applicable).
    ```git
    git clone https://github.com/Nino67/Hot-Tub-Controller.git
    cd HotTub-2025-05-03
    ```
4.  **Install Arduino Libraries:** PlatformIO will automatically install the required libraries defined in [`platformio.ini`](platformio.ini).
5.  **Configure PlatformIO:**
    - Set the correct board in `platformio.ini`: `board = genericSTM32F103C8`
    - Set the upload port in `platformio.ini` (if necessary): `upload_port = COMx` or `upload_port = /dev/ttyUSB0`
6.  **Upload the Code:** Build and upload the code to the STM32 Blue Pill board using PlatformIO.

## Usage

1.  **Connect the Hardware:** Connect all the hardware components according to the [wiring diagram.](docs/Hot-Tub-Sch.pdf)
2.  **Power On:** Power on the hot tub controller.
3.  **Interact with the Touchscreen Display:** Use the touchscreen display to:
    - Set the target temperature.
    - Enable or disable the heater.
    - Control the pumps (automatic or manual mode).
    - Monitor the status of the hot tub (temperature, pump status, heater status).

### Serial Communication Protocol

The Hot Tub Controller uses JSON messages for communication over the serial ports.

- **BAUDRATE:** `57600 bps` Default settable in [include/globals.h](include/globals.h)

- **Status Updates:** The controller sends status updates in the following JSON format:

  ```json
  {
    "temp": "XX.X",
    "sTemp": XX,
    "heat": true/false,
    "pLo": true/false,
    "pHi": true/false
  }
  ```

  - `temp`: Current temperature in Celsius (formatted to one decimal place).
  - `sTemp`: Set temperature.
  - `heat`: Heater status (true = on, false = off).
  - `pLo`: Low-speed pump status (true = on, false = off).
  - `pHi`: High-speed pump status (true = on, false = off).

- **Commands:** The controller receives commands in the same JSON format. For example, to set the temperature to 100 and enable the heater:

  ```json
  {
    "sTemp": 100,
    "heat": true
  }
  ```

## Project Structure

```
HotTub-2025-05-03/
├── .gitignore             # Specifies intentionally untracked files that Git should ignore
├── LICENSE                # License file (e.g., MIT License)
├── README.md              # Project documentation
├── platformio.ini         # PlatformIO project configuration file
├── src/                   # Source code directory
│   └── main.cpp           # Main application logic
├── include/               # Header files directory
│   └── globals.h          # Global constants, pin assignments, struct definitions
├── lib/                   # (Optional) Directory for custom libraries
├── images/                # Directory for images used in documentation
│   ├── 4-channel-5v-relay-module2.png
│   ├── HC-06_Bluetooth_Module.png
│   ├── Hot-Tub-Sch.jpg    # Block Wiring Diagram
│   ├── LM2596S-Buck-Converter.png
│   ├── MAX6675-Module-Pinout.png
│   ├── stm32_bluepill_pinout.png
│   └── TFT_LCD_Display_Module_and_UNO.png
│   └── TFT_LCD_Display_Module_Pinout.png
├── docs/                  # Directory for datasheets and other documents
│   ├── max6675.pdf        # MAX6675 Datasheet
│   └── stm32f103c8-1851025.pdf  # STM32F103C8 Datasheet
└── .vscode/              # VS Code specific settings (optional)
    └── settings.json      # VS Code settings for the project (e.g., formatting)
```

## Code Structure

- [`main.cpp`](src/main.cpp): Contains the main application logic, including:
  - Initialization of hardware components.
  - Main loop for reading temperature, controlling relays, and communicating status.
  - Functions for handling serial communication and JSON messages.
- [`include/globals.h`](include/globals.h): Defines global constants, pin assignments, and struct definitions.
- [`platformio.ini`](platformio.ini): Contains the PlatformIO project configuration, including board settings, library dependencies, and build flags.

## TODO:

- **PID Based Temperature Control:** Solid State Relays to be added.

## License

Copyright 2025 Nino67\<gricca1967@gmail.com\>

This repository is released under the MIT License, see [LICENSE](./LICENSE).  
Unless attributed otherwise, everything in this repository is under the MIT License.

## Acknowledgments

- ArduinoJson library: [https://arduinojson.org/](https://arduinojson.org/)
- MAX6675 library: [https://github.com/adafruit/MAX6675-library](https://github.com/adafruit/MAX6675-library)

## Contact Information

- Gaetano (Nino) Ricca: [gricca1967@gmail.com](mailto:gricca1967@gmail.com)

## Addendum

- Yes I was forced to build this, blame `Roland`.
- Yes `Roland` does not feed his slaves, or give lunch breaks.
- Mark my words, one day I will get that which is owed, the other half of the sandwhich... heheh
