# Sidus Sigma Smart Drone Project.

Welcome to our repo that includes both the flight control software running on Sidus Sigma Autopilot and ground station running on Windows environment. Please visit [Wiki](https://github.com/DevSidus/Sidus_Github/wiki) page for additional information.

## Getting Started
These instructions will get you a copy of the project up and running on Sidus Sigma Autopilot for development and testing purposes.

### Setting-Up the Toolset

* Install [Arduino IDE](https://www.arduino.cc/en/Main/Software)
  ```
  Suggestion: It is better to install Arduino IDE to the folder "D:\Arduino" to follow the next steps easily. 
  ``` 
* Install [Arduino Core for ESP32](https://github.com/espressif/arduino-esp32)
  ```
  Hints:
   - Basic step at this link is clone the repo at "https://github.com/espressif/arduino-esp32" to the folder "D:\Arduino\hardware\espressif\esp32"
   - To change directory at gitbash, use "cd /D/Arduino/hardware/espressif/esp32".
  ```
* Install [Microsoft Visual Studio](https://visualstudio.microsoft.com/)
* Install [Arduino for Visual Studio (Visual Micro)](https://www.visualmicro.com/)
  ```
  Hint: Don't forget to configure Arduino IDE location at Visual Studio. 
  ``` 
* Clone [Mavlink](https://github.com/mavlink/c_library_v2.git) library to the "D:\Arduino\hardware\espressif\esp32\libraries" folder.
