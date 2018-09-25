# Sidus Sigma Smart Drone Project.

Welcome to our fully functional flight control software running on freeRTOS. This safe and robust software is designed for ESP32 based Sidus Sigma Autopilot and optimized for Sidus Sigma Smart Drone. Our repo also includes the ground station software running on Windows environment.

Please visit [Wiki](https://github.com/DevSidus/Sidus_Github/wiki) page for additional information.

## Getting Started
These instructions will get you a copy of the project up and running the code for development and testing purposes.

### Setting-Up the Toolset

* Install [Arduino IDE](https://www.arduino.cc/en/Main/Software)
  ```
  Suggestion: It is better to install Arduino IDE to the folder "D:\Arduino" to follow the next steps easily. 
  ``` 
* Install [Arduino Core for ESP32](https://github.com/espressif/arduino-esp32)
  ```
  Hints:
   - Basic step at this link is clone the repo at "https://github.com/espressif/arduino-esp32" to the folder "D:\Arduino\hardware\espressif\esp32"
   - Install SmartGit for Git flow
   - To change directory at gitbash, use "cd /D/Arduino/hardware/espressif/esp32".
  ```
* Install [Microsoft Visual Studio](https://visualstudio.microsoft.com/)
* Install [Arduino for Visual Studio (Visual Micro)](https://www.visualmicro.com/)
  ```
  Hint: Don't forget to configure Arduino IDE location at Visual Studio. 
  ``` 
* Clone [Mavlink](https://github.com/mavlink/c_library_v2.git) library to the "D:\Arduino\hardware\espressif\esp32\libraries" folder.
* Add [I2CDev](https://github.com/DevSidus/Wiki_Documents/blob/master/Global_Arduino_Libraries/I2Cdev.zip) and [MPU6050](https://github.com/DevSidus/Wiki_Documents/blob/master/Global_Arduino_Libraries/MPU6050.zip) libraries to the "D:\Arduino\hardware\espressif\esp32\libraries" folder.
