# Sidus Sigma Smart Drone Project.

Welcome to our fully functional flight control software running on freeRTOS. This safe and robust software is designed for ESP32 based Sidus Sigma Autopilot and optimized for Sidus Sigma Smart Drone. Our repo also includes the ground station software running on Windows environment.

Please visit [Wiki](https://github.com/DevSidus/Sidus_Github/wiki) page for additional information.

## Getting Started
These instructions will get you a copy of the project up and running the code for development and testing purposes.

### Setting-up the toolset

* Install [Arduino IDE](https://www.arduino.cc/en/Main/Software)
  ```
  Suggestion: It is better to install Arduino IDE to the folder "C:\Arduino" to follow the next steps easily. 
  ``` 
* Install [Arduino Core for ESP32](https://github.com/espressif/arduino-esp32)
  ```
  Hints:
   - Basic step at this link is clone the repo at "https://github.com/espressif/arduino-esp32" to the folder "C:\Arduino\hardware\espressif\esp32"
   - Install SmartGit or TortoiseGit for Git flow
   - To change directory at Git Bash, use "cd /C/Arduino/hardware/espressif/esp32".
  ```
* Install [Microsoft Visual Studio](https://visualstudio.microsoft.com/)
* Install [Arduino for Visual Studio (Visual Micro)](https://www.visualmicro.com/)
  ```
  Hint: Don't forget to configure Arduino IDE location at Microsoft Visual Studio. 
  ``` 
* Clone [Mavlink](https://github.com/mavlink/c_library_v2.git) library to the "C:\Arduino\hardware\espressif\esp32\libraries" folder.
* Add [I2CDev](https://github.com/DevSidus/Wiki_Documents/blob/master/Global_Arduino_Libraries/I2Cdev.zip) and [MPU6050](https://github.com/DevSidus/Wiki_Documents/blob/master/Global_Arduino_Libraries/MPU6050.zip) libraries to the "C:\Arduino\hardware\espressif\esp32\libraries" folder.

### Build and upload the code
* Open the Sidus_Github_Solution.sln in Microsoft Visual Studio
* The Ground_Station code may not be loaded because of the .Net Framework version mismatch. Install the targeted .NET Framework version (Developer Pack).
* Select the "ESP32 Dev Module" and the COM port.
* Click the "Build and Upload" button.
