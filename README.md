# Accessible-Robot-Controls
A demo video of this project operating can be found here: https://youtu.be/NfFuXzByAeY

A method of controlling the Hiwonder MaxArm with both an Xbox Adaptive Controller and UDP head tracking and an ESP32 microcontroller.

This code was written by Kyle Patterson for Spring 2023 CE-490: Senior CE Design,
taught by Dr. Girma Tewolde at Kettering University. 

This code allows control of the Max Arm robot using both an Xbox One controller
(specifically the Xbox Adaptive Controller) 
and head-tracking software. Bluepad32 (credited below) is the project this code
is based off, which handles Bluetooth communication between the controller 
and the ESP32. Functions are then called to read the controller buttons. 

The face-tracking project used is called Opentrack. We used the Freelook
iOS app that implements Opentrack in our testing, however any implementation
that uses Opentrack to send face-tracking information over UDP should work. 6 
little endian doubles, each 8 bytes, are contained in the UDP packets.
The values are sent in IEEE 754 format.
The order is X, Y, Z, YAW, PITCH, ROLL. Every other packet is read
in our implementation. The two devices must be connected to the same LAN
and the SSID and password must be hard-coded in. 

As stated on the Gitlab for Bluepad32, the ESP IDF v4.4 is used
to build and flash the ESP32. The Arduino IDE is NOT SUPPORTED. This project
has all the code necessary, so no need to clone again from Gitlab.  

idf.py build  - builds the code. Make sure to cd into the correct directory

idf.py flash monitor - flash code and send output to idf window

idf.py menuconfig - change settings

Ensure this list of requirements are met in order to guarentee this will work:

- Xbox controller updated to firmware 5.15 (this can be done on Windows using the 
Xbox accessories app over usb)

- Disable the task watchdog timer in the ESP32. run the menuconfig, then
go to Component config -> ESP System Settings -> uncheck Initialize
task watchdog timer on startup

- Enable large projects on the ESP. run the menuconfig, then go to Partition
Table, Partition table (first option), Single Factory App (large), no OTA


Functionality:
The ESP will attempt to connect to the hard-coded WiFi network, but will 
ignore input over UDP until a controller is connected and Up on the 
directional pad is pressed. It will then ignore controller input and 
only accept UDP input. In its current configuration, the ESP must be reset in 
order to accept controller input again. 

In the event that the controller refuses to connect, try resetting the ESP 
or reflashing the code. 



Bluepad32 project: https://github.com/opentrack/opentrack

Opentrack project: https://gitlab.com/ricardoquesada/esp-idf-arduino-bluepad32-template

MaxArm product page: https://www.hiwonder.com/products/maxarm?variant=40008714092631

MaxArm reference: https://drive.google.com/drive/folders/1wi-O2ia-izYWM979iByIws6S4LRe7LKI?usp=sharing



http://retro.moe/unijoysticle2



Copyright 2021 Ricardo Quesada

Licensed under the Apache License, Version 2.0 (the "License");
you may not use this file except in compliance with the License.
You may obtain a copy of the License at

    http://www.apache.org/licenses/LICENSE-2.0

Unless required by applicable law or agreed to in writing, software
distributed under the License is distributed on an "AS IS" BASIS,
WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
See the License for the specific language governing permissions and
limitations under the License.
****************************************************************************/
