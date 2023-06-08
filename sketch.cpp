/****************************************************************************
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

#include "sdkconfig.h"
#ifndef CONFIG_BLUEPAD32_PLATFORM_ARDUINO
#error "Must only be compiled when using Bluepad32 Arduino platform"
#endif  // !CONFIG_BLUEPAD32_PLATFORM_ARDUINO

#include <Arduino.h>
#include <Bluepad32.h>
#include "ESP32PWMServo.h"
#include <WiFi.h>
#include "esp_wifi.h"
#include "AsyncUDP.h"
#include <String.h>

#include "Servo.h"
#include "SuctionNozzle.h"

#include "LobotSerialServoControl.h"
#include <Stream.h>
#define SERVO_SERIAL_RX   35
#define SERVO_SERIAL_TX   12
const int receiveEnablePin = 13;
const int transmitEnablePin = 14;

AsyncUDP udp;

LobotSerialServoControl::LobotSerialServoControl(HardwareSerial &A)
{
  isAutoEnableRT = true;
	isUseHardwareSerial = true;
	SerialX = (Stream*)(&A);
}
LobotSerialServoControl::LobotSerialServoControl(HardwareSerial &A,int receiveEnablePin1,int transmitEnablePin1)
{
  isAutoEnableRT = false;
  this->receiveEnablePin = receiveEnablePin1;
  this->transmitEnablePin = transmitEnablePin1;
  
  isUseHardwareSerial = true;
  SerialX = (Stream*)(&A);
}

void LobotSerialServoControl::OnInit(void)
{
  if(!isAutoEnableRT)
  {
    pinMode(receiveEnablePin, OUTPUT);
    pinMode(transmitEnablePin, OUTPUT);
    RxEnable();
  }
}

inline void LobotSerialServoControl::RxEnable(void)
{
  digitalWrite(receiveEnablePin, HIGH);
  digitalWrite(transmitEnablePin, LOW);
}
inline void LobotSerialServoControl::TxEnable(void)
{
  digitalWrite(receiveEnablePin, LOW);
  digitalWrite(transmitEnablePin, HIGH);
}

byte LobotSerialServoControl::LobotCheckSum(byte buf[])
{
  byte i;
  uint16_t temp = 0;
  for (i = 2; i < buf[3] + 2; i++) {
    temp += buf[i];
  }
  temp = ~temp;
  i = (byte)temp;
  return i;
}

void LobotSerialServoControl::LobotSerialServoMove(uint8_t id, int16_t position, uint16_t time)
{
  byte buf[10];
  if(position < 0)
    position = 0;
  if(position > 1000)
    position = 1000;
  buf[0] = buf[1] = LOBOT_SERVO_FRAME_HEADER;
  buf[2] = id;
  buf[3] = 7;
  buf[4] = LOBOT_SERVO_MOVE_TIME_WRITE;
  buf[5] = GET_LOW_BYTE(position);
  buf[6] = GET_HIGH_BYTE(position);
  buf[7] = GET_LOW_BYTE(time);
  buf[8] = GET_HIGH_BYTE(time);
  buf[9] = LobotCheckSum(buf);
  if(isAutoEnableRT == false)
    TxEnable();
  SerialX->write(buf, 10);
}

void LobotSerialServoControl::LobotSerialServoStopMove(uint8_t id)
{
  byte buf[6];
  buf[0] = buf[1] = LOBOT_SERVO_FRAME_HEADER;
  buf[2] = id;
  buf[3] = 3;
  buf[4] = LOBOT_SERVO_MOVE_STOP;
  buf[5] = LobotCheckSum(buf);
  if(isAutoEnableRT == false)
    TxEnable();
  SerialX->write(buf, 6);
}

void LobotSerialServoControl::LobotSerialServoSetID(uint8_t oldID, uint8_t newID)
{
  byte buf[7];
  buf[0] = buf[1] = LOBOT_SERVO_FRAME_HEADER;
  buf[2] = oldID;
  buf[3] = 4;
  buf[4] = LOBOT_SERVO_ID_WRITE;
  buf[5] = newID;
  buf[6] = LobotCheckSum(buf);
  if(isAutoEnableRT == false)
    TxEnable();
  SerialX->write(buf, 7);
  
#ifdef LOBOT_DEBUG
  Serial.println("LOBOT SERVO ID WRITE");
  int debug_value_i = 0;
  for (debug_value_i = 0; debug_value_i < buf[3] + 3; debug_value_i++)
  {
    Serial.print(buf[debug_value_i], HEX);
    Serial.print(":");
  }
  Serial.println(" ");
#endif

}

void LobotSerialServoControl::LobotSerialServoSetMode(uint8_t id, uint8_t Mode, int16_t Speed)
{
  byte buf[10];

  buf[0] = buf[1] = LOBOT_SERVO_FRAME_HEADER;
  buf[2] = id;
  buf[3] = 7;
  buf[4] = LOBOT_SERVO_OR_MOTOR_MODE_WRITE;
  buf[5] = Mode;
  buf[6] = 0;
  buf[7] = GET_LOW_BYTE((uint16_t)Speed);
  buf[8] = GET_HIGH_BYTE((uint16_t)Speed);
  buf[9] = LobotCheckSum(buf);

#ifdef LOBOT_DEBUG
  Serial.println("LOBOT SERVO Set Mode");
  int debug_value_i = 0;
  for (debug_value_i = 0; debug_value_i < buf[3] + 3; debug_value_i++)
  {
    Serial.print(buf[debug_value_i], HEX);
    Serial.print(":");
  }
  Serial.println(" ");
#endif
  if(isAutoEnableRT == false)
    TxEnable();
  SerialX->write(buf, 10);
}

void LobotSerialServoControl::LobotSerialServoLoad(uint8_t id)
{
  byte buf[7];
  buf[0] = buf[1] = LOBOT_SERVO_FRAME_HEADER;
  buf[2] = id;
  buf[3] = 4;
  buf[4] = LOBOT_SERVO_LOAD_OR_UNLOAD_WRITE;
  buf[5] = 1;
  buf[6] = LobotCheckSum(buf);
  if(isAutoEnableRT == false)
    TxEnable();
  SerialX->write(buf, 7);
  
#ifdef LOBOT_DEBUG
  Serial.println("LOBOT SERVO LOAD WRITE");
  int debug_value_i = 0;
  for (debug_value_i = 0; debug_value_i < buf[3] + 3; debug_value_i++)
  {
    Serial.print(buf[debug_value_i], HEX);
    Serial.print(":");
  }
  Serial.println(" ");
#endif

}

void LobotSerialServoControl::LobotSerialServoUnload(uint8_t id)
{
  byte buf[7];
  buf[0] = buf[1] = LOBOT_SERVO_FRAME_HEADER;
  buf[2] = id;
  buf[3] = 4;
  buf[4] = LOBOT_SERVO_LOAD_OR_UNLOAD_WRITE;
  buf[5] = 0;
  buf[6] = LobotCheckSum(buf);
  if(isAutoEnableRT == false)
    TxEnable();
  SerialX->write(buf, 7);
  
#ifdef LOBOT_DEBUG
  Serial.println("LOBOT SERVO LOAD WRITE");
  int debug_value_i = 0;
  for (debug_value_i = 0; debug_value_i < buf[3] + 3; debug_value_i++)
  {
    Serial.print(buf[debug_value_i], HEX);
    Serial.print(":");
  }
  Serial.println(" ");
#endif
}

int LobotSerialServoControl::LobotSerialServoReceiveHandle(byte *ret)
{
  bool frameStarted = false;
  bool receiveFinished = false;
  byte frameCount = 0;
  byte dataCount = 0;
  byte dataLength = 2;
  byte rxBuf;
  byte recvBuf[32];
  byte i;

  while (SerialX->available()) {
    rxBuf = SerialX->read();
    delayMicroseconds(100);
    if (!frameStarted) {
      if (rxBuf == LOBOT_SERVO_FRAME_HEADER) {
        frameCount++;
        if (frameCount == 2) {
          frameCount = 0;
          frameStarted = true;
          dataCount = 1;
        }
      }
      else {
        frameStarted = false;
        dataCount = 0;
        frameCount = 0;
      }
    }
    if (frameStarted) {
      recvBuf[dataCount] = (uint8_t)rxBuf;
      if (dataCount == 3) {
        dataLength = recvBuf[dataCount];
        if (dataLength < 3 || dataCount > 7) {
          dataLength = 2;
          frameStarted = false;
        }
      }
      dataCount++;
      if (dataCount == dataLength + 3) {
        
#ifdef LOBOT_DEBUG
        Serial.print("RECEIVE DATA:");
        for (i = 0; i < dataCount; i++) {
          Serial.print(recvBuf[i], HEX);
          Serial.print(":");
        }
        Serial.println(" ");
#endif

        if (LobotCheckSum(recvBuf) == recvBuf[dataCount - 1]) {
          
#ifdef LOBOT_DEBUG
          Serial.println("Check SUM OK!!");
          Serial.println("");
#endif

          frameStarted = false;
          memcpy(ret, recvBuf + 4, dataLength);
          return 1;
        }
        return -1;
      }
    }
  }
  return -1;
}

int LobotSerialServoControl::LobotSerialServoReadPosition(uint8_t id)
{
  int count = 10000;
  int ret;
  byte buf[6];

  buf[0] = buf[1] = LOBOT_SERVO_FRAME_HEADER;
  buf[2] = id;
  buf[3] = 3;
  buf[4] = LOBOT_SERVO_POS_READ;
  buf[5] = LobotCheckSum(buf);

#ifdef LOBOT_DEBUG
  Serial.println("LOBOT SERVO Pos READ");
  int debug_value_i = 0;
  for (debug_value_i = 0; debug_value_i < buf[3] + 3; debug_value_i++)
  {
    Serial.print(buf[debug_value_i], HEX);
    Serial.print(":");
  }
  Serial.println(" ");
#endif

  while (SerialX->available())
    SerialX->read();
    
  if(isAutoEnableRT == false)
    TxEnable();
  SerialX->write(buf, 6);
  if(isUseHardwareSerial)
  {
    delayMicroseconds(600);
  }
  if(isAutoEnableRT == false)
    RxEnable();
  while (!SerialX->available()) {
    count -= 1;
    if (count < 0)
      return -1;
  }

  if (LobotSerialServoReceiveHandle(buf) > 0)
    ret = BYTE_TO_HW(buf[2], buf[1]);
  else
    ret = -2048;

#ifdef LOBOT_DEBUG
  Serial.println(ret);
#endif
  return ret;
}

int LobotSerialServoControl::LobotSerialServoReadVin(uint8_t id)
{
  int count = 10000;
  int ret;
  byte buf[6];

  buf[0] = buf[1] = LOBOT_SERVO_FRAME_HEADER;
  buf[2] = id;
  buf[3] = 3;
  buf[4] = LOBOT_SERVO_VIN_READ;
  buf[5] = LobotCheckSum(buf);

#ifdef LOBOT_DEBUG
  Serial.println("LOBOT SERVO VIN READ");
  int debug_value_i = 0;
  for (debug_value_i = 0; debug_value_i < buf[3] + 3; debug_value_i++)
  {
    Serial.print(buf[debug_value_i], HEX);
    Serial.print(":");
  }
  Serial.println(" ");
#endif

  while (SerialX->available())
    SerialX->read();

  if(isAutoEnableRT == false)
    TxEnable();
    
  SerialX->write(buf, 6);

  if(isUseHardwareSerial)
  {
    delayMicroseconds(600);
  }
  if(isAutoEnableRT == false)
    RxEnable();
  
  while (!SerialX->available()) {
    count -= 1;
    if (count < 0)
      return -2048;
  }

  if (LobotSerialServoReceiveHandle(buf) > 0)
    ret = (int16_t)BYTE_TO_HW(buf[2], buf[1]);
  else
    ret = -2048;

#ifdef LOBOT_DEBUG
  Serial.println(ret);
#endif
  return ret;
}

const int pumpPin1 = 21;
const int pumpPin2 = 19;
const int valvePin1 = 18;
const int valvePin2 = 5;

void Nozzle_init(){
    pinMode(pumpPin1, OUTPUT);
    pinMode(pumpPin2, OUTPUT);
    pinMode(valvePin1, OUTPUT);
    pinMode(valvePin2, OUTPUT);
    digitalWrite(pumpPin1,LOW);
    digitalWrite(pumpPin2,LOW);
    digitalWrite(valvePin1,LOW);
    digitalWrite(valvePin2,LOW);  
}

void Pump_on(){
    digitalWrite(pumpPin1,LOW);
    digitalWrite(pumpPin2,HIGH);
}

void Valve_on(){
    digitalWrite(pumpPin1,LOW);
    digitalWrite(pumpPin2,LOW);
    digitalWrite(valvePin1,LOW);
    digitalWrite(valvePin2,HIGH);
}

void Valve_off(){
    digitalWrite(valvePin1,LOW);
    digitalWrite(valvePin2,LOW);
}

int Servo::channel_next_free = 0;

Servo::Servo(){ _resetFields();}

Servo::~Servo(){ detach();}

bool Servo::attach(int pin, int channel, 
                   int minAngle, int maxAngle, 
                   int minPulseWidth, int maxPulseWidth) 
{
    if(channel == CHANNEL_NOT_ATTACHED) {
        if(channel_next_free == CHANNEL_MAX_NUM) {
            return false;
        }
        _channel = channel_next_free;
        channel_next_free++;
    } else {
        _channel = channel;
    }

    _pin = pin;
    _minAngle = minAngle;
    _maxAngle = maxAngle;
    _minPulseWidth = minPulseWidth;
    _maxPulseWidth = maxPulseWidth;

    ledcSetup(_channel, 50, 16); // channel X, 50 Hz, 16-bit depth
    ledcAttachPin(_pin, _channel);
    return true;
}

bool Servo::detach() {
    if (!this->attached()) {
        return false;
    }

    if(_channel == (channel_next_free - 1))
        channel_next_free--;

    ledcDetachPin(_pin);
    _pin = PIN_NOT_ATTACHED;
    return true;
}

void Servo::write(int degrees) {
    degrees = constrain(degrees, _minAngle, _maxAngle);
    writeMicroseconds(_angleToUs(degrees));
}

void Servo::writeMicroseconds(int pulseUs) {
    if (!attached()) {
        return;
    }
    pulseUs = constrain(pulseUs, _minPulseWidth, _maxPulseWidth);
    _pulseWidthDuty = _usToDuty(pulseUs);
    ledcWrite(_channel, _pulseWidthDuty);
}

int Servo::read() {
    return _usToAngle(readMicroseconds());
}

int Servo::readMicroseconds() {
    if (!this->attached()) {
        return 0;
    }
    int duty = ledcRead(_channel);
    return _dutyToUs(duty);
}

bool Servo::attached() const { return _pin != PIN_NOT_ATTACHED; }

int Servo::attachedPin() const { return _pin; }

void Servo::_resetFields(void) {
    _pin = PIN_NOT_ATTACHED;
    _pulseWidthDuty = 0;
    _channel = CHANNEL_NOT_ATTACHED;
    _minAngle = MIN_ANGLE;
    _maxAngle = MAX_ANGLE;
    _minPulseWidth = MIN_PULSE_WIDTH;
    _maxPulseWidth = MAX_PULSE_WIDTH;
}

Servo servo1;
Servo servo2;
static const int servo1Pin = 15;
static const int servo2Pin = 4;

int status = 0;
int init_pulse[2] = {500,500};

void PWMServo_init(){
    servo1.attach(servo1Pin);
    servo2.attach(servo2Pin);
}

void SetPWMServo(int id, int pul, int duration){
    if((0 < id) & (id < 3)){
        if(pul < 500) pul = 500;
        if(pul > 2500) pul = 2500;
        if(status){
            int pulse = init_pulse[id-1];
            int value = pul - pulse;
            int degree = duration / 20;
            int d = value / degree;
            for(int count=0; count<int(degree); count++){            
                pulse = int(pulse + d);
                Serial.println(pulse);
                if(id == 1)servo1.writeMicroseconds(pulse);
                else if(id == 2)servo2.writeMicroseconds(pulse); 
                delay(20);
            }
            init_pulse[id-1] = pul;
        }  
        else{
            if(id == 1)servo1.writeMicroseconds(pul); 
            else if(id == 2)servo2.writeMicroseconds(pul); 
            init_pulse[id-1] = pul;
            status = 1;
        }
    }
}

GamepadPtr myGamepads[BP32_MAX_GAMEPADS];

// This callback gets called any time a new gamepad is connected.
// Up to 4 gamepads can be connected at the same time.
void onConnectedGamepad(GamepadPtr gp) {
    bool foundEmptySlot = false;
    for (int i = 0; i < BP32_MAX_GAMEPADS; i++) {
        if (myGamepads[i] == nullptr) {
            Console.printf("CALLBACK: Gamepad is connected, index=%d\n", i);
            // Additionally, you can get certain gamepad properties like:
            // Model, VID, PID, BTAddr, flags, etc.
            GamepadProperties properties = gp->getProperties();
            Console.printf("Gamepad model: %s, VID=0x%04x, PID=0x%04x\n", gp->getModelName(), properties.vendor_id,
                           properties.product_id);
            myGamepads[i] = gp;
            foundEmptySlot = true;
            break;
        }
    }
    if (!foundEmptySlot) {
        Console.println("CALLBACK: Gamepad connected, but could not found empty slot");
    }
}

void onDisconnectedGamepad(GamepadPtr gp) {
    bool foundGamepad = false;

    for (int i = 0; i < BP32_MAX_GAMEPADS; i++) {
        if (myGamepads[i] == gp) {
            Console.printf("CALLBACK: Gamepad is disconnected from index=%d\n", i);
            myGamepads[i] = nullptr;
            foundGamepad = true;
            break;
        }
    }

    if (!foundGamepad) {
        Console.println("CALLBACK: Gamepad disconnected, but not found in myGamepads");
    }
}

int speedx = 0;
int speedy = 0;
int speedz = 0;
int spinPos = 0;
int reachPos = 0;
int heightPos = 0;
int packetCount = 0;
bool wifiFlag = false;
HardwareSerial HardwareSerial(2);
LobotSerialServoControl BusServo(HardwareSerial,receiveEnablePin,transmitEnablePin);


const char* ssid = "KULABS"; //network name goes here
const char* password = "WHX434][{c"; //password goes here
bool udpFlag = false;

//hw_timer_t * timer = NULL;
void turnUDPOn(){
   udp.listen(443); // change this if udp port needs to be changed
   udp.onPacket([](AsyncUDPPacket packet) {
         packetCount ++;
         if(packetCount % 2 == 0){// && timerReadMilis(timer) <= 50){  //ignore every other packet
         //timerStop(timer);
         uint8_t all[48];
         for(int i = 0; i < 48; i++){  //put all of the individual coordinate values into their own arrays
            all[i] = *(packet.data() + i);
         }
         uint8_t x[8];
         for(int i = 0; i < 8; i++){
            x[i] = all[i];
         }
         uint8_t y[8];
         for(int i = 0; i < 8; i++){
            y[i] = all[i+8];
         }
         uint8_t z[8];
         for(int i = 0; i < 8; i++){
            z[i] = all[i+16];
         }
         uint8_t yaw[8];
         for(int i = 0; i < 8; i++){
            yaw[i] = all[i+24];
         }
         uint8_t pitch[8];
         for(int i = 0; i < 8; i++){
            pitch[i] = all[i+32];
         }
         uint8_t roll[8];
         for(int i = 0; i < 8; i++){
            roll[i] = all[i+40];
         }
        
         double xCoor;  //treat each array as its own double
         memcpy(&xCoor, x, 8);
         double yCoor;
         memcpy(&yCoor, y, 8);
         double zCoor;
         memcpy(&zCoor, z, 8);
         double yawCoor;
         memcpy(&yawCoor, yaw, 8);
         double pitchCoor;
         memcpy(&pitchCoor, pitch, 8);
         double rollCoor;
         memcpy(&rollCoor, roll, 8);
         
         if(zCoor > 2 || zCoor < -2){ //change this value to adjust deadzone of face tracking for each axis
            reachPos = BusServo.LobotSerialServoReadPosition(2);
                 speedz = (int) zCoor*8;
                 Console.printf("Here is the current reachPos: %d\n",BusServo.LobotSerialServoReadPosition(2));
                 if(reachPos <= 760 && reachPos >= 120 && reachPos != -2048 && reachPos != -1){
                    BusServo.LobotSerialServoMove(2,reachPos+speedz,140);
                    Console.printf("Moving Z\n");
                    delay(30);
                 } 
         }
         if(yawCoor > 5 || yawCoor < -5){
            spinPos = BusServo.LobotSerialServoReadPosition(1);
                 speedx = (int) yawCoor*2;
                 Console.printf("Here is the current spinPos: %d\n",BusServo.LobotSerialServoReadPosition(1));
                 if(spinPos <= 999 && spinPos >= 0 && spinPos != -2048 && spinPos != -1){
                    BusServo.LobotSerialServoMove(1,spinPos-speedx,140);
                    Console.printf("Moving Yaw\n");
                    delay(30);
                 } 
         }
         if(pitchCoor > 5 || pitchCoor < -5){
            heightPos = BusServo.LobotSerialServoReadPosition(3);
                 speedy = (int) pitchCoor*2;
                 Console.printf("Here is the current heightPos: %d\n",BusServo.LobotSerialServoReadPosition(3));
                 if(heightPos <= 870 && heightPos >= 440 && heightPos != -2048 && heightPos != -1){
                    BusServo.LobotSerialServoMove(3,heightPos-speedy,140);
                    Console.printf("Moving Pitch\n");
                    delay(30);
                 } 
         }
         if(rollCoor > 10){
            Console.printf("Pump On\n");
            Pump_on();
            delay(500);
            Valve_off();
         }
         if(rollCoor < -10){
            Console.printf("Pump off\n");
            Valve_on();
         }
         //timerRestart(timer);
         //timerStart(timer);
         }         
      });

   //timer = timerBegin(3, 80, true); 
}



void setup() {
    BP32.setup(&onConnectedGamepad, &onDisconnectedGamepad);
    BP32.forgetBluetoothKeys();
    PWMServo_init();
    Nozzle_init();
    BusServo.OnInit();
    WiFi.mode(WIFI_STA); //turn on wifi. comment this line out if you dont want to connect to wifi
    WiFi.begin(ssid, password); //also comment this line out to turn off wifi
    for(int i = 0; i < 10; i++){
    if (WiFi.status() != WL_CONNECTED) {
      Console.printf("Trying to connect to WiFi...\n");
      delay(1000);
    }
    }
    if(WiFi.status() == WL_CONNECTED){
       Console.print("Wifi connected\n");
       wifiFlag = true;
    }
    else{
    Console.print("Wifi not connected\n");
    wifiFlag = false;
    }
    HardwareSerial.begin(115200,SERIAL_8N1,SERVO_SERIAL_RX,SERVO_SERIAL_TX);
    delay(500);
    Console.printf("Firmware: %s\n", BP32.firmwareVersion());
    
    
    
    // Setup the Bluepad32 callbacks
    //BP32.setup(&onConnectedGamepad, &onDisconnectedGamepad);
    // "forgetBluetoothKeys()" should be called when the user performs
    // a "device factory reset", or similar.
    // Calling "forgetBluetoothKeys" in setup() just as an example.
    // Forgetting Bluetooth keys prevents "paired" gamepads to reconnect.
    // But might also fix some connection / re-connection issues.
    
    
    
    //turnUDPOn(); //uncomment this line...
    //udpFlag = true; //...and this line to start in head-tracking
    Valve_on();
    }

// Arduino loop function. Runs in CPU 1
void loop() {
    // This call fetches all the gamepad info from the NINA (ESP32) module.
    // Just call this function in your main loop.
    // The gamepads pointer (the ones received in the callbacks) gets updated
    // automatically.
    
    
    // It is safe to always do this before using the gamepad API.
    // This guarantees that the gamepad is valid and connected.
    //Console.print("In the loop\n");
    BP32.update();
    //for (int i = 0; i < BP32_MAX_GAMEPADS; i++) { //put this back in if multiple controllers becomes a need
        GamepadPtr myGamepad = myGamepads[0];

        if (myGamepad && myGamepad->isConnected() && !udpFlag) {
            Console.printf(
                "idx=%d, dpad: 0x%02x, buttons: 0x%04x, axis L: %4d, %4d, axis R: %4d, "
                "%4d, brake: %4d, throttle: %4d, misc: 0x%02x\n",
                0,                        // Gamepad Index
                myGamepad->dpad(),        // DPAD
                myGamepad->buttons(),     // bitmask of pressed buttons
                myGamepad->axisX(),       // (-511 - 512) left X Axis
                myGamepad->axisY(),       // (-511 - 512) left Y axis
                myGamepad->axisRX(),      // (-511 - 512) right X axis
                myGamepad->axisRY(),      // (-511 - 512) right Y axis
                myGamepad->brake(),       // (0 - 1023): brake button (left trigger)
                myGamepad->throttle(),    // (0 - 1023): throttle (AKA gas) button (right trigger)
                myGamepad->miscButtons()  // bitmak of pressed "misc" buttons
            );
            
            if (myGamepad->a()) {     
               
                Pump_on();
                delay(500);
                Valve_off();
                delay(250);
                
            }
            if (myGamepad->b()) {
               Valve_on();
               delay(250); 


            }
            if (myGamepad->axisX() < -10) {
              Console.printf("Here is the current spinPos: %d\n",BusServo.LobotSerialServoReadPosition(1));
              spinPos = BusServo.LobotSerialServoReadPosition(1);
              speedx = myGamepad->axisX()/20;
              if(spinPos <= 990 && spinPos != -2048 && spinPos != -1){
                BusServo.LobotSerialServoMove(1,spinPos-speedx,200);
                delay(50); 
              }

            }
            
            if (myGamepad->axisX() > 10) {
              Console.printf("Here is the current spinPos: %d\n",BusServo.LobotSerialServoReadPosition(1));
              spinPos = BusServo.LobotSerialServoReadPosition(1);
              speedx = myGamepad->axisX()/20;
              if(spinPos >= 10 && spinPos != -2048 && spinPos != -1){
                BusServo.LobotSerialServoMove(1,spinPos-speedx,200);
                delay(50); 
              }

            }
            
            if (myGamepad->axisY() < -10) {
              Console.printf("Here is the current reachPos: %d\n",BusServo.LobotSerialServoReadPosition(2));
              reachPos = BusServo.LobotSerialServoReadPosition(2);
              speedy = myGamepad->axisY()/20;
              if(reachPos <= 660 && reachPos != -2048 && reachPos != -1){
                 BusServo.LobotSerialServoMove(2,reachPos+speedy,200);
                 delay(50);
 
               }           
            }
            if (myGamepad->axisY() > 10) {
              Console.printf("Here is the current reachPos: %d\n",BusServo.LobotSerialServoReadPosition(2));
              reachPos = BusServo.LobotSerialServoReadPosition(2);
              speedy = myGamepad->axisY()/20;
              if(reachPos >= 120 && reachPos != -2048 && reachPos != -1){
                 BusServo.LobotSerialServoMove(2,reachPos+speedy,200);
                 delay(50);
 
               } 
            } 
            if (myGamepad->axisRY() < -10) {
              Console.printf("Here is the current heightPos: %d\n",BusServo.LobotSerialServoReadPosition(3));
              heightPos = BusServo.LobotSerialServoReadPosition(3);
              speedz = myGamepad->axisRY()/20;
              if(heightPos >= 500 && heightPos != -2048 && heightPos != -1){
                 BusServo.LobotSerialServoMove(3,heightPos+speedz,200);
                 delay(50);
 
               }           
            }
            if (myGamepad->axisRY() > 10) {
              Console.printf("Here is the current heightPos: %d\n",BusServo.LobotSerialServoReadPosition(3));
              heightPos = BusServo.LobotSerialServoReadPosition(3);
              speedz = myGamepad->axisRY()/20;
              if(heightPos <= 870 && heightPos != -2048 && heightPos != -1){
                 BusServo.LobotSerialServoMove(3,heightPos+speedz,200);
                 delay(50);
 
               }           
            }
            if (myGamepad->dpad() == 0x01){
               if(!udpFlag && wifiFlag){
                  turnUDPOn();
                  Console.print("Turning on UDP...");
                  udpFlag = true;
               }
            }
            delay(150);
        }
        /*else{//if(!myGamepad && !myGamepad->isConnected()){
           BP32.forgetBluetoothKeys();
           delay(10000);
        }*/
    //}    
}
