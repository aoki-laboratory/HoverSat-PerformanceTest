//------------------------------------------------------------------//
//Supported MCU:   ESP32 (M5Stack)
//File Contents:   HoverSat EjectionSystem
//Version number:  Ver.1.0
//Date:            2019.12.29
//------------------------------------------------------------------//
 
//This program supports the following boards:
//* M5Stack(Grey version)
 
//Include
//------------------------------------------------------------------//
#include <M5Stack.h>
#include <Servo.h>
#include <WiFi.h>
#include <WiFiUdp.h>
#include <Wire.h>
#include <time.h>
#include "utility/MPU9250.h"

//Define
//------------------------------------------------------------------//
#define TIMER_INTERRUPT       5

#define NOOFPATTERNS 5
int parameters[NOOFPATTERNS][3] =
{
// PWM, EjctionTime, HoverTime
{ 20, 100, 5000 },
{ 40, 200, 5000 },
{ 60, 300, 5000 },
{ 80, 400, 5000 },
{ 100, 500, 5000 },
};

//Global
//------------------------------------------------------------------//
const char* ssid = "HoverSat-2019"; 
const char* password = "root0123";
 
const char * to_udp_address = "192.168.4.1";
const int to_udp_port = 55555;
const int my_server_udp_port = 55556;

unsigned char udp_pattern = 0;
unsigned char udp_No = 0;
unsigned char udp_bb = 0;
unsigned char udp_flag = 0;

unsigned char pattern = 0;
bool log_flag = false;
unsigned char core0_pattern = 0;

unsigned long time_ms;
unsigned long time_buff = 0;
unsigned long time_buff2 = 0;
volatile int interruptCounter;
int iTimer10;

// WiFi
WiFiUDP udp;
TaskHandle_t task_handl;
bool connected = false;

// Timer
hw_timer_t * timer = NULL;
portMUX_TYPE timerMux = portMUX_INITIALIZER_UNLOCKED;

// MPU9250
MPU9250 IMU; 
float accelBiasX = 0;
float accelBiasY = 0;
float accelBiasZ = 0;
float gyroBiasZ = 0;

// DuctedFan
static const int DuctedFanPin = 15;
Servo DuctedFan;

//SD
File file;
String fname_buff;
const char* fname;

// Parameters
unsigned char hover_val = 70;
bool hover_flag = false;
unsigned int hover_time = 5000;
unsigned int ex_time = 500;
unsigned char patternNo = 0;
unsigned char flag = 0;

//Prototype
//------------------------------------------------------------------//
void receiveUDP(void);
void sendUDP(void);
void button_action(void);
void taskDisplay(void *pvParameters);
void IRAM_ATTR onTimer(void);
void Timer_Interrupt(void);
void LCD_Control(void);
void connectToWiFi(void);
void WiFiEvent(WiFiEvent_t event);

//Setup #1
//------------------------------------------------------------------//
void setup() {
  M5.begin();
  delay(1000);
  connectToWiFi();
  while(!connected){
    delay(1);
  }
  xTaskCreatePinnedToCore(&taskDisplay, "taskDisplay", 8192, NULL, 10, &task_handl, 0);

  // Create Log File
  fname_buff  = "/log/Satellite_log.csv";
  fname = fname_buff.c_str();

  SD.begin(4, SPI, 24000000);
  // Create Log File
  file = SD.open(fname, FILE_APPEND);
  if( !file ) {
    M5.Lcd.setTextSize(3);
    M5.Lcd.setTextColor(WHITE);
    M5.Lcd.setCursor(5, 160);
    M5.Lcd.println("Failed to open sd");
  }

  // Initialize IIC
  Wire.begin();
  Wire.setClock(400000);

  // Initialize MPU9250
  IMU.calibrateMPU9250(IMU.gyroBias, IMU.accelBias);
  IMU.initMPU9250();

  // Initialize Timer Interrupt
  timer = timerBegin(0, 80, true);
  timerAttachInterrupt(timer, &onTimer, true);
  timerAlarmWrite(timer, TIMER_INTERRUPT * 1000, true);
  timerAlarmEnable(timer); 
  delay(500);
}

//Main #1
//------------------------------------------------------------------//
void loop() {
  
  Timer_Interrupt(); 

  switch (pattern) {
    case 0:
      break;

    case 11:    
      M5.Lcd.fillRect(0, 0, 80, 80, TFT_RED);
      time_buff = millis();
      pattern = 12;
      break;

    case 12:
      if( millis() - time_buff >= ex_time ) {
        M5.Lcd.fillRect(0, 0, 80, 80, TFT_DARKGREY);
        pattern = 0;
      }
      break; 

  }
}

//Main #0
//------------------------------------------------------------------//
void taskDisplay(void *pvParameters){

  M5.Lcd.fillRect(0, 0, 320, 20, TFT_WHITE);
  M5.Lcd.fillRect(60, 20, 260, 60, TFT_DARKGREY);
  M5.Lcd.fillRect(0, 80, 60, 160, TFT_DARKGREY);
  M5.Lcd.fillRect(0, 20, 60, 60, TFT_LIGHTGREY);
  M5.Lcd.fillRect(0, 220, 320, 20, TFT_WHITE);

  M5.Lcd.setTextSize(2);
  M5.Lcd.setCursor(8, 2);
  M5.Lcd.setTextColor(BLACK);
  M5.Lcd.print("HoverSat u Test");
  M5.Lcd.setCursor(40, 222);
  M5.Lcd.print("HOVER");
  M5.Lcd.setCursor(140, 222);
  M5.Lcd.print("MODE");
  M5.Lcd.setCursor(228, 222);
  M5.Lcd.print("START");
  M5.Lcd.setTextSize(4);
  M5.Lcd.setCursor(8, 36);
  M5.Lcd.setTextColor(BLACK);
  M5.Lcd.print("St");
  M5.Lcd.setTextSize(3);
  M5.Lcd.setCursor(80, 40);
  M5.Lcd.setTextColor(WHITE);
  M5.Lcd.printf("Hover Disable");
  M5.Lcd.setTextSize(3);
  M5.Lcd.setTextColor(WHITE);
  M5.Lcd.setCursor(8, 110);
  M5.Lcd.print("No.");

  M5.Lcd.setTextColor(WHITE);
  M5.Lcd.setTextSize(5);
  M5.Lcd.setCursor(0, 150);
  M5.Lcd.printf("%2d", patternNo+1);
  M5.Lcd.setTextSize(2);
  M5.Lcd.setCursor(80, 120);
  M5.Lcd.printf("Ejection Time %4d", parameters[patternNo][1]);
  M5.Lcd.setTextSize(2);
  M5.Lcd.setCursor(80, 170);
  M5.Lcd.printf("Hovering Time %4d", parameters[patternNo][2]);
  

  while(1){    
    M5.update();
    button_action();
    receiveUDP();

    switch (core0_pattern) {
    case 0:
      LCD_Control();
      break;

    case 10:
      core0_pattern = 0;
      break;
    }

     core0_pattern++;
    delay(1);
  }
}

// Timer Interrupt
//------------------------------------------------------------------//
void Timer_Interrupt( void ){
  if (interruptCounter > 0) {

    portENTER_CRITICAL(&timerMux);
    interruptCounter--;
    portEXIT_CRITICAL(&timerMux);

    // If intPin goes high, all data registers have new data
    // On interrupt, check if data ready interrupt
    if (IMU.readByte(MPU9250_ADDRESS, INT_STATUS) & 0x01) {  
      IMU.readAccelData(IMU.accelCount);  // Read the x/y/z adc values
      IMU.getAres();

      IMU.ax = (float)IMU.accelCount[0]; // - accelBias[0];
      IMU.ay = (float)IMU.accelCount[1]; // - accelBias[1];

      IMU.readGyroData(IMU.gyroCount);  // Read the x/y/z adc values
      IMU.getGres();

      IMU.gz = (float)IMU.gyroCount[2];
    }

    file.print(millis());
    file.print(",");
    file.print(pattern);
    file.print(",");
    file.print(IMU.ax);
    file.print(",");
    file.print(IMU.ay);
    file.print(",");
    file.print(IMU.gz);
    file.println(",");

  }
}

// IRAM
//------------------------------------------------------------------//
void IRAM_ATTR onTimer() {
  portENTER_CRITICAL_ISR(&timerMux);
  interruptCounter=1;
  portEXIT_CRITICAL_ISR(&timerMux);
}

// LCD_Control
//------------------------------------------------------------------//
void LCD_Control() {
  

}

void connectToWiFi(){
  Serial.println("Connecting to WiFi network: " + String(ssid));
  WiFi.disconnect(true, true);
  delay(1000);
  WiFi.onEvent(WiFiEvent);
  WiFi.begin(ssid, password);
  Serial.println("Waiting for WIFI connection...");
}

void WiFiEvent(WiFiEvent_t event){
  IPAddress myIP = WiFi.localIP();
  switch(event) {
    case SYSTEM_EVENT_STA_GOT_IP:
      Serial.println("WiFi connected!");
      Serial.print("My IP address: ");
      Serial.println(myIP);
      udp.begin(myIP, my_server_udp_port);
      delay(1000);
      connected = true;
      break;
    case SYSTEM_EVENT_STA_DISCONNECTED:
      Serial.println("WiFi lost connection");
      connected = false;
      break;
    default:
      break;
  }
}

void receiveUDP(){
  int packetSize = udp.parsePacket();
  if(packetSize > 0){
    M5.Lcd.setTextColor(TFT_DARKGREY);
    M5.Lcd.setTextSize(5);
    M5.Lcd.setCursor(0, 150);
    M5.Lcd.printf("%2d", patternNo+1);
    M5.Lcd.setTextColor(BLACK);
    M5.Lcd.setTextSize(2);
    M5.Lcd.setCursor(80, 120);
    M5.Lcd.printf("Ejection Time %4d", parameters[patternNo][1]);
    M5.Lcd.setCursor(80, 170);
    M5.Lcd.printf("Hovering Time %4d", parameters[patternNo][2]);
    pattern = udp.read();
    patternNo = udp.read();
    udp_bb = udp.read();
    udp_flag = udp.read();
    M5.Lcd.setTextColor(WHITE);
    M5.Lcd.setTextSize(5);
    M5.Lcd.setCursor(0, 150);
    M5.Lcd.printf("%2d", patternNo+1);
    M5.Lcd.setTextSize(2);
    M5.Lcd.setCursor(80, 120);
    M5.Lcd.printf("Ejection Time %4d", parameters[patternNo][1]);
    M5.Lcd.setCursor(80, 170);
    M5.Lcd.printf("Hovering Time %4d", parameters[patternNo][2]);
  }
}
 
void sendUDP(){
  udp.beginPacket(to_udp_address, to_udp_port);
  udp.write(udp_pattern);
  udp.write(udp_No);
  udp.write(udp_bb);
  udp.write(udp_flag);
  udp.endPacket();
}
 
void button_action(){
  if (M5.BtnA.wasPressed()) {
    hover_flag = !hover_flag;    
    if(hover_flag) {
        M5.Lcd.setTextSize(3);
        M5.Lcd.setCursor(80, 40);
        M5.Lcd.setTextColor(TFT_DARKGREY);
        M5.Lcd.printf("Hover Disable");
        M5.Lcd.setCursor(80, 40);
        M5.Lcd.setTextColor(WHITE);
        M5.Lcd.printf("Hover PWM %3d", hover_val);
        DuctedFan.attach(DuctedFanPin);
        DuctedFan.write(0);
        time_buff2 = millis();
        delay(3000);
        DuctedFan.write(hover_val); 
    } else {
      M5.Lcd.setTextSize(3);
      M5.Lcd.setCursor(80, 40);
      M5.Lcd.setTextColor(TFT_DARKGREY);
      M5.Lcd.printf("Hover PWM %3d", hover_val);
      M5.Lcd.setCursor(80, 40);
      M5.Lcd.setTextColor(WHITE);
      M5.Lcd.printf("Hover Disable");
      DuctedFan.detach();
    }
  } else if (M5.BtnB.wasPressed()) {
    M5.Lcd.setTextColor(TFT_DARKGREY);
    M5.Lcd.setTextSize(5);
    M5.Lcd.setCursor(0, 150);
    M5.Lcd.printf("%2d", patternNo+1);
    M5.Lcd.setTextColor(BLACK);
    M5.Lcd.setTextSize(2);
    M5.Lcd.setCursor(80, 120);
    M5.Lcd.printf("Ejection Time %4d", parameters[patternNo][1]);
    M5.Lcd.setCursor(80, 170);
    M5.Lcd.printf("Hovering Time %4d", parameters[patternNo][2]);

    patternNo++;
    if( patternNo >= NOOFPATTERNS ) {
      patternNo = 0;
    }
    udp_No = patternNo;
    sendUDP();
    M5.Lcd.setTextColor(WHITE);
    M5.Lcd.setTextSize(5);
    M5.Lcd.setCursor(0, 150);
    M5.Lcd.printf("%2d", patternNo+1);
    M5.Lcd.setTextSize(2);
    M5.Lcd.setCursor(80, 120);
    M5.Lcd.printf("Ejection Time %4d", parameters[patternNo][1]);
    M5.Lcd.setCursor(80, 170);
    M5.Lcd.printf("Hovering Time %4d", parameters[patternNo][2]);

  } else if (M5.BtnC.wasPressed()) {
    udp_pattern = 111;
    sendUDP();
    pattern = 11;
  }
} 


