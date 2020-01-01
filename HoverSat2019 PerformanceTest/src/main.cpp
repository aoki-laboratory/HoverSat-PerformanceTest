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
#include <EEPROM.h>
#include "utility/MPU9250.h"

//Define
//------------------------------------------------------------------//
#define TIMER_INTERRUPT       5

#define ASCALE 2        // 0:2G, 1:4G, 2:8G, 3:16G
#define GSCALE 1        // 0:250dps, 1:500dps, 2:1000dps, 3:2000dps

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
unsigned long time_buff3 = 0;
long          time_current = 0;
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

// Battery
unsigned int cnt_battery;
unsigned char battery_status;
unsigned char battery_persent;

// Parameters
unsigned char hover_val = 70;
bool hover_flag = false;
bool hover_flag2 = false;
unsigned char hover_pattern = 0;
unsigned int hover_time = 5000;
unsigned int ex_time = 500;
unsigned char patternNo = 0;
unsigned char flag = 0;
bool cnt_flag = false;

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
uint8_t getBatteryGauge(void);
void taskInit(void);

//Setup #1
//------------------------------------------------------------------//
void setup() {
  M5.begin();
  delay(1000);
  connectToWiFi();
  while(!connected){
    delay(1);
  }
  xTaskCreatePinnedToCore(&taskDisplay, "taskDisplay", 6144, NULL, 10, &task_handl, 0);

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

  if(GSCALE == 0) {
    IMU.writeByte(MPU9250_ADDRESS, GYRO_CONFIG, 0x00);  // 250dps
  } else if(GSCALE == 1) {
    IMU.writeByte(MPU9250_ADDRESS, GYRO_CONFIG, 0x08);  // 500dps
  } else if(GSCALE == 2) {
    IMU.writeByte(MPU9250_ADDRESS, GYRO_CONFIG, 0x10);  // 1000dps
  } else {
    IMU.writeByte(MPU9250_ADDRESS, GYRO_CONFIG, 0x18);  // 2000dps
  }

  if(ASCALE == 0) {
    IMU.writeByte(MPU9250_ADDRESS, ACCEL_CONFIG, 0x00); // 2G
  } else if(ASCALE == 1) {
    IMU.writeByte(MPU9250_ADDRESS, ACCEL_CONFIG, 0x08); // 4G
  } else if(ASCALE == 2) {
    IMU.writeByte(MPU9250_ADDRESS, ACCEL_CONFIG, 0x10); // 8G
  } else {
    IMU.writeByte(MPU9250_ADDRESS, ACCEL_CONFIG, 0x18); // 16G
  }

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
    
    case 111:    
      time_buff = millis();
      hover_flag = true;
      M5.Lcd.setTextSize(3);
      M5.Lcd.setCursor(80, 40);
      M5.Lcd.setTextColor(TFT_DARKGREY);
      M5.Lcd.printf("Hover Disable");
      M5.Lcd.setCursor(80, 40);
      M5.Lcd.setTextColor(WHITE);
      M5.Lcd.printf("Hover PWM %3d", hover_val);
      DuctedFan.attach(DuctedFanPin);
      DuctedFan.write(0);
      M5.Lcd.fillRect(0, 20, 60, 60, TFT_LIGHTGREY);
      time_buff2 = 0;
      time_buff3 = 0;
      pattern = 112;
      break;
    
    case 112:
      if(cnt_flag) {
        M5.Lcd.setTextSize(4);
        M5.Lcd.setCursor(8, 36);
        M5.Lcd.setTextColor(TFT_LIGHTGREY);
        M5.Lcd.printf("%2d", time_buff3);
        M5.Lcd.setTextSize(4);
        M5.Lcd.setCursor(8, 36);
        M5.Lcd.setTextColor(BLACK);
        M5.Lcd.printf("%2d", time_buff2);
        cnt_flag = false;
      }
      time_buff3 = time_buff2;
      time_buff2 = (10000-(millis()-time_buff))/1000;
      if(time_buff2 < time_buff3) {
        cnt_flag = true;
      }
      if( millis() - time_buff >= 3000 ) {
        DuctedFan.write(hover_val);
        pattern = 113;
      }
      break;
    
    case 113:
      if(cnt_flag) {
        M5.Lcd.setTextSize(4);
        M5.Lcd.setCursor(8, 36);
        M5.Lcd.setTextColor(TFT_LIGHTGREY);
        M5.Lcd.printf("%2d", time_buff3);
        M5.Lcd.setTextSize(4);
        M5.Lcd.setCursor(8, 36);
        M5.Lcd.setTextColor(BLACK);
        M5.Lcd.printf("%2d", time_buff2);
        cnt_flag = false;
      }
      time_buff3 = time_buff2;
      time_buff2 = (10000-(millis()-time_buff))/1000;
      if(time_buff2 < time_buff3) {
        cnt_flag = true;
      }
      if( millis() - time_buff >= 7000 ) {
        log_flag = true;
        pattern = 114;
      }
      break;

    case 114:
      if(cnt_flag) {
        M5.Lcd.setTextSize(4);
        M5.Lcd.setCursor(8, 36);
        M5.Lcd.setTextColor(TFT_LIGHTGREY);
        M5.Lcd.printf("%2d", time_buff3);
        M5.Lcd.setTextSize(4);
        M5.Lcd.setCursor(8, 36);
        M5.Lcd.setTextColor(BLACK);
        M5.Lcd.printf("%2d", time_buff2);
        cnt_flag = false;
      }
      time_buff3 = time_buff2;
      time_buff2 = (10000-(millis()-time_buff))/1000;
      if(time_buff2 < time_buff3) {
        cnt_flag = true;
      }
      if( millis() - time_buff >= 10000 ) {
        time_buff = millis();
        pattern = 115;
      }
      break;
    
    case 115:
      if( millis() - time_buff >= parameters[patternNo][2] ) {
        pattern = 0;
        cnt_flag = false;
        DuctedFan.detach();
        log_flag = false;
        delay(50);
        M5.Lcd.setTextSize(3);
        M5.Lcd.setCursor(80, 40);
        M5.Lcd.setTextColor(TFT_DARKGREY);
        M5.Lcd.printf("Hover PWM %3d", hover_val);
        M5.Lcd.setCursor(80, 40);
        M5.Lcd.setTextColor(WHITE);
        M5.Lcd.printf("Hover Disable");
        M5.Lcd.fillRect(0, 20, 60, 60, TFT_LIGHTGREY);
        M5.Lcd.setTextSize(4);
        M5.Lcd.setCursor(8, 36);
        M5.Lcd.setTextColor(BLACK);
        M5.Lcd.print("St");
        file.close();
      }    
      break;

  }
}

//Main #0
//------------------------------------------------------------------//
void taskDisplay(void *pvParameters){

  EEPROM.begin(128);
  hover_val = EEPROM.read(0);
  taskInit();  

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
    cnt_battery++;
    if( cnt_battery >= 5000 && !log_flag ) {
      M5.Lcd.setTextSize(2);
      M5.Lcd.setCursor(280, 2);
      M5.Lcd.setTextColor(WHITE);
      M5.Lcd.printf("%3d",battery_persent);
      battery_status = getBatteryGauge();
      switch (battery_status) {
      case 0xF0:
        battery_persent = 0;
        break;
      case 0xE0:
        battery_persent = 25;
        break;
      case 0xC0:
        battery_persent = 50;
        break;
      case 0x80:
        battery_persent = 75;
        break;
      case 0x00:
        battery_persent = 100;
        break;        
      }
      M5.Lcd.setTextSize(2);
      M5.Lcd.setCursor(280, 2);
      M5.Lcd.setTextColor(BLACK);
      M5.Lcd.printf("%3d",battery_persent);
      cnt_battery = 0;
    }
    
  }
}

// Timer Interrupt
//------------------------------------------------------------------//
void Timer_Interrupt( void ){
  if (interruptCounter > 0) {

    portENTER_CRITICAL(&timerMux);
    interruptCounter--;
    portEXIT_CRITICAL(&timerMux);

    if( log_flag ) {
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

      time_current = 10000 - (millis() - time_buff);
      if( pattern == 115 ) {
        file.print(millis()-time_buff);
      } else {
        file.print(time_current * -1);
      }
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
    delay(20);
    M5.Lcd.setTextColor(WHITE);
    M5.Lcd.setTextSize(5);
    M5.Lcd.setCursor(0, 150);
    M5.Lcd.printf("%2d", patternNo+1);
    M5.Lcd.setTextSize(2);
    M5.Lcd.setCursor(80, 120);
    M5.Lcd.printf("Ejection Time %4d", parameters[patternNo][1]);
    M5.Lcd.setCursor(80, 170);
    M5.Lcd.printf("Hovering Time %4d", parameters[patternNo][2]);
    delay(20);
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
  if ((M5.BtnA.wasPressed() && pattern == 0) || hover_flag2) {
    switch (hover_pattern) {
    case 0:
      if(hover_flag) {
        hover_pattern = 5;
        break;
      }
      hover_flag = true;
      M5.Lcd.setTextSize(3);
      M5.Lcd.setCursor(80, 40);
      M5.Lcd.setTextColor(TFT_DARKGREY);
      M5.Lcd.printf("Hover Disable");
      M5.Lcd.setCursor(80, 40);
      M5.Lcd.setTextColor(WHITE);
      M5.Lcd.printf("Hover PWM %3d", hover_val);
      DuctedFan.attach(DuctedFanPin);
      DuctedFan.write(0);
      hover_flag2 = true;
      hover_pattern = 1;
      time_buff3 = millis();
      break;
    
    case 1:
      if(millis()-time_buff3 >= 3000) {
        M5.Lcd.setCursor(80, 40);
        M5.Lcd.setTextColor(BLACK);
        M5.Lcd.printf("Hover PWM %3d", hover_val);  
        hover_pattern = 2;
        break;
      }
      M5.Lcd.setTextSize(3);
      M5.Lcd.setCursor(80, 40);
      M5.Lcd.setTextColor(TFT_DARKGREY);
      M5.Lcd.printf("          %3d", hover_val);
      if(M5.BtnA.wasPressed()) {
        hover_val += 5;
        if(hover_val > 100) {
          hover_val = 60;
        }     
      }     
      EEPROM.write(0, hover_val);
      EEPROM.commit();
      M5.Lcd.setCursor(80, 40);
      M5.Lcd.setTextColor(WHITE);
      M5.Lcd.printf("          %3d", hover_val); 
      break;
    
    case 2:       
      DuctedFan.write(hover_val);
      hover_pattern = 0;
      break;

    case 5:  
      if(M5.BtnA.wasPressed()) {  
        M5.Lcd.setTextSize(3);
        M5.Lcd.setCursor(80, 40);
        M5.Lcd.setTextColor(TFT_DARKGREY);
        M5.Lcd.printf("Hover PWM %3d", hover_val);
        M5.Lcd.setCursor(80, 40);
        M5.Lcd.setTextColor(WHITE);
        M5.Lcd.printf("Hover Disable");
        DuctedFan.detach();
        hover_flag = false;
        hover_flag2 = false;
        hover_pattern = 0;
      }
      break;

    }

    /*hover_flag = !hover_flag;    
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
    }*/
  } else if (M5.BtnB.wasPressed() && pattern == 0) {
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

  } else if (M5.BtnC.wasPressed() && pattern == 0) {
    udp_pattern = 111;
    sendUDP();
    udp_pattern = 0;
    pattern = 111;
  }
} 

uint8_t getBatteryGauge() {
  Wire.beginTransmission(0x75);
  Wire.write(0x78);
  Wire.endTransmission(false);
  if(Wire.requestFrom(0x75, 1)) {
    return Wire.read();
  }
  return 0xff;
}

void taskInit() {
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
}
