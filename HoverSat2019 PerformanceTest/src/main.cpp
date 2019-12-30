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
#include <WiFi.h>
#include <WiFiUdp.h>
#include <Wire.h>
#include <time.h>
#include "utility/MPU9250.h"

//Define
//------------------------------------------------------------------//
#define TIMER_INTERRUPT       1

#define NOOFPATTERNS 5
int parameters[NOOFPATTERNS][2] =
{
// PWM, EjctionTime/10
{ 20, 50 },
{ 40, 50 },
{ 60, 50 },
{ 80, 50 },
{ 100, 50 },
};

//Global
//------------------------------------------------------------------//
const char* ssid = "HoverSat-2019"; 
const char* password = "root0123";
 
const char * to_udp_address = "192.168.4.1";
const int to_udp_port = 55555;
const int my_server_udp_port = 55556;

unsigned char udp_pattern = 0;
unsigned char udp_aa = 0;
unsigned char udp_bb = 0;
unsigned char udp_flag = 0;

unsigned char pattern = 0;
bool log_flag = false;

unsigned long time_ms;
unsigned long time_buff = 0;
volatile int interruptCounter;
int iTimer10;
 
WiFiUDP udp;
TaskHandle_t task_handl;
hw_timer_t * timer = NULL;
portMUX_TYPE timerMux = portMUX_INITIALIZER_UNLOCKED;

// Parameters
unsigned char hover_val = 70;
unsigned int ex_time = 100;
unsigned char patternNo = 0;

unsigned char flag = 0;

//Prototype
//------------------------------------------------------------------//
void receiveUDP(void);
void sendUDP(void);
void setupWiFiUDPserver(void);
void button_action(void);
void taskDisplay(void *pvParameters);
void IRAM_ATTR onTimer(void);
void Timer_Interrupt(void);
void LCD_Control(void);

//Setup #1
//------------------------------------------------------------------//
void setup() {
  M5.begin();
  delay(1000);
  setupWiFiUDPserver();
  
  xTaskCreatePinnedToCore(&taskDisplay, "taskDisplay", 4096, NULL, 10, &task_handl, 0);

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
  receiveUDP();
  Timer_Interrupt(); 

  switch (pattern) {
    case 0:
      break;

    case 11:    
      time_buff = millis();
      pattern = 12;
      break;

    case 12:
      if( millis() - time_buff >= ex_time*10 ) {
        M5.Lcd.fillRect(0, 0, 80, 80, TFT_RED);
        pattern = 0;
      }
      break; 

  }
}

//Main #0
//------------------------------------------------------------------//
void taskDisplay(void *pvParameters){
  while(1){    
    M5.update();
    button_action();
    LCD_Control();
    delay(100);
  }
}

// Timer Interrupt
//------------------------------------------------------------------//
void Timer_Interrupt( void ){
  if (interruptCounter > 0) {

    portENTER_CRITICAL(&timerMux);
    interruptCounter--;
    portEXIT_CRITICAL(&timerMux);

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
  M5.Lcd.fillRect(0, 0, 80, 80, TFT_WHITE);
  M5.Lcd.fillRect(80, 0, 240, 80, TFT_DARKGREY);
  M5.Lcd.fillRect(0, 80, 80, 160, TFT_DARKGREY);
  M5.Lcd.setTextSize(5);
  M5.Lcd.setCursor(13, 23);
  M5.Lcd.setTextColor(BLACK);
  M5.Lcd.print("Ej");
  M5.Lcd.setTextSize(3);
  M5.Lcd.setCursor(96, 30);
  M5.Lcd.setTextColor(WHITE);
  M5.Lcd.printf("PWM      %3d", parameters[patternNo][0]);
  M5.Lcd.setCursor(15, 120);
  M5.Lcd.print("No.");
  M5.Lcd.setTextSize(5);
  M5.Lcd.setCursor(10, 160);
  M5.Lcd.printf("%2d", patternNo+1);

  M5.Lcd.setTextSize(2);
  M5.Lcd.setCursor(96, 112);
  M5.Lcd.printf("Ejection Time %4d", parameters[patternNo][1]*10);

}

void receiveUDP(){
  int packetSize = udp.parsePacket();
  if(packetSize > 0){
    pattern = udp.read();
    udp_aa = udp.read();
    udp_bb = udp.read();
    udp_flag = udp.read();
  }
}
 
void sendUDP(){
  udp.beginPacket(to_udp_address, to_udp_port);
  udp.write(udp_pattern);
  udp.write(udp_aa);
  udp.write(udp_bb);
  udp.write(udp_flag);
  udp.endPacket();
}
 
void setupWiFiUDPserver(){
  WiFi.disconnect(true, true);
  WiFi.softAP(ssid, password);
  IPAddress myIP = WiFi.softAPIP();
  udp.begin(myIP, my_server_udp_port);
  delay(1000);
}
 
void button_action(){
  if (M5.BtnA.isPressed()) {
  } else if (M5.BtnB.isPressed()) {
  } else if (M5.BtnC.isPressed()) {
    udp_pattern = 11;
    udp_aa = 12;
    udp_bb = 13;
    udp_flag = 0;
    sendUDP();
    delay(1000);
    pattern = 11;
  }
} 


