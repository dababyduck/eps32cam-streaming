// source libraries
#include <Arduino.h>
#include <SPI.h>
#include <TFT_eSPI.h>
#include <WiFi.h>
#include <esp_wifi.h>
#include <esp_event.h>
#include <nvs_flash.h>
#include <esp_log.h>
#include "freertos/queue.h"
#include "freertos/task.h"

#include <pinout.h>

// subsystems
#include <subsystems/prom_wifi.h>



// server definition
const char *ssid = "ESP32S3";
const char *password = "BIGCAMERA864";
const uint16_t portTCP = 1234;
const uint16_t portUDP = 1235;

// display definition
TFT_eSPI display = TFT_eSPI();

// CONFIG
int display_FPS = 30;
// int TEST_MODE = 0;

int deadZoneBoundaries = 109; // in both directions from half, zone where control stick input would be taken as 0

byte invertAxisX = 1; // 0-no, 1-yes
byte invertAxisY = 1; // 0-no, 1-yes

const uint8_t myAddress[] = {0x34, 0x85, 0x18, 0x9b, 0x5c, 0x30};
const uint8_t esp32c3Address[] = {0x94, 0xA9, 0x90, 0x96, 0x61, 0x5C};
// const uint8_t esp32camAddress[] = {0xD0, 0xEF, 0x76, 0xEF, 0x5B, 0xB8};

typedef struct controlpacket
{
  int8_t pitch;
  int8_t roll;
  int8_t power;
} controlpacket;


#define READ_GPIO_FAST(pin) ((GPIO.in >> (pin)) & 0x1) // allegedly has better performance than digitalRead and others

volatile byte encleft_flag = 0;
volatile byte encright_flag = 0;
volatile int8_t encmoved = 0; // 0 idle;-1 left;1 right
int counter = 0;
unsigned long _lastEncRotationMillis = 0;
byte _ENC_DEBOUNCE_TIME = 10; // in ms, or us if using micros
byte update_display = 0;

// TEST FLAGS
byte foundESP32C3, staetofUNKNCLNT, stateofESP32C3CLNT, UNCT0CONNECTED, UNCT0AVAILABLE, CLIENTAMOUNT; // debug flags
uint8_t ___ENC_CHECKTIME()
{
  if (millis() - _lastEncRotationMillis >= _ENC_DEBOUNCE_TIME)
  {
    _lastEncRotationMillis = millis();
    return 1;
  }
  else
    return 0;
}
void IRAM_ATTR _ENC_LEFT_INTERRUPT()
{
  // uint8_t state = digitalRead(ENC_LEFT);
  uint8_t state = READ_GPIO_FAST(ENC_LEFT);
  if (state)
  {
    encleft_flag = 0;
  }
  else
  {
    encleft_flag = 1;
    if (encright_flag)
    {
      encmoved = -1;
      encright_flag = 0;
      encleft_flag = 0;
    }
  }
}
void IRAM_ATTR _ENC_RIGHT_INTERRUPT()
{
  // uint8_t state = digitalRead(ENC_RIGHT);
  uint8_t state = READ_GPIO_FAST(ENC_RIGHT);
  if (state)
  {
    encright_flag = 0;
  }
  else
  {
    encright_flag = 1;
    if (encleft_flag)
    {
      encmoved = 1;
      encleft_flag = 0;
      encright_flag = 0;
    }
  }
}

uint16_t swap_bytes(uint16_t value)
{
  return (value >> 8) | (value << 8);
}

void displayUpdate(uint16_t *buffer, uint16_t width, uint16_t height, size_t size)
{
  display.pushImage(0, 0, width, height, buffer);
}

byte lastpacketwasreceived = 0;
void pw_callback(uint8_t state, int16_t value)
{
  display.fillRect(200, 200, 30, 30, TFT_BLUE);
  if (state == PW_CALLBACK_SIZEMISMATCH)
  {
    display.fillRect(0, 240, 120, 40, TFT_BLACK);
    display.setCursor(0, 240);
    display.setTextColor(TFT_RED);
    display.setTextSize(1);
    display.printf("size:%d\n", value);
  }
  else if (state == PW_CALLBACK_PACKETSRECEIVED)
  {
    if (lastpacketwasreceived)
    {
      display.fillRect(120, 255, 15, 15, TFT_DARKGREEN); // CONTINUOUS RECEIVED
    }
    else
    {
      display.fillRect(120, 240, 15, 15, TFT_GREEN); // FIRST RECEIVED
      display.fillRect(120, 255, 15, 15, TFT_BLACK); // resetting the continuous
      lastpacketwasreceived = 1;
    }
  }
  else if (state == PW_CALLBACK_PACKETSDROPPED)
  {
    if (!lastpacketwasreceived)
    {
      display.fillRect(120, 255, 15, 15, TFT_BROWN); // CONTINUOUS DROPPED
    }
    else
    {
      display.fillRect(120, 240, 15, 15, TFT_RED);   // FIRST DROPPED
      display.fillRect(120, 255, 15, 15, TFT_BLACK); // resetting the continuous
      lastpacketwasreceived = 0;
    }
  }
  else if (state == PW_CALLBACK_UPDATEIMAGE)
  {
    display.fillRect(117, 240, 15, 15, TFT_YELLOW); // UPDATEIMAGE COLOR
  }
  else if (state == PW_CALLBACK_DEFAULT)
  {
    display.setCursor(0, 50);
    display.setTextColor(TFT_WHITE);
    display.print("DEF;");
  }
  else if (state == PW_CALLBACK_INIT)
  {
    display.print("INIT;");
  }
  else if (state == PW_CALLBACK_RECEIVED)
  {
    display.fillRect(0, 240, 120, 40, TFT_BLACK);
    display.setCursor(0, 240);
    display.setTextColor(TFT_WHITE);
    display.setTextSize(1);
    display.printf("size:%d\n", value);
    delay(500);
  }

}

void setup()
{
  // display init
  SPI.begin(TFT_SCLK, TFT_MISO, TFT_MOSI, TFT_CS);
  display.init();
  display.fillScreen(TFT_BLACK);
  display.setCursor(0, 100);
  display.setTextSize(2);
  display.setTextColor(TFT_GREEN);
  display.println("Awaiting camera.");

  display.setTextColor(TFT_WHITE);
  display.setTextSize(1);

  // encoder, control stick init
  pinMode(ENC_LEFT, INPUT_PULLUP);
  pinMode(ENC_RIGHT, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(ENC_LEFT), _ENC_LEFT_INTERRUPT, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC_RIGHT), _ENC_RIGHT_INTERRUPT, CHANGE);

  pinMode(AxisX_Pin, INPUT);
  pinMode(AxisY_Pin, INPUT);

  pinMode(RED_LED_PIN, OUTPUT);
  pinMode(GREEN_LED_PIN, OUTPUT);

  digitalWrite(RED_LED_PIN, HIGH);
  digitalWrite(GREEN_LED_PIN, HIGH);
  delay(50);

  digitalWrite(RED_LED_PIN, LOW);
  digitalWrite(GREEN_LED_PIN, LOW);
  delay(450);

  digitalWrite(RED_LED_PIN, HIGH);
  digitalWrite(GREEN_LED_PIN, HIGH);
  delay(50);

  digitalWrite(RED_LED_PIN, LOW);
  digitalWrite(GREEN_LED_PIN, LOW);
  delay(450);

  //===PROM WIFI===
  prom_wifi_setDisplayCallback(displayUpdate);
  prom_wifi_setStateCallback(pw_callback);
  prom_wifi_sniffer_init();
}

unsigned long lastMillis = 0;
int framedeltatime = 1000 / display_FPS;

void loop()
{

  // HARDWARE
  //  encoder reading
  if (encmoved)
  {
    if (___ENC_CHECKTIME())
    {
      noInterrupts();
      counter += encmoved;
      if (counter < 0)
      {
        counter = 0;
      }
      if (counter > 100)
      {
        counter = 100;
      }
      encmoved = 0;
      if (update_display)
      {
        display.fillScreen(TFT_BLACK);
        display.setCursor(0, 0);
        display.print(counter);
      }

      interrupts();
    }
  }

  // DISPLAY
  //  display update
  if (millis() - lastMillis >= framedeltatime)
  {
    int rawReadX, rawReadY;
    int8_t pitch, roll;

    lastMillis = millis();
  }
}
