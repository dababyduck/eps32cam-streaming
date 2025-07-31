#ifndef PINOUT_H
#define PINOUT_h

// DISPLAY PINS
#define TFT_CS 5    // Chip select control pin
#define TFT_RST 10  // Reset pin (could connect to RST pin)
#define TFT_DC 16   // Data Command control pin
#define TFT_SCLK 11 // Clock pin
#define TFT_MOSI 12 // Data out pin
#define TFT_MISO -1
#define TOUCH_CS -1

// ENCODER PINS
#define ENC_LEFT 13
#define ENC_RIGHT 14


// CONTROL STICK PINS
#define AxisX_Pin 1
#define AxisY_Pin 2

//TEST
#define GREEN_LED_PIN 35
#define RED_LED_PIN 37

#endif
