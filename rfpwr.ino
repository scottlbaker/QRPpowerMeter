
// ============================================================================
//  rfpwr.ino :: Dummy-load and RF power meter
//
//  Copyright 2021  Scott Baker KJ7NLA
// ============================================================================

#define VERSION   "1.01a"
#define BUTTON   12      // PB4/D12    (Nano pin 15)
#define VP0      14      // PC0/ADC0   (Nano pin 19)
#define SDA      18      // PC4/SDA    (Nano pin 23)
#define SCL      19      // PC5/SCL    (Nano pin 24)

#include <Wire.h>
#include "SSD1306Ascii.h"
#include "SSD1306AsciiWire.h"

#define I2C_ADDRESS 0x3C

// user interface macros
#define UIKEY !digitalRead(BUTTON)

// resistor divider values
#define R1 249000.0
#define R2 33000.0

// diode voltage drop
#define DVD .20

// correction factor for AVCC = 5.0V
// use this when using regulated power
// #define VCF 209715

// correction factor for AVCC = 4.8V
// use this when using USB power
#define VCF ((209715 * 5.0) / 4.8);

void  initpins();
void  adc_init();
uint16_t adc_read();
void  mydelay(uint16_t dly);
void  show_version();
void  clearline(int x);
void  printV(char* msg, float fpv, char c);
float get_samples();
float calc_vin(float vai);
void  calc_power();
void  check_button();

SSD1306AsciiWire oled;

// initialize I/O pins
void initpins() {
  pinMode(VP0, INPUT);            // ADC0 input
  pinMode(BUTTON, INPUT_PULLUP);  // UI button
}

// ADC init
void adc_init() {
  ADCSRA = 0x87;         // adc_clock = cpu_clock/128
  ADCSRB = 0;            // clear ADCSRB register
  ADMUX  = 0x40;         // use AVCC (5V) reference
}

// ADC read
uint16_t adc_read() {
  noInterrupts();
  // start a conversion
  ADCSRA |= (1 << ADSC);
  // wait for the ADC conversion to complete
  while (ADCSRA & (1 << ADSC));
  // get ADC result
  uint16_t v = ADC;
  interrupts();
  return v;
}

// delay with button check
void mydelay(uint16_t dly) {
  uint32_t endtime;
  endtime = millis() + dly;
  while (millis() < endtime) {
    check_button();
    delay(10);
  }
}

// show version
void show_version() {
  oled.clear();
  oled.setCursor(0,0);
  oled.print("RF Power");
  oled.setCursor(0,2);
  oled.print("Meter");
  oled.setCursor(0,4);
  oled.print("Rev ");
  oled.print(VERSION);
}

// clear a text line
void clearline(int x) {
  oled.setCursor(0,x);
  oled.print("                ");
  oled.setCursor(0,x);
}

// print a float value
void printV(char* msg, float fpv, char c) {
  char val[10];
  oled.print(msg);
  dtostrf(fpv, 6, 2, val);
  oled.print(val);
  oled.print(c);
}

// get samples and average
float get_samples() {
  float acc = 0.0;
  uint16_t i;
  for (i=0; i<1024; i++) {
    acc += (float)adc_read();
  }
  acc = acc / VCF;   // correction factor
  return acc;
}

// calculate the input voltage
float calc_vin(float vai) {
  float idv;     // divider current
  float vin;     // calculated input voltage
  idv = vai/R2;  // divider current
  vin = vai + (idv * R1);
  vin += DVD;    // add diode drop
  vin = vin * 2.0;
  return vin;
}

#define STOPPED  0  // stopped
#define RUNNING  1  // run/capture mode
#define SPLASH   2  // initial state

volatile uint8_t state = SPLASH;

// calculate power
void calc_power() {
  float vai;     // average voltage at ADC input
  float vin;     // calculated input voltage
  float pwr;     // calculated power
  vai = get_samples();
  vin = calc_vin(vai);
  pwr = (vin * vin) / 50.0;
  clearline(2);
  printV("ADC", vai, 'V');
  clearline(4);
  printV("VIN", vin, 'V');
  clearline(6);
  printV("PWR", pwr, 'W');
  mydelay(2000);
}

#define LONGPRESS  500

#define NBP  0  // no-button-pushed
#define BSC  1  // button-single-click
#define BPL  2  // button-push-long

// check UI button
void check_button() {
  uint8_t event = NBP;
  if (UIKEY) {
    event = BSC;
    // check for long button press
    uint32_t t0 = millis();
    while (UIKEY && (event != BPL)) {
      if (millis() > (t0 + LONGPRESS)) event = BPL;
      delay(100);  // debounce
    }
    switch (event) {

      case BSC:  // button single click
        // check the state
        switch (state) {
          case SPLASH:
            oled.clear();
            oled.setCursor(0,0);
            oled.print(">> running");
            state = RUNNING;
            break;
          case STOPPED:
            clearline(0);
            oled.print(">> running");
            state = RUNNING;
            break;
          case RUNNING:
            clearline(0);
            oled.print(">> stopped");
            state = STOPPED;
            break;
          default:
            break;
        }
        break;

      case BPL:  // button press long
        state = SPLASH;
        show_version();
        break;

      default:  // not possible
        break;
    }
    // wait for button release
    while (UIKEY) {
      delay(100);  // debounce
    }
  }
}

// initial setup
void setup() {
  initpins();
  Wire.begin();
  Wire.setClock(400000L);
  oled.begin(&Adafruit128x64, I2C_ADDRESS);
  oled.setFont(fixed_bold10x15);
  adc_init();
  show_version();
}

// main loop
void loop() {
  check_button();
  if (state == RUNNING) {
    calc_power();
  }
}

