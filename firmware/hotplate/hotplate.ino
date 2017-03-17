/**
 * Arduino Hotplate Controller
 *
 * Jon Sowman 2017
 */

#include "max6675.h"
#include <PID_v1.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

/* Solid state relay for heater attached to this pin */
#define RelayPin 10

/* OLED settings */
#define OLED_RESET 0
Adafruit_SSD1306 display(OLED_RESET);

/* Pin defs for the MAX6675 */
int thermoDO = 4;
int thermoCS = 5;
int thermoCLK = 6;
int vccPin = 3;
int gndPin = 2;

/* Create a thermocouple instance */
static MAX6675 thermocouple(thermoCLK, thermoCS, thermoDO);

/* PWM period for PID controller */
int WindowSize = 5000;

/* Time storage for PID */
unsigned long windowStartTime, last_tc_read;

/* Reference, measurement and PID output */
double Setpoint, Input, Output;

/* Create and configure a PID controller (or P in this case) */
static PID myPID(&Input, &Output, &Setpoint, 50, 0, 0, DIRECT);

void setup()
{
    Serial.begin(9600);
    // use Arduino pins 
    pinMode(vccPin, OUTPUT); digitalWrite(vccPin, HIGH);
    pinMode(gndPin, OUTPUT); digitalWrite(gndPin, LOW);
    pinMode(RelayPin, OUTPUT);

    Serial.println("HOTPLATE V0.1");
    // wait for MAX chip to stabilize
    delay(500);

    display.begin(SSD1306_SWITCHCAPVCC, 0x3C);  // initialize with the I2C addr 0x3D (for the 128x64)
    display.display();
    display.clearDisplay();

    windowStartTime = millis();

    //initialize the variables we're linked to
    Setpoint = 230.0F;

    //tell the PID to range between 0 and the full window size
    myPID.SetOutputLimits(0, WindowSize);

    //turn the PID on
    myPID.SetMode(AUTOMATIC);

    last_tc_read = millis();
}

void loop()
{
    /* MAX6675 can only be read at a max of like 5Hz */
    if( millis() - last_tc_read > 1000 )
    {
        Input = thermocouple.readCelsius();
        last_tc_read = millis();
        Serial.print("Current: ");
        Serial.print(Input);
        Serial.print("C (");

        /* Run PID */
        myPID.Compute();
        Serial.print((Output*100.0)/WindowSize);
        Serial.println("%)");
        /* OLED */
        display.clearDisplay();
        display.setTextSize(2);
        display.setTextColor(WHITE);
        display.setCursor(0,0);
        display.print("CURR: ");
        display.print(Input, 0);
        display.println("C");
        display.setCursor(0,16);
        display.print("TARG: ");
        display.print(Setpoint, 0);
        display.println("C");
        display.setCursor(0,32);
        display.print("PWM:  ");
        display.print((Output*100.0)/WindowSize, 0);
        display.println("%");
        display.display();
    }

    /* turn the output pin on/off based on pid output */
    if(millis() - windowStartTime>WindowSize)
    { //time to shift the Relay Window
        windowStartTime += WindowSize;
    }
    if(Output < millis() - windowStartTime)
        digitalWrite(RelayPin, LOW);
    else
        digitalWrite(RelayPin, HIGH);
}
