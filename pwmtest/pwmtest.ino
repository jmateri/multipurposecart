/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * Copyright 2017 Jonathan Materi and Nicholas DeNomme 
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 *
 * This program is to control the functions of the Arduino, mainly to
 * recieve and process data sent from the serial port and output the 
 * correct pulse-width modulation signal to the motor controllers. It 
 * also serves the function of receiving actions from the key fob to 
 * manually control the cart from a distance, and controls the neopixels.
 *
 * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 */

#include <Adafruit_NeoPixel.h>

//Analog PWM output pins for motor controllers
//Pins 8 and 9 are for one motor controller, 10 and 11 for the other
const short int analogPin8 = 8;
const short int analogPin9 = 9;
const short int analogPin10 = 10;
const short int analogPin11 = 11;

//Pins and declarations for the neo Pixels
const short int neoPixelPin1 = 12;
const short int neoPixelPin2 = 13;
const short int neoPixelCount = 8;
Adafruit_NeoPixel pixels1, pixels2;

//Pins for controlling the cart manually with the key fob
const short int stopPin = 18;
const short int manualForwardPin = 19;
const short int manualBackwardPin = 20;
const short int manualTurnPin = 21;

//Bytes received from the serial port
int incomingDirectionByte = 0;
int incomingSpeedByte = 0;

//Time in milliseconds that's passed since the program started
long unsigned int elapsedTime;

//True means the cart will move. False will mean it will stop
volatile bool startMovement = true;

//Function Declarations
void inline timeoutCart();
void inline outputStopCart();
void inline ledWrite(int pixelNumber, int red, int green, int blue);
void inline setAllPixels(int red, int green, int blue);
void pixelStartup(int red1, int green1, int blue1, int red2, int green2, int blue2);

//Used for neo pixel color for backwards movement. Will be yellow when both sides are moving backward
bool backwards1 = false, backwards2 = false;

void setup() {
  //Sets PWM pins as output pins
  pinMode(analogPin8, OUTPUT);
  pinMode(analogPin9, OUTPUT);
  pinMode(analogPin10, OUTPUT);
  pinMode(analogPin11, OUTPUT);

  //Sets pins for manually controlling the cart as interrupt pins
  pinMode(stopPin, INPUT_PULLUP);
  pinMode(manualForwardPin, INPUT_PULLUP);
  pinMode(manualBackwardPin, INPUT_PULLUP);
  pinMode(manualTurnPin, INPUT_PULLUP);

  //Attaches functions to interrupt pins to function is called when pin receives a rising edge
  attachInterrupt(digitalPinToInterrupt(stopPin), manualStopCart, RISING);
  attachInterrupt(digitalPinToInterrupt(manualForwardPin), manualForwardStart, RISING);
  attachInterrupt(digitalPinToInterrupt(manualBackwardPin), manualBackwardStart, RISING);
  attachInterrupt(digitalPinToInterrupt(manualTurnPin), manualTurnStart, RISING);

  //Stop cart at beginning
  outputStopCart();

  //Set serial port baud rate to 9600
  Serial.begin(9600);

  //Set up neo pixels
  pixels1 = Adafruit_NeoPixel(neoPixelCount, neoPixelPin1, NEO_GRB + NEO_KHZ800);
  pixels2 = Adafruit_NeoPixel(neoPixelCount, neoPixelPin2, NEO_GRB + NEO_KHZ800);
  pixels1.begin();
  pixels2.begin();

  //Start neo pixels with NDSU colors until movement begins
  pixelStartup(0, 255, 0, 255, 125, 0);
}

void loop() {
  //Check if data is available from serial port
  if (Serial.available() > 1) {
    //Neo pixels are green until both sides move backward, then they're yellow
    if ((backwards1 && backwards2) && startMovement) {
      setAllPixels(255, 125, 0);
    } else if (startMovement){
      setAllPixels(0, 255, 0);
    }

    //Read one byte of data to determine direction
    incomingDirectionByte = Serial.read();

    //Confirm that the byte signals a valid direction. 255 or 0 determines
    //the side of the cart that moves. (e.g. 255 is for the right wheels and 0 
    //is for the left wheels. It doesn't really matter which side is which in 
    //the program, since you can just flip the wiring if it's wrong. If it's 
    //not 255 or 0, it's not a direction byte and the program will ignore it
    if ((incomingDirectionByte == 255) || (incomingDirectionByte == 0)) {
      //Read byte for speed. Should be between 1 and 254, inclusive
      incomingSpeedByte = Serial.read();

      //If for some reason the byte is greater or less than the bounds, cap it at the max
      //to avoid unexpected behavior
      if (incomingSpeedByte > 254) {
        incomingSpeedByte = 254;
      }
      if (incomingSpeedByte < 1) {
        incomingSpeedByte = 1;
      }

      //If movement is true, move one side (left or right) of the cart's wheels
      if ((incomingDirectionByte == 255) && startMovement) {
        //Determines direction. Greater than 127 is forward, less than 127 is backward
        if (incomingSpeedByte >= 127) {
          backwards1 = false;

          //This is to normalize the speed for the motor controller. Outputing zero one
          //one pin and the speed on the other will cause the wheel to move. Flipping the
          //pins so that zero and speed are on the opposite will cause the cart to move the
          //other direction
          incomingSpeedByte -= 127;
          incomingSpeedByte <<= 1; //Multiply by 2 to normalize speed to get from zero to 255

          //Write speed and direction to the pins
          analogWrite(analogPin8, incomingSpeedByte);
          analogWrite(analogPin9, 0);
        } else {
          backwards1 = true;
          //Same as above but for the opposite direction
          incomingSpeedByte = 127 - incomingSpeedByte;
          incomingSpeedByte <<= 1; //Multiply by 2 to normalize

          //Write speed and direction to pins
          analogWrite(analogPin8, 0);
          analogWrite(analogPin9, incomingSpeedByte);
        }
      }
      //If movement is true, move other side of the cart's wheels
      else if ((incomingDirectionByte == 0) && startMovement) {
        if (incomingSpeedByte >= 127) {
          backwards2 = false;
          //This is to normalize the speed for the motor controller. Outputing zero one
          //one pin and the speed on the other will cause the wheel to move. Flipping the
          //pins so that zero and speed are on the opposite will cause the cart to move the
          //other direction
          incomingSpeedByte -= 127;
          incomingSpeedByte <<= 1; //Multiply by 2 to normalize

          //Write speed and direction to pins
          analogWrite(analogPin10, incomingSpeedByte);
          analogWrite(analogPin11, 0);
        } else {
          backwards2 = true;
          //Same as above but for the opposite direction
          incomingSpeedByte = 127 - incomingSpeedByte;
          incomingSpeedByte <<= 1; //Multiply by 2 to normalize

          //Write speed and direction to pins
          analogWrite(analogPin10, 0);
          analogWrite(analogPin11, incomingSpeedByte);
        }
      }
      //If byte is received, get ellapsed time since program has run in milliseconds
      elapsedTime = millis();
    }
  }

  //Timeout cart if no byte has been received after a certain amount of time
  timeoutCart();
}

//Stop cart from moving
void inline outputStopCart() {
  analogWrite(analogPin8, 0);
  analogWrite(analogPin9, 0);
  analogWrite(analogPin10, 0);
  analogWrite(analogPin11, 0);

  //Solid red on neo pixels for stop
  setAllPixels(255, 0, 0);
}

//Stop cart from moving if it doesn't receive data after a given amount of time
//This prevents the cart from unexpectedly running into walls and such
void inline timeoutCart() {
  //Get current time and subtract the time since the last byte was received.
  //If the time is greater than 333 milliseconds (1/3 second), stop the cart.
  if ((millis() - elapsedTime > 333) && startMovement) {
    outputStopCart();

    //Flash neo pixels red to indicate there's no visible tag
    pixelStartup(255, 0, 0, 255, 0, 0);
  }
}

//Stop cart if key fob stop button is pressed. This function is connected to
//an interrupt
void manualStopCart() {
  //Stop the cart
  outputStopCart();

  //Toggle each time the button is pressed to stop/start the cart
  startMovement = !startMovement;
}

//Move the cart forward manually if the forward button is pressed. This function is
//connected to an interrupt
void manualForwardStart() {
  //Change interrupt to change on falling edge, to stop cart after the forward button 
  //has been released
  detachInterrupt(digitalPinToInterrupt(manualForwardPin));
  attachInterrupt(digitalPinToInterrupt(manualForwardPin), interruptStop, FALLING);

  //Only go to manual mode if cart is stopped
  if (!startMovement) {
    //Set neo pixels blue for manual movement
    setAllPixels(0, 0, 255);

    //Move cart forward at a constant speed
    analogWrite(analogPin8, 80);
    analogWrite(analogPin9, 0);
    analogWrite(analogPin10, 80);
    analogWrite(analogPin11, 0);
  }
}

//Move the cart backward manually if the backward button is pressed. This function is
//connected to an interrupt
void manualBackwardStart() {
  //Change interrupt to change on falling edge, to stop cart after the backward button 
  //has been released
  detachInterrupt(digitalPinToInterrupt(manualBackwardPin));
  attachInterrupt(digitalPinToInterrupt(manualBackwardPin), interruptStop, FALLING);

  //Only go to manual mode if cart is stopped
  if (!startMovement) {
    //Set neo pixels blue for manual movement
    setAllPixels(0, 0, 255);

    //Move cart backward at a constant speed
    analogWrite(analogPin8, 0);
    analogWrite(analogPin9, 80);
    analogWrite(analogPin10, 0);
    analogWrite(analogPin11, 80);
  }
}

//Turn the cart manually if the turn button is pressed. This function is connected to
//an interrupt
void manualTurnStart() {
  //Change interrupt to change on falling edge, to stop cart after the turn button 
  //has been released
  detachInterrupt(digitalPinToInterrupt(manualTurnPin));
  attachInterrupt(digitalPinToInterrupt(manualTurnPin), interruptStop, FALLING);

  //Only go to manual mode if cart is stopped
  if (!startMovement) {
    //Set neo pixels blue for manual movement
    setAllPixels(0, 0, 255);

    //Turn at a constant speed
    analogWrite(analogPin8, 100);
    analogWrite(analogPin9, 0);
    analogWrite(analogPin10, 0);
    analogWrite(analogPin11, 100);
  }
}

//Stop cart upon manual button being released
void interruptStop() {
  //Reset all inerrupts to activate on a rising edge to detect if a button is pressed again
  detachInterrupt(digitalPinToInterrupt(manualForwardPin));
  attachInterrupt(digitalPinToInterrupt(manualForwardPin), manualForwardStart, RISING);
  attachInterrupt(digitalPinToInterrupt(manualBackwardPin), manualBackwardStart, RISING);
  attachInterrupt(digitalPinToInterrupt(manualTurnPin), manualTurnStart, RISING);

  //Stop cart
  outputStopCart();
}

//Writes individual LED on the strip the given RGB color
void inline ledWrite(int pixelNumber, int red, int green, int blue) {
  pixels1.setPixelColor(pixelNumber, pixels1.Color(red, green, blue));
  pixels2.setPixelColor(pixelNumber, pixels2.Color(red, green, blue));
  pixels1.show();
  pixels2.show();
}

//Writes all LEDs on the strips the given RGB color
void inline setAllPixels(int red, int green, int blue) {
  for (int i = 0; i < neoPixelCount; i++) {
    pixels1.setPixelColor(i, pixels1.Color(red, green, blue));
    pixels2.setPixelColor(i, pixels2.Color(red, green, blue));
  }
  pixels1.show();
  pixels2.show();
}

//Flashes pixels back and forth between two colors. Doesn't resume until serial data
//is available, or the stop button on the key fob is pressed.
void pixelStartup(int red1, int green1, int blue1, int red2, int green2, int blue2) {
  for (int i = 0; (Serial.available() == 0) && startMovement; i++) {
    for (int j = 0; j < neoPixelCount; j++) {
      if (i % 2 == 0) {
        if (j % 2 == 0) {
          ledWrite(j, red1, green1, blue1);
        } else {
          ledWrite(j, 0, 0, 0);
        }
      } else {
        if (j % 2 == 1) {
          ledWrite(j, red2, green2, blue2);
        } else {
          ledWrite(j, 0, 0, 0);
        }
      }
    }
    delay(500);
  }
}
