/*
 * Copyright 2017 Jonathan Materi & Nick DeNomme 
 */

const int analogPin2 = 2;
const int analogPin3 = 3;
const int analogPin7 = 7;
const int analogPin8 = 8;
const int stopPin = 18;
const int manualForwardPin = 19;
const int manualBackwardPin = 20;
const int manualTurnPin = 21;

int incomingByte1 = 0;
int incomingByte2 = 0;
long unsigned int elapsedTime;
volatile bool startMovement = true;

void inline timeoutCart();
void inline outputStopCart();

void setup()
{
  pinMode(analogPin2, OUTPUT);   // sets the pin as output
  pinMode(analogPin3, OUTPUT);
  pinMode(analogPin7, OUTPUT);   // sets the pin as output
  pinMode(analogPin8, OUTPUT);

  pinMode(stopPin, INPUT_PULLUP);
  pinMode(manualForwardPin, INPUT_PULLUP);
  pinMode(manualBackwardPin, INPUT_PULLUP);
  pinMode(manualTurnPin, INPUT_PULLUP);
  
  attachInterrupt(digitalPinToInterrupt(stopPin), stopCart, RISING);
  attachInterrupt(digitalPinToInterrupt(manualForwardPin), manualForwardStart, RISING);
  attachInterrupt(digitalPinToInterrupt(manualBackwardPin), manualBackwardStart, RISING);
  attachInterrupt(digitalPinToInterrupt(manualTurnPin), manualTurnStart, RISING);
  
  outputStopCart();
  
  Serial.begin(9600);
}



void loop()
{
  if (Serial.available() > 1)
  {
    incomingByte1 = Serial.read();
    
    if(incomingByte1 == 255 || incomingByte1 == 0)
    {
     incomingByte2 = Serial.read();
     if(incomingByte2 > 254)
     {
       incomingByte2 = 254;
     }
     if(incomingByte2 < 1)
     {
       incomingByte2 = 1;
     }
  
    if(incomingByte1 == 255 && startMovement)
    {
      if(incomingByte2 >= 127)
      {
        incomingByte2 -= 127;
        incomingByte2 <<= 1;
        analogWrite(analogPin3, 0);
        analogWrite(analogPin2, incomingByte2);
        
      }
      else
      {
        incomingByte2 = 127 - incomingByte2;
        incomingByte2 <<= 1;
        analogWrite(analogPin2, 0);
        analogWrite(analogPin3, incomingByte2);
      }
    }
    else if(incomingByte1 == 0 && startMovement)
    {
      if(incomingByte2 >= 127)
      {
        incomingByte2 -= 127;
        incomingByte2 <<= 1;
        analogWrite(analogPin7, incomingByte2);
        analogWrite(analogPin8, 0);
      }
      else
      {
        incomingByte2 = 127 - incomingByte2;
        incomingByte2 <<= 1;
        analogWrite(analogPin7, 0);
        analogWrite(analogPin8, incomingByte2);
      }
    }   
    elapsedTime = millis(); 
   }
  }
  timeoutCart();
}

void inline outputStopCart()
{
  analogWrite(analogPin2, 0);
  analogWrite(analogPin3, 0);
  analogWrite(analogPin7, 0);
  analogWrite(analogPin8, 0);
}

void inline timeoutCart()
{
  if((millis() - elapsedTime > 333) && startMovement)
  {
    outputStopCart();
    while(Serial.available() == 0);
  }
}

void stopCart()
{
  digitalWrite(LED_BUILTIN, HIGH);   // turn the LED on (HIGH is the voltage level)
  delay(1000);                       // wait for a second
  digitalWrite(LED_BUILTIN, LOW);    // turn the LED off by making the voltage LOW
  delay(1000);  
 outputStopCart();
 startMovement = !startMovement; 
}

void manualForwardStart()
{
  detachInterrupt(digitalPinToInterrupt(manualForwardPin));
  attachInterrupt(digitalPinToInterrupt(manualForwardPin), interruptStop, FALLING);
  if(!startMovement)
  {
    analogWrite(analogPin2, 75);
    analogWrite(analogPin3, 0);
    analogWrite(analogPin7, 75);
    analogWrite(analogPin8, 0);
  }
}

void manualBackwardStart()
{
  detachInterrupt(digitalPinToInterrupt(manualBackwardPin));
  attachInterrupt(digitalPinToInterrupt(manualBackwardPin), interruptStop, FALLING);
  if(!startMovement)
  {
    analogWrite(analogPin2, 0);
    analogWrite(analogPin3, 100);
    analogWrite(analogPin7, 0);
    analogWrite(analogPin8, 100);
  }
}

void manualTurnStart()
{
  detachInterrupt(digitalPinToInterrupt(manualTurnPin));
  attachInterrupt(digitalPinToInterrupt(manualTurnPin), interruptStop, FALLING);
  if(!startMovement)
  {
    analogWrite(analogPin2, 75);
    analogWrite(analogPin3, 0);
    analogWrite(analogPin7, 0);
    analogWrite(analogPin8, 75);
  } 
}

void interruptStop()
{
  detachInterrupt(digitalPinToInterrupt(manualForwardPin));
  attachInterrupt(digitalPinToInterrupt(manualForwardPin), manualForwardStart, RISING);
  attachInterrupt(digitalPinToInterrupt(manualBackwardPin), manualBackwardStart, RISING);
  attachInterrupt(digitalPinToInterrupt(manualTurnPin), manualTurnStart, RISING);
  outputStopCart();
}

