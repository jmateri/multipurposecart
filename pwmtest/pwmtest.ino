int analogPin1 = 2;
int analogPin2 = 3;// potentiometer connected to analog pin 3
int stopPin = 18;

int incomingByte1 = 0;
int incomingByte2 = 0;
long unsigned int elapsedTime;
volatile bool startMovement = true;

void setup()
{
  pinMode(analogPin1, OUTPUT);   // sets the pin as output
  pinMode(analogPin2, OUTPUT);

  pinMode(stopPin, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(stopPin), stopCart, RISING);
  
  analogWrite(analogPin1, 127);
  analogWrite(analogPin2, 127);
  Serial.begin(9600);
  //pinMode(LED_BUILTIN, OUTPUT);
}



void loop()
{
//  for(int i=0; i <= 255; i++){
//    analogWrite(analogPin1, i);
//    analogWrite(analogPin2, i);
//    delay(100);
//  }
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
      analogWrite(analogPin1, incomingByte2);
    }
    else if(incomingByte1 == 0 && startMovement)
    {
      analogWrite(analogPin2, incomingByte2);
    }   
    elapsedTime = millis(); 
   }
  }
  if(millis() - elapsedTime > 1000 && startMovement)
  {
    analogWrite(analogPin1, 130);
    analogWrite(analogPin2, 130);
    while(Serial.available() == 0);
  }
}

void stopCart()
{
  analogWrite(analogPin1, 127);
  analogWrite(analogPin2, 127);
 startMovement = !startMovement; 
}
