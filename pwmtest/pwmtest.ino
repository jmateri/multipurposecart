int analogPin1 = 2;
int analogPin2 = 3;// potentiometer connected to analog pin 3
int incomingByte1 = 0;
int incomingByte2 = 0;
long unsigned int elapsedTime;


void setup()
{
  pinMode(analogPin1, OUTPUT);   // sets the pin as output
  pinMode(analogPin2, OUTPUT);
  
  analogWrite(analogPin1, 125);
  analogWrite(analogPin2, 125);
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
  
    if(incomingByte1 == 255)
    {
      analogWrite(analogPin1, incomingByte2);
    }
    else if(incomingByte1 == 0)
    {
      analogWrite(analogPin2, incomingByte2);
    }   
    elapsedTime = millis(); 
   }
  }
  if(millis() - elapsedTime > 1000)
  {
    analogWrite(analogPin1, 125);
    analogWrite(analogPin2, 125);
    while(Serial.available() == 0);
  }
}
