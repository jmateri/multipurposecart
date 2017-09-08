int analogPin = 2;   // potentiometer connected to analog pin 3
int incomingByte = 0;


void setup()
{
  pinMode(analogPin, OUTPUT);   // sets the pin as output
  
  analogWrite(analogPin, 127);
  Serial.begin(9600);
  //pinMode(LED_BUILTIN, OUTPUT);
}



void loop()
{
  if (Serial.available() > 0)
  {
    //analogWrite(analogPin, 127);
    // read the incoming byte:
    incomingByte = Serial.read();
    Serial.flush();
//    digitalWrite(LED_BUILTIN, HIGH);   // turn the LED on (HIGH is the voltage level)
//    delay(1000);                       // wait for a second
//    digitalWrite(LED_BUILTIN, LOW);    // turn the LED off by making the voltage LOW
//    delay(1000);  
    // say what you got:
    //    if (incomingByte == 64)
    //    {
    //}
    //analogWrite(analogPin, incomingByte % 255);
//    Serial.print("I received: ");
//    Serial.println(incomingByte, DEC);
    if(incomingByte > 255)
    {
      incomingByte = 255;
    }
    if(incomingByte < 0)
    {
      incomingByte = 0;
    }
    
    analogWrite(analogPin, incomingByte);

  }
   

}
