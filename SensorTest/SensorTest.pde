
void setup() {
 pinMode(0,INPUT);
 pinMode(2,OUTPUT);
 Serial.begin(9600); 
}

void loop()  {
 digitalWrite(2,HIGH);
 Serial.println(analogRead(0));
 delay(100);
}
