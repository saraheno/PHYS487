void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
}

int i=0;
void loop() {
  // put your main code here, to run repeatedly:
Serial.print(sin(i*50.0/360.0));
//Serial.print(" ");
//Serial.print(i);
  Serial.write(13);
  Serial.write(10);
  i += 1;
  delay(100);
  //if(i>100) i=0;
  //exit(0);
}
