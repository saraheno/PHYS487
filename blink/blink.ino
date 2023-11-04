int led_pin=6;
int nblink=3;
int iblink=0;
void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  pinMode(led_pin,OUTPUT);
  iblink=0;
}

void loop() {
  // put your main code here, to run repeatedly:
  digitalWrite(led_pin,HIGH);
  delay(1000);
  digitalWrite(led_pin,LOW);
  delay(1000);
  iblink+=1;
  Serial.print("iblink=");
  Serial.print(iblink);
  Serial.print("\n");
  if(iblink>nblink) exit(0);
}
