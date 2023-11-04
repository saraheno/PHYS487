/*
  AnalogReadSerial

  Reads an analog input on pin 0, prints the result to the Serial Monitor.
  Graphical representation is available using Serial Plotter (Tools > Serial Plotter menu).
  Attach the center pin of a potentiometer to pin A0, and the outside pins to +5V and ground.

  This example code is in the public domain.

  https://www.arduino.cc/en/Tutorial/BuiltInExamples/AnalogReadSerial
*/

// the setup routine runs once when you press reset:
int led_pin = 6;
float B = 3380.;
float r = 10000;
float v0=5.;
float r0=12000;
float t0=293;
float convert=273.;
float cycle = 200;
void setup() {
  // initialize serial communication at 9600 bits per second:
  Serial.begin(9600);
  pinMode(led_pin, OUTPUT);
}

// the loop routine runs over and over again forever:
void loop() {
  
  // read the input on analog pin 0:
  int sensorValue = analogRead(A0);
  // print out the value you read:
  Serial.write('\n');
  Serial.print("new run");
   Serial.write('\n');
  for(int i=0; i<60; i++){
    cycle=255*sin(3.14*i/60.);
    Serial.print("cycle is ");
    Serial.print(cycle);
    Serial.print("\n");
    analogWrite(led_pin, cycle);
    int sensorValue = analogRead(A0);
    float Vnow = sensorValue/256.;
    float RT = r*((v0/Vnow)-1);
    float b= (1/B)*log(RT/r0)+(1/t0);
    float tnow=1/b;
    Serial.print("i=");
    Serial.print(i);
    Serial.print(" voltage=");
    Serial.print(Vnow);
    Serial.print(", thermistor resistance=");
    Serial.print(RT);
    Serial.print(", temperature=");
    Serial.print(tnow-convert);
    Serial.print("\n");
    delay(2000);  // take a reading every minute
  }
  exit(0);
}
