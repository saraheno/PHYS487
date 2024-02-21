float resistance = 10.6;  //measured resistance of the power resistor.

float supplyVoltage = 5;  //voltage of the powre source, in this case a regulated 5V from a USB port

int modeSwitch = 12;      //pin to read output mode (voltage/current)
int modeState = 0;        //mode (1 = Target Voltage, 0 = Target Current)
int upButtonPin = 11;     //Pin to read positive manual adjustment buttons
int upButtonState = 0;    //Current state of positive adjustment button
int downButtonPin = 10;   //Pin to read negative manual adjustment buttons
int downButtonState = 0;  //Current state of negative adjustment button
int outputPin = 9;        // power supply connected to digital pin 9
int analogPinOne = 1;     //first voltage probe connected to analog pin 1
int analogPinTwo = 2;     //second voltage probe connected to analog pin 2


float voltageOne = 0;  //calculated voltage at analogPinOne


float voltageTwo = 0;  //calculated voltage at analogPinTwo

float voltageDifference = 0;  //difference in voltage between analogPinOne and analogPinTwo
float terminalVoltage = 0;
float current = 0;  //calculated current through the load

// SET TARGETS HERE
float targetVoltage = 2.5;        //set output voltage
float previousTargetVoltage = 0;  //previous output voltage
float targetCurrent = 0;          //set output current
float previousTargetCurrent = 0;  //previous output current
float voltageError = 0;           //difference between target voltage and actual voltage
float currentError = 0;           //difference between target current and actual current
int outputValue = 0;              //set PWM value
int previousOutputValue = 0;      //set PWM value

// calibration stuff
float voltageTargetmin;
float voltageTargetmax;
const int ncalib = 20;
int calibspace = 10;
float calibpoints[ncalib];
float calib_m;
float calib_b;



void setup() {
  Serial.begin(9600);  //  setup serial
  Serial.println("hi there");
  pinMode(outputPin, OUTPUT);     // sets the pin as output
  pinMode(upButtonPin, INPUT);    // sets the pin as input
  pinMode(downButtonPin, INPUT);  // sets the pin as input
  pinMode(modeSwitch, INPUT);     // sets the pin as input
  modeState = digitalRead(modeSwitch);
  Serial.print("mode is ");
  if (modeState == HIGH) {
    Serial.println("voltage source");
    Serial.print("target voltage is ");
    Serial.println(targetVoltage);
  }
  if (modeState == LOW) {
    Serial.println("current source");
    Serial.println("target current is");
    Serial.println(targetCurrent);
  }

  //calibrate


  doRead(1, voltageOne, voltageTwo, voltageDifference, current, terminalVoltage, voltageError, currentError);
}



void doRead(int talk, float &voltageOne, float &voltageTwo, float &voltageDifference, float &current, float &terminalVoltage, float &voltageError, float &currentError) {
  delay(1000);
  float valueProbeOneTotal = 0;
  float valueProbeTwoTotal = 0;

  for (int i = 0; i <= 49; i++)  //measure voltage at probes to determine voltage and current of the output
  {
    float valueProbeOne = analogRead(analogPinOne);           //read the input pin
    float valueProbeTwo = analogRead(analogPinTwo);           //read the input pin
    valueProbeOneTotal = valueProbeOneTotal + valueProbeOne;  //sums value from analogPinOne 50 times
    valueProbeTwoTotal = valueProbeTwoTotal + valueProbeTwo;  //sums value from analogPinTwo 50 times
  }

  float valueProbeOneAverage = valueProbeOneTotal / 50;  //divides by 50 to get average value
  float valueProbeTwoAverage = valueProbeTwoTotal / 50;  //divides by 50 to get average value


  voltageOne = valueProbeOneAverage * supplyVoltage / 1023;  //converts analog input reading into voltage
  voltageTwo = valueProbeTwoAverage * supplyVoltage / 1023;  //converts analog input reading into voltage
  voltageDifference = voltageOne - voltageTwo;               //difference in voltage between the two probes
  current = voltageDifference * 1000 / resistance;           //divide by the value of the reference resistor to get the current in mA
  terminalVoltage = supplyVoltage - voltageOne;              // voltage across the load
  voltageError = targetVoltage - terminalVoltage;            //difference between target voltage and measured voltage
  currentError = targetCurrent - current;                    //difference between target current and measured current


  if (talk > 0) {

    Serial.print("voltage across load is ");
    Serial.println(terminalVoltage);
    Serial.print("current is ");
    Serial.println(current);

    if (talk > 1) {
      Serial.print("voltageOne is ");
      Serial.println(voltageOne);

      Serial.print("voltageTwo is ");
      Serial.println(voltageTwo);
      Serial.print("voltageDifference is ");
      Serial.println(voltageDifference);
      Serial.print("voltageError is ");
      Serial.println(voltageError);
      Serial.print("modeState is ");
      Serial.println(modeState);
      Serial.print("upButtonState ");
      Serial.println(upButtonState);
      Serial.print("downButtonState ");
      Serial.println(downButtonState);
      Serial.print("output value is ");
      Serial.println(outputValue);
    }
  }
}

void loop() {
  if (millis() < 1000)  // millis is an ardunio function that returns the number of ms since the start of the current program
  {
    delay(1000);  // delay to allow output to reset after startup
  }


}
