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

//debugging stuff
int iyuck = 0;

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


  analogWrite(outputPin, 0);  // start with highest duty cycle and get readings
  Serial.println(" ");
  Serial.println(" ");
  Serial.println("doing calibration for all high pwm 0");
  doRead(1, voltageOne, voltageTwo, voltageDifference, current, terminalVoltage, voltageError, currentError);
  voltageTargetmin = terminalVoltage;

  analogWrite(outputPin, 255);  // then lowest
  Serial.println(" ");
  Serial.println(" ");
  Serial.println("doing calibration for all low  pwm 255");
  doRead(1, voltageOne, voltageTwo, voltageDifference, current, terminalVoltage, voltageError, currentError);
  voltageTargetmax = terminalVoltage;

  Serial.println(" ");
  Serial.print("note that goal voltage has to be between ");
  Serial.print(voltageTargetmin);
  Serial.print(" and ");
  Serial.println(voltageTargetmax);




  if ((targetVoltage < voltageTargetmin) || (targetVoltage > voltageTargetmax)) {
    Serial.print("illegal target voltage ");
    Serial.println(targetVoltage);
    Serial.print("setting target voltage to ");
    targetVoltage = (voltageTargetmax - voltageTargetmin) / 2 + voltageTargetmin;
    Serial.println(targetVoltage);
  }

  for (int i = 0; i < ncalib; i = i + 1) {
    analogWrite(outputPin, i * calibspace);  // start with highest duty cycle and get readings
    Serial.println(" ");
    Serial.println(" ");
    Serial.print("doing calibration for pwm ");
    Serial.println(i * calibspace);
    doRead(1, voltageOne, voltageTwo, voltageDifference, current, terminalVoltage, voltageError, currentError);
    calibpoints[i] = terminalVoltage;
  }
  // calculate calibration constants

  // calculate target pwm
  int iii = 0;
  Serial.println("");
  Serial.print("looking for calib point for targetVoltage ");
  Serial.println(targetVoltage);
  while (calibpoints[iii] < targetVoltage) {
    iii += 1;
    Serial.print(" iii is now ");
    Serial.println(iii);
  }
  if ((iii < 1) || (iii > 254)) {
    Serial.println("illegal calib point");
    iii = 1;
    Serial.print("Setting calibpoint to");
    Serial.println(iii);
  }
  Serial.print("closest calib point is point ");
  Serial.print(iii);
  Serial.print(" corresponding to pwm of ");
  Serial.print(iii * calibspace);
  Serial.print(" and voltage of ");
  Serial.println(calibpoints[iii]);

  calib_m = (calibpoints[iii] - calibpoints[iii - 1]);
  calib_b = calibpoints[iii] - calib_m * iii;
  Serial.print(" slope is ");
  Serial.println(calib_m);
  Serial.print(" intercept is ");
  Serial.println(calib_b);

  outputValue = calibspace * (targetVoltage - calib_b) / calib_m;
  Serial.println("");
  Serial.print("setting outputValue initally to ");
  Serial.print(outputValue);
  Serial.print(" corresponding to index ");
  float aaaaa = float(outputValue) / float(calibspace);
  Serial.println(aaaaa);
  analogWrite(outputPin, outputValue);
  Serial.println("");
  Serial.println("readings with this value are ");
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


  if (iyuck < 2000) {

    doRead(1, voltageOne, voltageTwo, voltageDifference, current, terminalVoltage, voltageError, currentError);

    modeState = digitalRead(modeSwitch);           //Check switch to determine mode
    upButtonState = digitalRead(upButtonPin);      //Check buttons for manual adjustments
    downButtonState = digitalRead(downButtonPin);  //Check buttons for manual adjustment



    if (modeState == HIGH)  //if HIGH, circuit is in Voltage Output Mode
    {


      targetCurrent = (int)current;  //sets target value to current value to avoid problems when switching modes
      Serial.print("switching target current to ");
      Serial.println(targetCurrent);

      if (previousTargetVoltage != targetVoltage || previousOutputValue != outputValue)  //dispay serial data
      {
        Serial.println("need to update");
        Serial.println("   Voltage Mode");  //display measured values
        Serial.print("   Output Value: ");
        Serial.println(outputValue);
        //Serial.print("Voltage Point 1: ");
        //Serial.println(voltageOne);
        //Serial.print("Voltage Point 2: ");
        //Serial.println(voltageTwo);
        //Serial.print("Voltage Difference: ");
        //Serial.println(voltageDifference);
        Serial.print("   Target Voltage: ");
        Serial.println(targetVoltage);
        Serial.print("   Terminal Voltage: ");
        Serial.println(terminalVoltage);
        Serial.print("   Voltage Error: ");
        Serial.println(voltageError);
        Serial.print("   Current (mA): ");
        Serial.println(current);
        Serial.println();
        Serial.println();
        Serial.println();
      }

      previousOutputValue = outputValue;
      previousTargetVoltage = targetVoltage;

      if (upButtonState == LOW)  //if up adjustment button is pressed
      {
        Serial.println("UPBUTTON is on");
        targetVoltage = targetVoltage + 0.05;  //Adjust target voltage setting up
        if (targetVoltage > 5) {
          targetVoltage = 5;  //maximum target value
        }
        delay(200);  //delay to avoid errors from switch bounce
      }

      if (downButtonState == LOW)  //if down adjustment button is pressed
      {
        Serial.println("DOWNBUTTON is on");
        targetVoltage = targetVoltage - 0.05;  //Adjust target voltage setting down
        if (targetVoltage < 0) {
          targetVoltage = 0;  //minumum value of 0
        }
        delay(200);  //delay to avoid errors from switch bounce
      }

      if (current < 150)  //if current is safe
      {
        if (abs(voltageError) > 0.04)  //if output error is large enough adjust output
        {
          int ddd = -1;
          if (voltageError > 0) ddd *= -1;
          outputValue = outputValue + ddd;
          //outputValue=255;
          if (outputValue < 0) {
            Serial.print("illegal output value ");
            Serial.println(outputValue);
            Serial.println("setting to zero");
            outputValue = 0;
          }
          if (outputValue > 255) {
            Serial.print("illegal output value ");
            Serial.println(outputValue);
            Serial.println("setting to 255");
            outputValue = 255;
          }
          Serial.print("CHANGing target to ");
          Serial.println(outputValue);
          analogWrite(outputPin, outputValue);  //write new value
          delay(100);                           //delay to allow the output to adjust
        }
      } else {
        outputValue = outputValue - 1;  //lower current until it is below 150mA to protect the Arduino
      }
    }

    if (modeState == LOW)  //if LOW, circuit is in Current Output Mode
    {
      targetVoltage = terminalVoltage;  //sets target value to current value to avoid problems when switching modes
      Serial.println("switching target voltage to ");
      Serial.println(targetVoltage);

      if (previousTargetCurrent != targetCurrent || previousOutputValue != outputValue)  //dispay serial data
      {
        Serial.println("Current Mode");  //display measured values
        Serial.print("Output Value: ");
        Serial.println(outputValue);
        Serial.print("Target Current (mA): ");
        Serial.println(targetCurrent);
        Serial.print("Current (mA): ");
        Serial.println(current);
        Serial.print("Current Error  (mA): ");
        Serial.println(currentError);
        Serial.print("Terminal Voltage: ");
        Serial.println(terminalVoltage);
        Serial.println();
        Serial.println();
        Serial.println();
      }

      previousOutputValue = outputValue;
      previousTargetCurrent = targetCurrent;



      if (upButtonState == LOW)  //if up adjustment button is pressed
      {
        targetCurrent = targetCurrent + 5;  //Adjust target voltage setting up
        if (targetCurrent > 150)            //limit current to 150mA to protect the Arduino
        {
          targetCurrent = 150;
        }
        delay(200);  //delay to avoid errors from switch bounce
      }

      if (downButtonState == LOW)  //if down adjustment button is pressed
      {
        targetCurrent = targetCurrent - 5;  //Adjust target voltage setting down
        if (targetCurrent < 0) {
          targetCurrent = 0;
        }
        delay(200);  //delay to avoid errors from switch bounce
      }



      if (abs(currentError) > 5)  //if output error is large enough adjust output
      {
        outputValue = outputValue + currentError / 5;
        analogWrite(outputPin, outputValue);  //write new value
        delay(100);                           //delay to allow the output to adjust
      }
    }

    if (outputValue < 1)  //output can never go below 0
    {
      outputValue = 0;
    }

    if (outputValue > 250)  //output can never go above 255
    {
      outputValue = 250;
    }


    iyuck += 1;
   // Serial.print("changing iyuck to ");
    //Serial.println(iyuck);
  }
}
