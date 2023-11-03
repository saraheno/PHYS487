int resistance = 10.6;     //measured resistance of the power resistor
float supplyVoltage = 5;     //voltage of the powre source, in this case a regulated 5V from a USB port

int modeSwitch = 12;     //pin to read output mode (voltage/current)
int modeState = 0;     //mode (1 = Target Voltage, 0 = Target Current)
int upButtonPin = 11;     //Pin to read positive manual adjustment buttons
int upButtonState = 0;     //Current state of positive adjustment button
int downButtonPin = 10;     //Pin to read negative manual adjustment buttons
int downButtonState = 0;     //Current state of negative adjustment button  
int outputPin = 9;     // power supply connected to digital pin 9
int analogPinOne = 1;     //first voltage probe connected to analog pin 1
int analogPinTwo = 2;     //second voltage probe connected to analog pin 2

int valueProbeOne = 0;     //variable to store the value of analogPinOne
float valueProbeOneTotal = 0;     //sum of 1000 measurements from analogPinOne
float valueProbeOneAverage = 0;     //average of measurements from analogPinOne 
float voltageOne = 0;     //calculated voltage at analogPinOne

int valueProbeTwo = 0;     //variable to store the value of analogPinTwo
float valueProbeTwoTotal = 0;     //sum of 1000 measurements from analogPinTwo
float valueProbeTwoAverage = 0;     //average of measurements from analogPinTwo
float voltageTwo = 0;     //calculated voltage at analogPinTwo

float voltageDifference = 0;     //difference in voltage between analogPinOne and analogPinTwo
float terminalVoltage = 0;
float current = 0;     //calculated current through the load

float targetVoltage = 0;     //set output voltage
float previousTargetVoltage = 0;     //previous output voltage
float targetCurrent = 0;     //set output current
float previousTargetCurrent = 0;     //previous output current
float voltageError = 0;     //difference between target voltage and actual voltage
float currentError = 0;     //difference between target current and actual current
int outputValue = 0;     //set PWM value 
int previousOutputValue = 0;     //set PWM value 


void setup()
{
  Serial.begin(9600);     //  setup serial
  pinMode(outputPin, OUTPUT);     // sets the pin as output
  pinMode(upButtonPin, INPUT);     // sets the pin as input
  pinMode(downButtonPin, INPUT);     // sets the pin as input
  pinMode(modeSwitch, INPUT);     // sets the pin as input
}



void loop()
{
if(millis() < 1000)
 {
  delay(1000);     // delay to allow output to reset after startup  
 }

for (int i=0; i <= 49; i++)     //measure voltage at probes to determine voltage and current of the output
 {
  valueProbeOne = analogRead(analogPinOne);     //read the input pin
  valueProbeTwo = analogRead(analogPinTwo);     //read the input pin
  valueProbeOneTotal = valueProbeOneTotal + valueProbeOne;     //sums value from analogPinOne 1000 times
  valueProbeTwoTotal = valueProbeTwoTotal + valueProbeTwo;     //sums value from analogPinTwo 1000 times
 } 

valueProbeOneAverage = valueProbeOneTotal / 50;     //divides by 1000 to get average value
valueProbeTwoAverage = valueProbeTwoTotal / 50;     //divides by 1000 to get average value
valueProbeOneTotal = 0;     //resets total value
valueProbeTwoTotal = 0;     //resets total value  

voltageOne = valueProbeOneAverage * 5 /1023;     //converts analog input reading into voltage
voltageTwo = valueProbeTwoAverage * 5 /1023;     //converts analog input reading into voltage
voltageDifference = voltageOne - voltageTwo;     //difference in voltage between the two probes
current = voltageDifference * 1000 / resistance;     //divide by the value of the reference resistor to get the current
terminalVoltage = supplyVoltage - voltageOne;
voltageError = targetVoltage - terminalVoltage;     //difference between target voltage and measured voltage
currentError = targetCurrent - current;     //difference between target current and measured current
  
modeState = digitalRead(modeSwitch);     //Check switch to determine mode
upButtonState = digitalRead(upButtonPin);     //Check buttons for manual adjustments
downButtonState = digitalRead(downButtonPin);     //Check buttons for manual adjustments




if (modeState == HIGH)     //if HIGH, circuit is in Voltage Output Mode
 {
  targetCurrent = (int) current;     //sets target value to current value to avoid problems when switching modes

  if(previousTargetVoltage != targetVoltage || previousOutputValue != outputValue )     //dispay serial data
   {
    Serial.println("Voltage Mode");     //display measured values
    Serial.print("Output Value: ");
    Serial.println(outputValue);  
    //Serial.print("Voltage Point 1: "); 
    //Serial.println(voltageOne);  
    //Serial.print("Voltage Point 2: "); 
    //Serial.println(voltageTwo);  
    //Serial.print("Voltage Difference: "); 
    //Serial.println(voltageDifference);  
    Serial.print("Target Voltage: "); 
    Serial.println(targetVoltage); 
    Serial.print("Terminal Voltage: "); 
    Serial.println(terminalVoltage);  
    Serial.print("Voltage Error: "); 
    Serial.println(voltageError); 
    Serial.print("Current (mA): "); 
    Serial.println(current);  
    Serial.println();     
    Serial.println(); 
    Serial.println(); 
   }
 
  previousOutputValue = outputValue;  
  previousTargetVoltage = targetVoltage;

  if (upButtonState == LOW)     //if up adjustment button is pressed
   {
    targetVoltage = targetVoltage + 0.05;     //Adjust target voltage setting up   
    if(targetVoltage > 5)
     {
      targetVoltage = 5;     //maximum target value
     }
    delay(200);     //delay to avoid errors from switch bounce
   }

  if (downButtonState == LOW)     //if down adjustment button is pressed
   {
    targetVoltage = targetVoltage - 0.05;     //Adjust target voltage setting down 
    if(targetVoltage < 0)
     {
      targetVoltage = 0;     //minumum value of 0
     }
    delay(200);     //delay to avoid errors from switch bounce
   }  

  if(current < 150)     //if current is above 150mA
   {
     if(abs(voltageError) > 0.04)     //if output error is large enough adjust output
     {
      outputValue = outputValue + voltageError * 50;
      analogWrite(outputPin, outputValue);     //write new value
      delay(100);     //delay to allow the output to adjust
     }
   }
  else
   {
    outputValue = outputValue - 1;     //lower current until it is below 150mA to protect the Arduino
   }
 }



if (modeState == LOW)     //if LOW, circuit is in Current Output Mode 
 {
  targetVoltage = terminalVoltage;     //sets target value to current value to avoid problems when switching modes  

  if(previousTargetCurrent != targetCurrent || previousOutputValue != outputValue )     //dispay serial data
   {
    Serial.println("Current Mode");     //display measured values
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
  
  
  
  if (upButtonState == LOW)     //if up adjustment button is pressed 
   {
    targetCurrent = targetCurrent + 5;     //Adjust target voltage setting up   
    if(targetCurrent > 150)     //limit current to 150mA to protect the Arduino
     {
      targetCurrent = 150;
     }
    delay(200);     //delay to avoid errors from switch bounce
   }

  if (downButtonState == LOW)     //if down adjustment button is pressed 
   {
    targetCurrent = targetCurrent - 5;     //Adjust target voltage setting down 
    if(targetCurrent < 0)
     {
      targetCurrent = 0;
     }
    delay(200);     //delay to avoid errors from switch bounce
   } 
  

  
  if(abs(currentError) > 5)     //if output error is large enough adjust output
   {
    outputValue = outputValue + currentError / 5;
    analogWrite(outputPin, outputValue);     //write new value
    delay(100);     //delay to allow the output to adjust
   }
}

if(outputValue < 1)    //output can never go below 0
 {
  outputValue = 0;
 }

if(outputValue > 250)     //output can never go above 255
 {
  outputValue = 250;
 }
 
}  

