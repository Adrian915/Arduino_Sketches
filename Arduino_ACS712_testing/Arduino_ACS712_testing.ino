/*
  A SKETCH THAT TESTS A 5A ACS712 VOLTAGE AND CURRENT OUTPUTS IN NORMAL AND STRESS CONDITIONS, MEANT FOR CALIBRATON OF SENSORS AND RESULTS
  DEVICE WAS TESTED BY CONNECTING DIFFERENT 12V LED AND FILAMENT BULBS, WHICH RESULTED IN THE SAME OUTPUTS AS THE MULTIMETER
  
  SERIOUS WARNING - DO NOT IGNORE: 
    SINCE ACS712 CAN READ BOTH POSITIVE AND NEGATIVE, CALIBRATING THE SENSOR TO 0 (THAT MEANS OUTPUT = VCC / 2)IS IMPORTANT
    IN THE CALIBRATION PROCESS, THE VALUE OF VCC IS USED, WHICH CAN VARY DEPENDING ON YOUR SETUP AND CONSUMERS BEING USED AT THAT MOMENT
        E.G https://forum.arduino.cc/t/voltage-drop-when-relay-activates-and-affects-analog-readings/199499
    THIS WILL RESULT IN INCORRECT DATA
    MY PERSONAL SOLUTION WAS TO RESERVE A PAIR OF 5V AND GND PINS ONLY FOR THOSE SENSORS WHILE POWERING THE DEVICE FROM USB OR VIN

    Thanks for inspiration and basics:
    https://learn.sparkfun.com/tutorials/acs712-low-current-sensor-hookup-guide/calibration-and-example-code
*/

float vref = 0.0000;
float halfVref = 0.0000;

// internal bandgap reference voltage in millivolts
const unsigned long InternalReferenceMillivolts = 1080UL;


// Variables will change:
int ledState = LOW;             // ledState used to set the LED

// Generally, you should use "unsigned long" for variables that hold time
// The value will quickly become too large for an int to store
unsigned long previousMillis = 0;        // will store last time LED was updated

// constants won't change:
const long interval = 1000;           // interval at which to blink (milliseconds)


// the setup function runs once when you press reset or power the board
void setup() 
{
  Serial.begin(9600);
  
  // initialize digital pin LED_BUILTIN as an output.
  pinMode(LED_BUILTIN, OUTPUT);
}

// the loop function runs over and over again forever
void loop() {

  unsigned long currentMillis = millis();

  if (currentMillis - previousMillis >= interval) {
    // save the last time you blinked the LED
    previousMillis = currentMillis;

    // alternate relay for voltage drop tests & make sure the results are constant
    SwitchRelay();
  }                    

  // update VREF
  vref = getVccAmps();

  Serial.print("VOLTAGE is ");
  Serial.println(vref);

  halfVref = vref / 2;

  Serial.print("Half VOLTAGE (ZERO threshold) is ");
  Serial.println(halfVref);

  float current = Get_Current(vref);

  Serial.println("");
  Serial.println("");

  delay(500);
}

float getVccAmps()
{
  float result = 0.0000f;
  result = getVccMillivolts() / 1000;

  return result;
}

void SwitchRelay()
{
    // if the LED is off turn it on and vice-versa:
    if (ledState == LOW) {
      ledState = HIGH;
    } else {
      ledState = LOW;
    }

    // set the LED with the ledState of the variable:
    digitalWrite(LED_BUILTIN, ledState);
}

float Get_Voltage(float vref)
{
  int numSamples = 5000;
  float current = 0.0000f;
  unsigned long raw = 0.0000;
  for(int i=0; i<numSamples; i++) 
  {
    raw = raw + analogRead(A0);    
                                  
  }
  raw = raw / numSamples;

  float rawVref = raw * vref;
  float voltage = rawVref / 1020;

  Serial.print("raw * vref is ");
  Serial.print(rawVref);
  Serial.print(" where raw is ");
  Serial.print(raw);
  Serial.print(" and vref is ");
  Serial.println(vref);
  
  return(voltage);
}

float Get_Current(float vref)
{
  float current = 0.0000f;
  float voltageReading = Get_Voltage(vref);

  float halfVref = vref / 2;
  bool isNearZero = voltageReading < halfVref;
  if (!isNearZero)
  {
    // move on with the calculations
    current = (voltageReading - halfVref) / 0.185;
  }

  Serial.print("Read in voltage is ");
  Serial.println(voltageReading);
  Serial.print("which in current means ");
  Serial.println(current);
  
  
  return(current);
}

// Returns actual value of Vcc in millivolts
float getVccMillivolts()
{
  float result = 0.0000;
  // Set the analog reference to DEFAULT (AVcc == Vcc power rail)
  // REFS1 REFS0          --> 0b01   -Selects DEFAULT (AVcc) reference
  // Set the analog input to channel 14: the INTERNAL bandgap reference (1.1V +/- 10%)
  // MUX3 MUX2 MUX1 MUX0  --> 0b1110 -Selects channel 14, bandgap voltage, to measure
  ADMUX = (0 << REFS1) | (1 << REFS0) | (0 << ADLAR) | (1 << MUX3) | (1 << MUX2) | (1 << MUX1) | (0 << MUX0);

  delay(50);  // Let mux settle a little to get a more stable A/D conversion

  // Start a conversion to measure the INTERNAL reference relative to the DEFAULT (Vcc) reference.
  ADCSRA |= _BV( ADSC );
  // Wait for it to complete
  while (ADCSRA & (1 << ADSC));

  result = (InternalReferenceMillivolts * 1024UL) / ADC;

  // Calculate the power rail voltage (reference voltage) relative to the known voltage
  return result;
}
