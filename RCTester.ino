/*
  RC Tester
  Christoffer Karlsson 2022

  Simple device for measuring resistor and capacitor values
  With auto-ranging, possibility of calibration over the Serial interface,
  and displaying measured values on a seven segment LED display

  Resistance measurement by a simple voltage divider with a resistor ladder
  Capacitance measurement using the same resistor ladder and capturing
  time-dependent voltage.
*/

///////////////////////////////////////////////////////////////
//Includes

#include <math.h>
#include <EEPROM.h>

///////////////////////////////////////////////////////////////
//Defines

//Pins for buttons to trigger measurements
#define TEST_RES 2
#define TEST_CAP 3

//Display shift register communication pins
#define CLOCK 10
#define LATCH 11
#define DATA 12

//#define RESET_COUNTER A4 //Pin to reset display decade counter (not used)

//Status LED pins
#define MSRLED LED_BUILTIN
#define OLLED A3

//Test resistor pins (connect to AIN)
//N.B. Must be in decreasing pin order with resistance
#define R47 9
#define R1k 8
#define R10k 7
#define R100k 6
#define R1M 5
#define R10M 4

//Defines first used resistor and the upper/lower limits
#define RSTART R10k
#define RMIN R47
#define RMAX R10M

//Number of calibration factors = one more than highest resistor pin
#define N_CALFAC RMIN + 1

//Test pin (connect test device to ground)
#define AIN A0

//Constants for resistor test
#define RES_VALUE_MIN 200
#define RES_VALUE_MAX 800
#define RES_VALUE_OL 1000

//Constants for capacitor test
#define INIT_VALUE_MIN 7
#define INIT_VALUE_MAX 100
#define FINAL_VALUE_MIN 600
#define FINAL_VALUE_MAX 1015

#define EEPROM_ADDR 0 //Address location for EEPROM storage

///////////////////////////////////////////////////////////////
//Constants

//Bit values to display characters on the 6xseven segment display
                       // 0     1     2     3     4     5     6     7     8     9
const byte number[10] = {0xfc, 0x60, 0xda, 0xf2, 0x66, 0xb6, 0xbe, 0xe0, 0xfe, 0xf6},
           // p     n     u     m           k     M
  prefix[7] = {0xce, 0x2a, 0x4e, 0xec, 0x00, 0x2e, 0xec},
          // ohm   farad
  unit[2] = {0x3a, 0x8e};
//Characters for units and prefixes
const char prefixChar[] = "pnum kM";
const String unitStr[2] = {"Ohm", "F"};

//Variables
int resistor = RSTART, //Currently resistor pin in resistor ladder used for measurement
  measureDelay = 100; //Delay for capacitance measurements
bool keepResistor = false; //Keep using the same resistor for capacitance measurements

float calibrationFactor[N_CALFAC]; //Calibration factors

char displayValue[10];  //Currently displayed characters
int displayPrefix = 0, displayUnit = 0; //Unit and prefix displayed
bool overload = false; //If an overload was encountered during measurement
//Currently displayed bits
byte displayByte[8] = {0x12, 0x11, 0x11, 0x1e, 0x1f, 0x3f, 0x03, 0x20}; //HELLO!

//Flags to start resistance or capacitance measurement
volatile bool testResTriggered = false, testCapTriggered = false;

///////////////////////////////////////////////////////////////
//Functions

//Callback triggering resistance measurement
void testResTrigger() {
  testResTriggered = true;
}

//Callback triggering capacitance measurement
void testCapTrigger() {
  testCapTriggered = true;
}

//Setup
void setup() {
  delay(100);

  //Buttons to start measurement
  pinMode(TEST_RES, INPUT_PULLUP);
  pinMode(TEST_CAP, INPUT_PULLUP);
  
  //Attach interrupts for buttons and reset trigger flags
  attachInterrupt(digitalPinToInterrupt(TEST_RES), testResTrigger, FALLING);
  attachInterrupt(digitalPinToInterrupt(TEST_CAP), testCapTrigger, FALLING);
  resetTriggers();
  
  //Resistance ladder for measurement
  pinMode(R47, INPUT);
  pinMode(R1k, INPUT);
  pinMode(R10k, INPUT);
  pinMode(R100k, INPUT);
  pinMode(R1M, INPUT);
  pinMode(R10M, INPUT);
  
  //Input for device under test
  pinMode(AIN, INPUT);

  //LED outputs: Measure and overload, active low
  digitalWrite(MSRLED, HIGH);
  digitalWrite(OLLED, HIGH);
  pinMode(MSRLED, OUTPUT);
  pinMode(OLLED, OUTPUT);

  //Display communication pins
  digitalWrite(CLOCK, LOW);
  digitalWrite(LATCH, LOW);
  digitalWrite(DATA, LOW);
  pinMode(CLOCK, OUTPUT);
  pinMode(LATCH, OUTPUT);
  pinMode(DATA, OUTPUT);

  //Reset the decade counter at startup - can be achieved with timing circuit as well
  //pinMode(RESET_COUNTER, OUTPUT);
  //digitalWrite(RESET_COUNTER, HIGH);
  //delay(10);
  //digitalWrite(RESET_COUNTER, LOW);
  //pinMode(RESET_COUNTER, INPUT);

  //Display a little animation on startup
  for (int i=0; i<7; i++) {
    digitalWrite(LATCH, HIGH);
    delay(10);
    digitalWrite(LATCH, LOW);
    delay(10);
  }

  loadCalibrationFactors();

  //Initialize serial communication
  Serial.begin(9600);
  //Serial.println("Hello!");
  printSerialMenu(); //Print menu over serial
}

//Loop
void loop() {
  if (testResTriggered) { //Resistance measurement triggered
    startResTest();
    postTest();
  } else if (testCapTriggered) { //Capacitance measurement triggered
    startCapTest();
    postTest();
  }

  if (Serial.available() > 0) //If there's something received over serial:
    performSerialAction(Serial.read()); //Perform action accordingly

  showDisplay(); //Show display
}

//Start resistance measurement and store value
void startResTest() {
  //Reset overload LED and boolean
  digitalWrite(OLLED, HIGH);
  overload = false;
  
  float r = (float) testRes(); //Perform resistance measurement

  if (r < calibrationFactor[1]) { //If smaller than intrinsic resistance: overload
    overload = true;
    digitalWrite(OLLED, LOW);
    return;
  }

  r -= calibrationFactor[1]; //Adjust for intrinsic resistance
  
  //Choose appropriate prefix
  if (r < 1000) { //none (ohm)
    displayPrefix = 4;
  } else if (r < 1000000) { //k
    displayPrefix = 5;
    r /= 1000;
  } else { //M
    displayPrefix = 6;
    r /= 1000000;
  }

  dtostrf(r, 5, 3, displayValue); //Store resistance in displayValue to show
  displayUnit = 0; //Unit: ohm
}

//Measure resistance and return value in ohms
double testRes() {
  digitalWrite(MSRLED, LOW); //Show measure LED
  //Switch on analog input and turn the current test resistor to high
  pinMode(AIN, INPUT);
  digitalWrite(resistor, HIGH);
  pinMode(resistor, OUTPUT);
  delay(200); //Wait for 200ms
  int value = analogRead(AIN); //Read value on analog input
  pinMode(resistor, INPUT); //Turn off test resistor
  digitalWrite(MSRLED, HIGH); //Switch off measure LED
   
  if (value < RES_VALUE_MIN) { //If input is less than cutoff:
    if (resistor != RMIN) { //If resistor is not already the smallest:
      resistor++; //Go to smaller resistor
      return testRes(); //Measure again
    }
  } else if (value > RES_VALUE_MAX) { //If input is more than cutoff:
    if (resistor != RMAX) { //If resistor is not already the largest:
      resistor--; //Go to larger resistor
      return testRes(); //Measure again
    } else if (value > RES_VALUE_OL) //If largest resistor is used and value is high enough:
      return -1.0; //Return overload
  }

  //Calculate resistance value from analog input and return value in ohms
  double r = getResistance() * double(value) / double(1023 - value);
  return r;
}

//Start capacitance test and store value
void startCapTest() {
  //Reset overload
  digitalWrite(OLLED, HIGH);
  overload = false;
  
  double c = testCap(); //Measure capacitance

  if (c < calibrationFactor[2]) { //0.0001) { //If value is smaller than intrinsic capacitance: Overload
    digitalWrite(OLLED, LOW);
    overload = true;
    return;
  }
  
  c -= calibrationFactor[2]; //Adjust for intrinsic capacitance
  
  //Choose appropriate prefix
  if (c < 0.001) { //p
    c *= 1000000;
    displayPrefix = 0;
  } else if (c < 1) { //n
    c *= 1000;
    displayPrefix = 1;
  } else if (c < 1000) { //micro
    displayPrefix = 2;
  } else { //m
    c /= 1000;
    displayPrefix = 3;
  }

  dtostrf(c, 5, 3, displayValue); //Store value for displaying
  displayUnit = 1; //Unit: Farad
}

//Measure capacitance and return value in microfarad
double testCap() {
  //Turn analog input to low for 200ms to discharge capacitor before measurement
  pinMode(AIN, OUTPUT);
  digitalWrite(AIN, LOW);
  delay(200);
  pinMode(AIN, INPUT);
  //Start measurement
  //Serial.println("START");
  digitalWrite(MSRLED, LOW); //Turn on measurement LED
  digitalWrite(resistor, HIGH); //Turn the current resistor to high to start measurement
  pinMode(resistor, OUTPUT);
  unsigned long t0 = micros(); //Record time at starting
  //delayMicroseconds(500); //Wait 0.5ms before starting to record potential

  int intialValue = analogRead(AIN);

  //If initial input is above higher cutoff and resistor is not the largest one and we are allowed to change it:
  if (intialValue > INIT_VALUE_MAX && resistor != RMAX && !keepResistor) {
    pinMode(resistor, INPUT); //Turn of resistor
    //Serial.println("Too fast charge, increasing R...");
    resistor--; //Go to larger resistor
    return testCap(); //Measure capacitance again
  }
  //If initial input is less than lower cutoff and resistor is not the smallest one and we are allowed to change it:
  else if (intialValue < INIT_VALUE_MIN && resistor != RMIN && !keepResistor) {
    pinMode(resistor, INPUT); //Turn of resistor
    //Serial.println("Too slow charge, decreasing R...");
    resistor++; //Go to smaller resistor
    return testCap(); //Measure capacitance again
  }
  keepResistor = true; //Flag to keep the current resistor for any subsequent measurements

  //Analog input after 0.5ms is within the accepted limits: Start measuring the potential over time
  const int points = 100; //Record 100 data points
  int y[points]; //Potential of capacitor
  unsigned long t[points]; //Time
  
  for (int i=0; i<points; i++) { //Record the potential and time with a certain delay
    y[i] = analogRead(AIN);
    t[i] = micros();
    delayMicroseconds(measureDelay);
  }

  //Turn off resistor and measurement LED
  pinMode(resistor, INPUT);
  digitalWrite(MSRLED, HIGH);
  
  int finalValue = y[points-1]; //The last recorded potential
  //If the final potential is below the lower cutoff and the measurement delay is still not too high:
  if (finalValue < FINAL_VALUE_MIN && measureDelay < 8000) {
    //Serial.print("Too slow charge, increasing delay... ");
    //Serial.println(measureDelay);
    measureDelay *= 2; //Double the measurement delay to allow longer charge time
    return testCap(); //Measure capacitance again
  }
  //If the final potential is above the higher cutoff and the delay is not already zero:
  if (finalValue > FINAL_VALUE_MAX && measureDelay != 0) {
    //Serial.println("Too fast charge, decreasing delay...");
    measureDelay = 0; //Set delay to zero, measuring as fast as possible
    return testCap(); //Measure capacitance again
  }
  //The final value is within the accepted limits: Calculate the capacitance from the recorded data

  /** Print the recorded data
  Serial.println("PRINT");
  for (int i=0; i<points; i++) {
    //t[i] -= t0;
    //Serial.println(t[i]);
    //Serial.print(": ");
    Serial.println(y[i]);
    //delay(1);
  }/**/

  //Calculate the capacitance
  double res = getResistance(), //Value of the current resistor
    sumOfCap = 0.0; //Sum of capacitance values

  //Calculate the capacitance from each data point, and take the average
  int count = 0; //Number of data points within the accepted limits
  for (int i=0; i<points; i++) {
    if (y[i] < INIT_VALUE_MIN) //If the data point is below the accepted start value: ignore and continue
      continue;
    if (y[i] > FINAL_VALUE_MAX) //If the data point is above the accepted final value: stop averaging (plateau reached)
      break;
    //Calculate the capacitance from the current data point and add to the cumulative capacitance
    sumOfCap += double(t[i]-t0)/(res*log(1023.0/(1023.0-y[i])));
    count++; //Increment the number of calculated values
  }

  //Discharge capacitor by pulling the analog input low for 1s
  pinMode(AIN, OUTPUT);
  digitalWrite(AIN, LOW);
  delay(1000);
  pinMode(AIN, INPUT);

  //Reset measurement parameters
  measureDelay = 100;
  keepResistor = false;

  //If no points were in the acceptable window: Return overload (avoid dividing by 0)
  if (count == 0)
    return -1.0;

  //Serial.println("DONE");
  return sumOfCap / count; //Return average capacitance: sum of values / number of values
}

/* Measure inductance: Requires very large inductors or very high current resistors!
double testInd() {
}*/
    
//Return the resistance of a certain resistor
double getResistance() {
  if  (resistor < RMAX || resistor > RMIN) {
    Serial.println("ERROR: Unknown resistor!");
    return -1.0;
  }
  else return calibrationFactor[resistor];
}

//Reset the measurement trigger flags
void resetTriggers() {
  testResTriggered = false;
  testCapTriggered = false;
}

//Create the byte to be sent to the display from the displayValue
void createDisplayByte() {
  for (int i=0; i<8; i++) //Reset display byte
    displayByte[i] = 0x00;
  
  if (overload) return; //If overload: Display nothing

  //Loop over displayValue, which holds a c-string of the value to display
  //j: position in displayValue string
  //k: digit position on display to write (shifted due to decimal point)
  for (int j=0, k=0; k<6; j++, k++) {
    byte byteToWrite; //Current byte to write to displayByte
    if (displayValue[j] == '.') { //If current character is a decimal point:
      displayByte[7] |= (1 << k); //Add the appropriate segment to the current character
      k--; //Decrement k to continue writing to the same digit on the next loop
    } else if (k==4) { //If we are on position 4: Write the prefix
      byteToWrite = prefix[displayPrefix];
    } else if (k==5) { //If we are on position 5: Write the unit
      byteToWrite = unit[displayUnit];
    } else { //Otherwise: write the digit of the number
      byteToWrite = number[displayValue[j] - '0'];
    }

    //Loop through the bits of the byte to write, and set the appropriate bits in the displayByte
    for (int i=0; i<8; i++)
      if (bitRead(byteToWrite, i)) //If the current bit is set:
        displayByte[7-i] |= (1 << k); //Set that bit (reverse bit order) in the displayByte
        //displayByte[n] describes which digits has segment n enabled
        //displayByte[n] |= (1 << k) therefore turns on segment k in digit n
  }
}

//Shift data out to the display, one full loop through each segment
void showDisplay() {
  for (int i=0; i<8; i++) {
    digitalWrite(LATCH, LOW);
    shiftOut(DATA, CLOCK, LSBFIRST, displayByte[i]);
    digitalWrite(LATCH, HIGH);
    delayMicroseconds(3000);
  }
  digitalWrite(LATCH, LOW);
}

//Print the displayed value over the serial port
void printDisplay() {
  if (overload) Serial.println("Overload");
  else {
    Serial.print(displayValue);
    Serial.print(prefixChar[displayPrefix]);
    Serial.println(unitStr[displayUnit]);
  }
}

//Perform actions after measurement is done
void postTest() {
  printDisplay(); //Print the value to be displayed
  createDisplayByte(); //Create the bytes to show on the display
  resetTriggers(); //Reset the measurement triggers
}

//Perform action according to menu choice
void performSerialAction(int choice) {
  switch (choice) {
    case 10: case 13: case -1: //White space characters: Do nothing
      break;
    case '0':
      Serial.println(F("Measuring resistance..."));
      startResTest();
      postTest();
      break;
    case '1':
      Serial.println(F("Measuring capacitance..."));
      startCapTest();
      postTest();
      break;
    case '2':
      Serial.println(F("Calibration wizard"));
      flushSerialInput();
      Serial.println(F("Please insert 10 Mohm resistor and press Enter"));
      waitForSerialAcknowledgement();
      calibrateInternalResistorValue();
      flushSerialInput();
      Serial.println(F("Please insert 1 Mohm resistor and press Enter"));
      waitForSerialAcknowledgement();
      calibrateInternalResistorValue();
      flushSerialInput();
      Serial.println(F("Please insert 100 kohm resistor and press Enter"));
      waitForSerialAcknowledgement();
      calibrateInternalResistorValue();
      flushSerialInput();
      Serial.println(F("Please insert 10 kohm resistor and press Enter"));
      waitForSerialAcknowledgement();
      calibrateInternalResistorValue();
      flushSerialInput();
      Serial.println(F("Please insert 1 kohm resistor and press Enter"));
      waitForSerialAcknowledgement();
      calibrateInternalResistorValue();
      flushSerialInput();
      Serial.println(F("Please insert 47 ohm resistor and press Enter"));
      waitForSerialAcknowledgement();
      calibrateInternalResistorValue();
      flushSerialInput();
      Serial.println(F("Please short the measurement leads and press Enter"));
      waitForSerialAcknowledgement();
      calibrateIntrinsicResistance();
      flushSerialInput();
      Serial.println(F("Please leave the measurement leads open and press Enter"));
      waitForSerialAcknowledgement();
      calibrateIntrinsicCapacitance();
      Serial.println(F("Calibration wizard complete!"));
      Serial.println(F("New calibration factors:"));
      printCalibrationFactors();
      break;
    case '3':
      Serial.println(F("Calibrating internal resistor value"));
      calibrateInternalResistorValue();
      break;
    case '4':
      Serial.println(F("Calibrating intrinsic resistance..."));
      calibrateIntrinsicResistance();
      break;
    case '5':
      Serial.println(F("Calibrating intrinsic capacitance..."));
      calibrateIntrinsicCapacitance();
      break;
    case '6':
      Serial.println(F("Current calibration factors:"));
      printCalibrationFactors();
      break;
    case '7':
      Serial.print(F("Resetting calibration factors"));
      for (int i=0; i<3; i++) { //Employ waiting time before reset...
        delay(1000);
        Serial.print(". ");
      }
      resetCalibrationFactors();
      Serial.println(F("Done!"));
      break;
    default:
      Serial.print(F("Unknown menu choice: "));
      Serial.println(char(choice));
      Serial.println();
      printSerialMenu();
      break;
  }
  flushSerialInput(); //Flush any additional characters in serial input pipeline
  Serial.println(F("- - -"));
}

//Calibrate the value of an internal resistor
void calibrateInternalResistorValue() {
  flushSerialInput(); //Flush input
  Serial.setTimeout(1e7); //Set timeout of Serial.parseFloat to a very large value
  Serial.println(F("Please input exact value of resistor under test (ohm)"));
  float rMes = (float) testRes(); //Perform resistance measurement (auto range will determine which resistor will be calibrated)
  float rCal = Serial.parseFloat(); //Input actual value
  Serial.print(F("Resistor used for measurement: #")); //Show which number resistor was used and that will be calibrated
  Serial.println(resistor);
  Serial.print(F("Old calibration factor (ohm): ")); //Show old calibration factor
  Serial.println(calibrationFactor[resistor]);
  calibrationFactor[resistor] *= rCal / rMes; //Adjust calibration factor for new value
  writeCalibrationFactor(resistor); //Save the calibration factor to EEPROM
  Serial.print(F("New calibration factor (ohm): ")); //Show the new calibration factor value
  Serial.println(calibrationFactor[resistor]);
}

//Calibrate the intrinsic resistance
//i.e. the resistance measured for a short-circuit (contact resistances etc.)
void calibrateIntrinsicResistance() {
  Serial.print(F("Old intrinsic resistance value (mohm): ")); //Show old calibration value in milli-ohms
  Serial.println((1e3) * calibrationFactor[1]);
  calibrationFactor[1] = (float) testRes(); //Perform resistance measurement and store directly as calibration factor 1
  writeCalibrationFactor(1); //Save resistance of short circuit
  Serial.print(F("New intrinsic resistance value (mohm): ")); //Show the new calibration factor value
  Serial.println((1e3) * calibrationFactor[1]);
}

//Calibrate the instrinsic capacitance
//i.e. the capacitance measured for an open circuit on the measurement leads
void calibrateIntrinsicCapacitance() {
  Serial.print(F("Old intrinsic capacitance value (pF): ")); //Show old calibration value in picofarads
  Serial.println((1e6) * calibrationFactor[2]); //Capacitance values are in microfarads = 1e6pF
  calibrationFactor[2] = (float) testCap(); //Perform capacitance measurement and store directly as calibration factor 2
  writeCalibrationFactor(2); //Save capacitance of open circuit
  Serial.print(F("New intrinsic capacitance value (pF): ")); //Show the new calibration factor value
  Serial.println((1e6) * calibrationFactor[2]);
}

//Wait for some serial input and then flush the remaining input
void waitForSerialAcknowledgement() {
  waitForSerialInput();
  flushSerialInput();
}

//Wait for some serial input to arrive, but don't read anything
void waitForSerialInput() {
  while (Serial.available() == 0) {}
}

//Flush anything arriving on the serial input within 100ms
void flushSerialInput() {
  delay(100);
  while (Serial.available() > 0)
    Serial.read();
}

//Parse double from Serial - no advantage wrt the built-in Serial.parseFloat()
/*float serialParseDouble() {
  while (Serial.available() == 0) {} //Wait for input
  delay(1000);
  double value = 0.0, factor = 1.0;
  while (Serial.available() > 0) {
    int c = Serial.read();
    //if (c == 10 || c == 13 || c == -1) break;
    if (c >= '0' && c <= '9') {
      if (factor == 1)
        value = 10 * value + (c - '0');
      else {
        value = value + (double)(c - '0') / factor;
        factor *= 10;
      }
    } else if (c == '.') {
      factor = 10;      
    } else break;
  }
  return value;
}*/

//Load calibration factors from EEPROM - reset on first run
void loadCalibrationFactors() {

  if (EEPROM.read(EEPROM_ADDR) != 42) { //If the first address does not hold the magic number: First run
    EEPROM.write(EEPROM_ADDR, 42); //Write the magic number
    resetCalibrationFactors(); //Reset calibration factors
    return;
  }

  for (int i=1; i<N_CALFAC; i++) //Loop through the rest of the addresses and put them in the calibrationFactor array
    EEPROM.get(EEPROM_ADDR + sizeof(float) * i, calibrationFactor[i]);
}

//Save calibration factors to EEPROM
void saveCalibrationFactors() {
  for (int i=1; i<N_CALFAC; i++)
    writeCalibrationFactor(i);
}

//Write one calibration factor to EEPROM
void writeCalibrationFactor(int idx) {
  EEPROM.put(EEPROM_ADDR + sizeof(float) * idx, calibrationFactor[idx]);
}

//Reset calibration factors to standard values
void resetCalibrationFactors() {
  calibrationFactor[1] = 0.0f; //Intrinsic resistance
  calibrationFactor[2] = 0.0f; //Intrinsic capacitance
  calibrationFactor[3] = 0.0f; //Unused
  calibrationFactor[R47] =   47.0f; //Resistor ladder values
  calibrationFactor[R1k] =   1000.0f;
  calibrationFactor[R10k] =  10000.0f;
  calibrationFactor[R100k] = 100000.0f;
  calibrationFactor[R1M] =   1000000.0f;
  calibrationFactor[R10M] =  10000000.0f;
  
  saveCalibrationFactors(); //Save
}

//Show calibration factors over serial
void printCalibrationFactors() {
  Serial.print(F("Intrinsic resistance (mohm): "));
  Serial.println((1e3) * calibrationFactor[1]);
  Serial.print(F("Intrinsic capacitance (pF): "));
  Serial.println((1e6) * calibrationFactor[2]);
  Serial.println(F("Resistor values (ohm): "));
  for (int i=RMIN; i>=RMAX; i--)
    Serial.println(calibrationFactor[i]);
}

//Print the menu options over serial
void printSerialMenu() {
  Serial.println(F("=== Multimeter Main Menu ==="));
  Serial.println(F("Please perform action in <> before sending command in []:"));
  Serial.println(F("[0] Measure resistance <Insert resistor>"));
  Serial.println(F("[1] Measure capacitance <Insert capacitor>"));
  Serial.println(F("[2] Calibration wizard"));
  Serial.println(F("[3] Calibrate resistor value <Insert resistor>"));
  Serial.println(F("[4] Calibrate intrinsic resistance <Short terminals>"));
  Serial.println(F("[5] Calibrate intrinsic capacitance <Leave terminals open>"));
  Serial.println(F("[6] Show current calibration factors"));
  Serial.println(F("[7] Reset calibration factors <Cannot be undone!>"));
}
