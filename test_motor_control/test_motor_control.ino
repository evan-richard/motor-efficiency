// This module controls a BLDC bafang motor by specifying the 3-phase power supplied to it.
// The speed is read from a throttle and output is sent to the motor using PWM.
// Correction for efficiency is based on a PID loop and gear changes are specified as
// necessary to maintain speed.
//
// Author: Evan Richard

// phase outputs: on an Arduino Uno, PWM output is possible on digital I/O pins 3, 5, 6, 9, 10 and 11.
#define INHA 9
#define INHB 10
#define INHC 11
#define INLA 5
#define INLB 6
#define INLC 7

// Original
 #define HALLA 8
 #define HALLB 12
 #define HALLC 13

// #define HALLA 8
// #define HALLB 13
// #define HALLC 12
// 
// #define HALLA 12
// #define HALLB 8
// #define HALLC 13
//
// #define HALLA 12
// #define HALLB 13
// #define HALLC 8
//
// #define HALLA 13
// #define HALLB 8
// #define HALLC 12
//
// #define HALLA 13
// #define HALLB 12
// #define HALLC 8

// Shunt Amplifier input vals: analog
#define SOA A0
#define SOB A1
#define SOC A2

// Potentiometer input val: analog
#define POT A3

// set enable low to turn off driver
// send out pulse to reset driver
#define ENABLE 4

// fault condition: default low, error high
#define nFAULT 2

// threshold for the hall sensor RPM (more accurate when higher)
#define HALL_THRESH 100.0

int fault;
float desiredSpeed, pwmOutput;                    // pwm to write to the 3-phase motor
int hallValA, hallValB, hallValC; // values from hall sensors A, B, C respectively
float curr1, curr2, curr3;          // current vals from SOA, SOB, SOC respectively
float potValue, lastVal;            // current and last potentiometer value, respectively

void setup() {
  // read at baud rate: 9600
  Serial.begin(9600);

  // assume the driver starts properly
  fault = 0;
  
  // configure the phase output pins
  pinMode(INHA, OUTPUT);
  pinMode(INHB, OUTPUT);
  pinMode(INHC, OUTPUT);
  pinMode(INLA, OUTPUT);
  pinMode(INLB, OUTPUT);
  pinMode(INLC, OUTPUT);

  digitalWrite(INLA, HIGH);
  digitalWrite(INLB, HIGH);
  digitalWrite(INLC, HIGH);
  delay(15);

  // configure hall sensor input pins
  pinMode(HALLA, INPUT);
  pinMode(HALLB, INPUT);
  pinMode(HALLC, INPUT);

  // configure nFault input
  pinMode(nFAULT, INPUT);

  /* Set PWM frequency on pins 9,10, and 11
  // this bit of code comes from 
  http://usethearduino.blogspot.com/2008/11/changing-pwm-frequency-on-arduino.html
  */  
  // Set PWM for pins 9,10 to 32 kHz
  //First clear all three prescaler bits:
  int prescalerVal = 0x07; //create a variable called prescalerVal and set it equal to the binary number "00000111"
  TCCR1B &= ~prescalerVal; //AND the value in TCCR0B with binary number "11111000"

  //Now set the appropriate prescaler bits:
  int prescalerVal2 = 1; //set prescalerVal equal to binary number "00000001"
//  int prescalerVal2 = 0x03; //set prescalerVal equal to binary number "00000010"
  TCCR1B |= prescalerVal2; //OR the value in TCCR0B with binary number "00000001"
  
  // Set PWM for pins 3,11 to 32 kHz (Only pin 11 is used in this program)
  //First clear all three prescaler bits:
  TCCR2B &= ~prescalerVal; //AND the value in TCCR0B with binary number "11111000"

  //Now set the appropriate prescaler bits:
 
  TCCR2B |= prescalerVal2; //OR the value in TCCR0B with binary number "00000001"//First clear all three prescaler bits:

  // configure enable output
  pinMode(ENABLE, OUTPUT);
  digitalWrite(ENABLE, HIGH);
}

void loop() {
  
  fault = digitalRead(nFAULT);

  if (!fault) {
    Serial.println("FAULT***");
    digitalWrite(ENABLE, LOW);
    delay(30);
    return;
  }

  digitalWrite(ENABLE, HIGH);

  pwmOutput = 100;
  
  // MARK: Write RPM/torque to motor
//  writePwm();
  
  // MARK: measure current and voltage(*****STILL NEED****)
  curr1 = analogRead(SOA);  // read the current input pins
  curr2 = analogRead(SOB);  // read the current input pins
  curr3 = analogRead(SOC);  // read the current input pins
  Serial.print("SOA: : ");
  Serial.println(curr1);
  Serial.print("SOB: ");
  Serial.println(curr2);
  Serial.print("SOC: ");
  Serial.println(curr3);
  float i = max(max(curr1, curr2), curr3);

  // MARK: measure torque and RPM(motor)
  // torque = k_t * i;
  // float rpmMotor = getRpm(HALLA);
}

// Returns the RPM based on readings from a single hall sensor
// Borrowed from: Joshua Hrisko, via
// https://engineersportal.com/blog/2018/10/3/arduino-tachometer-using-a-hall-effect-sensor-to-measure-rotations-from-a-fan 
float getRpm(int hallSensor) {
  float hall_count = 1.0;       // how many times the sensor is toggled
  float start = micros();       // start time in µs
  bool on_state = false;        // is the sensor high or low (prevent double counting)
  
  while(true){
    if (digitalRead(hallSensor) == 0) {
      if (on_state == false) {
        on_state = true;
        hall_count+=1.0;
      }
    } else {
      on_state = false;
    }
    
    if (hall_count >= HALL_THRESH) {
      break;
    }
  }
  
  // print information about Time and RPM
  float end_time = micros();
  float time_passed = ((end_time-start)/1000000.0);
  Serial.print("Time Passed: ");
  Serial.print(time_passed);
  Serial.println("s");
  float rpm_val = (hall_count/time_passed)*60.0;
  Serial.print(rpm_val);
  Serial.println(" RPM");
  return rpm_val;
}

// Determine the state from the hall sensors
// and from that write to the 3-phases the
// computed pwm value.
void writePwm() {
  // read the hall sensor input pins
  hallValA = digitalRead(HALLA);
  hallValB = digitalRead(HALLB);
  hallValC = digitalRead(HALLC);
  Serial.print("HALLA: ");
  Serial.println(hallValA);
  Serial.print("HALLB: ");
  Serial.println(hallValB);
  Serial.print("HALLC: ");
  Serial.println(hallValC);
  
    if (hallValA == 0 && hallValB == 0 && hallValC == 0) {
    // stop: should not get here
    digitalWrite(INHA, LOW);
    digitalWrite(INLA, LOW);
    digitalWrite(INHB, LOW);
    digitalWrite(INLB, LOW);
    digitalWrite(INHC, LOW);
    digitalWrite(INLC, LOW);
    Serial.println("Read STOP from hall sensors");
  } else if (hallValA == 1 && hallValB == 1 && hallValC == 1) {
    // align: should not get here
    analogWrite(INHA, pwmOutput);
    analogWrite(INLA, LOW);
    digitalWrite(INHB, LOW);
    digitalWrite(INLB, HIGH);
    digitalWrite(INHC, LOW);
    digitalWrite(INLC, HIGH);
    Serial.println("Read ALIGN from hall sensors");
  } else if (hallValA == 1 && hallValB == 1 && hallValC == 0) {
    // STATE 1:
    // write the pwm to the appropriate gate
    digitalWrite(INHA, LOW);
    digitalWrite(INLA, LOW);
    analogWrite(INHB, pwmOutput);
    digitalWrite(INLB, LOW);
    digitalWrite(INHC, LOW);
    digitalWrite(INLC, HIGH);
  } else if (hallValA == 1 && hallValB == 0 && hallValC == 0) {
    // STATE 2:
    // write the pwm to the appropriate gate
    analogWrite(INHA, pwmOutput);
    digitalWrite(INLA, LOW);
    digitalWrite(INHB, LOW);
    digitalWrite(INLB, LOW);
    digitalWrite(INHC, LOW);
    digitalWrite(INLC, HIGH);
  } else if (hallValA == 1 && hallValB == 0 && hallValC == 1) {
    // STATE 3:
    // write the pwm to the appropriate gate
    analogWrite(INHA, pwmOutput);
    digitalWrite(INLA, LOW);
    digitalWrite(INHB, LOW);
    digitalWrite(INLB, HIGH);
    digitalWrite(INHC, LOW);
    digitalWrite(INLC, LOW);
  } else if (hallValA == 0 && hallValB == 0 && hallValC == 1) {
    // STATE 4:
    // write the pwm to the appropriate gate
    digitalWrite(INHA, LOW);
    digitalWrite(INLA, LOW);
    digitalWrite(INHB, LOW);
    digitalWrite(INLB, HIGH);
    analogWrite(INHC, pwmOutput);
    digitalWrite(INLC, LOW);
  } else if (hallValA == 0 && hallValB == 1 && hallValC == 1) {
    // STATE 5:
    // write the pwm to the appropriate gate
    digitalWrite(INHA, LOW);
    digitalWrite(INLA, HIGH);
    digitalWrite(INHB, LOW);
    digitalWrite(INLB, LOW);
    analogWrite(INHC, pwmOutput);
    digitalWrite(INLC, LOW);
  } else if (hallValA == 0 && hallValB == 1 && hallValC == 0) {
    // STATE 6:
    // write the pwm to the appropriate gate
    digitalWrite(INHA, LOW);
    digitalWrite(INLA, HIGH);
    analogWrite(INHB, pwmOutput);
    digitalWrite(INLB, LOW);
    digitalWrite(INHC, LOW);
    digitalWrite(INLC, LOW);
  }
  Serial.flush();
}
