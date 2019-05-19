//Gyro - Arduino UNO R3
//VCC  -  5V
//GND  -  GND
//SDA  -  A4
//SCL  -  A5
//INT - port-2

#include <Wire.h>
#include <Servo.h>
#include <LiquidCrystal_PCF8574.h>

#define SERVO1 9
#define SERVO2 10

#define SERVO1_MAX 179
#define SERVO1_MIN 10

#define SERVO2_MAX 0
#define SERVO2_MIN 180

#define WAIT 150

#define SCREEN_DELAY_TIME 500 //in millis second

//Declaring some global variables
int gyro_x, gyro_y, gyro_z;
long gyro_x_cal, gyro_y_cal, gyro_z_cal;
boolean set_gyro_angles;
int gear=3;
long count=0, tcount5=0, tcount4=0, tcount3=0, tcount2=0, tcount1=0;
int Isfirst=1, dif;

// Variables to track servo state
long releaseMillis = 0, pullMillis = 0;
int repeat = 0, state = 0, pulled = 0;

Servo myservo1, myservo2;  // create servo object to control a servo

LiquidCrystal_PCF8574 lcd(0x27); // set the LCD address to 0x27 for a 16 chars and 2 line display


long acc_x, acc_y, acc_z, acc_total_vector;
float angle_roll_acc, angle_pitch_acc;

float angle_pitch, angle_roll;
int angle_pitch_buffer, angle_roll_buffer;
float angle_pitch_output, angle_roll_output;
float correction_factor = -9;

long loop_timer;
int temp;
unsigned long father = 0;

void setup() {
  Wire.begin();                                                        //Start I2C as master
  setup_mpu_6050_registers();                                          //Setup the registers of the MPU-6050 
  Serial.print("  gear= ");
  for (int cal_int = 0; cal_int < 1000 ; cal_int ++){                  //Read the raw acc and gyro data from the MPU-6050 for 1000 times
    read_mpu_6050_data();                                             
    gyro_x_cal += gyro_x;                                              //Add the gyro x offset to the gyro_x_cal variable
    gyro_y_cal += gyro_y;                                              //Add the gyro y offset to the gyro_y_cal variable
    gyro_z_cal += gyro_z;                                              //Add the gyro z offset to the gyro_z_cal variable
    delay(3);                                                          //Delay 3us to have 250Hz for-loop   
 }
 myservo1.attach(SERVO1);  // attaches the servo on pin 9 to the servo object
 myservo2.attach(SERVO2);  // attaches the servo on pin 9 to the servo object
 myservo1.write(SERVO1_MIN);
 myservo2.write(SERVO2_MIN);
  
  // divide by 1000 to get avarage offset
  gyro_x_cal /= 1000;                                                 
  gyro_y_cal /= 1000;                                                 
  gyro_z_cal /= 1000;                                                 
  Serial.begin(115200);
  loop_timer = micros();                                               //Reset the loop timer
  father = millis();
  releaseMillis = millis();
  pullMillis = millis();
  lcd.begin(16,2);
}

void loop(){

  read_mpu_6050_data();   
 //Subtract the offset values from the raw gyro values
  gyro_x -= gyro_x_cal;                                                
  gyro_y -= gyro_y_cal;                                                
  gyro_z -= gyro_z_cal;                                                
         
  //Gyro angle calculations . Note 0.0000611 = 1 / (250Hz x 65.5)
  angle_pitch += gyro_x * 0.0000611;                                   //Calculate the traveled pitch angle and add this to the angle_pitch variable
  angle_roll += gyro_y * 0.0000611;                                    //Calculate the traveled roll angle and add this to the angle_roll variable
  //0.000001066 = 0.0000611 * (3.142(PI) / 180degr) The Arduino sin function is in radians
  angle_pitch += angle_roll * sin(gyro_z * 0.000001066);               //If the IMU has yawed transfer the roll angle to the pitch angel
  angle_roll -= angle_pitch * sin(gyro_z * 0.000001066);               //If the IMU has yawed transfer the pitch angle to the roll angel
  
  //Accelerometer angle calculations
  acc_total_vector = sqrt((acc_x*acc_x)+(acc_y*acc_y)+(acc_z*acc_z));  //Calculate the total accelerometer vector
  //57.296 = 1 / (3.142 / 180) The Arduino asin function is in radians
  angle_pitch_acc = asin((float)acc_y/acc_total_vector)* 57.296;       //Calculate the pitch angle
  angle_roll_acc = asin((float)acc_x/acc_total_vector)* -57.296;       //Calculate the roll angle
  
  angle_pitch_acc -= 0.0;                                              //Accelerometer calibration value for pitch
  angle_roll_acc -= 0.0;                                               //Accelerometer calibration value for roll

  if(set_gyro_angles){                                                 //If the IMU is already started
    angle_pitch = angle_pitch * 0.9996 + angle_pitch_acc * 0.0004;     //Correct the drift of the gyro pitch angle with the accelerometer pitch angle
    angle_roll = angle_roll * 0.9996 + angle_roll_acc * 0.0004;        //Correct the drift of the gyro roll angle with the accelerometer roll angle
  }
  else{                                                                //At first start
    angle_pitch = angle_pitch_acc;                                     //Set the gyro pitch angle equal to the accelerometer pitch angle 
    angle_roll = angle_roll_acc;                                       //Set the gyro roll angle equal to the accelerometer roll angle 
    set_gyro_angles = true;                                            //Set the IMU started flag
  }
  
  //To dampen the pitch and roll angles a complementary filter is used
  angle_pitch_output = angle_pitch_output * 0.9 + angle_pitch * 0.1;   //Take 90% of the output pitch value and add 10% of the raw pitch value
  angle_roll_output = angle_roll_output * 0.9 + angle_roll * 0.1;      //Take 90% of the output roll value and add 10% of the raw roll value

  angle_roll_output = angle_roll_output-correction_factor;
  angle_roll_output = angle_roll_output;
  angle_pitch_output = angle_pitch_output-correction_factor;
  
 /*lcd.setCursor(0, 0);
 lcd.print(" Angle  = "); lcd.print(angle_pitch_output);
 lcd.setCursor(0, 1);
 lcd.print("  gear= "); lcd.print(gear);*/

 if(millis() - father >= SCREEN_DELAY_TIME){   
   lcd.setBacklight(255);
   lcd.setCursor(0,0);
   lcd.print("Angle: "); lcd.print(-1*angle_roll_output);
   lcd.setCursor(0,1);
   lcd.print("Gear: "); lcd.print(gear);
   setup_mpu_6050_registers();
   Serial.print("  Angle  = "); Serial.print(-1*angle_roll_output);
   Serial.print("  gear= "); Serial.println(gear);
   father = millis();
 }


 if (repeat) {                                      // in the middle of changing gears
  switch(state) {
    case 1:
      if (pulled && (millis() - pullMillis >= 450)) {
        gearupRelease();
        releaseMillis = millis();
        pulled = 0;
        repeat--;
      } else {
        if (!pulled && millis() - releaseMillis >= 500) {
          gearupPull();
          pullMillis = millis();
          pulled = 1;
        }
      }
      break;
    case 2:
      if (pulled && (millis() - pullMillis >= 350)) {
        geardownRelease();
        releaseMillis = millis();
        pulled = 0;
        repeat--;
      } else {
        if (!pulled && millis() - releaseMillis >= 500) {
          geardownPull();
          pullMillis = millis();
          pulled = 1;
        }
      }
      break;
    }
 } else {                                           // done changing gears, get new state
   if (angle_roll_output<-10 && gear != 10)
   {
    if (Isfirst==0){
      count=tcount1;
      count=count+1;
        Serial.print(" \t  \t  \t  \t count= "); Serial.println(count);
      if(count>=WAIT){
        
     dif=1-gear;
     if (dif==-1){
      state = 1;    // gearup
      repeat = 4;   // do it 4 times
     }
     else if (dif ==-2) {
      state = 1;    // gearup
      repeat = 3;   // do it 3 times
     }
     else if (dif== -3){
      state = 1;    // gearup
      repeat = 2;   // do it 2 times
      }
     else if (dif == -4)
      { 
        state = 1;    // gearup
        repeat = 1;   // do it 1 time
        }
     count=0;
     Isfirst=1;
     tcount1=0;

        }
        tcount1=count;
      }
    else if (Isfirst==1){
        Isfirst=0;
      }
    }
      
   if (angle_roll_output<=-6 && angle_roll_output>-10 && gear != 5)
   {

    if (Isfirst==0){
      count=tcount2;
      count=count+1;
        Serial.print(" \t  \t  \t  \t count= "); Serial.println(count);
      if(count>=WAIT){
        
     dif=2-gear;
     if (dif==-4){
      state = 2;    // geardown
      repeat = 1;   // do it 1 time
     }
     else if (dif ==-2) {
      state = 1;    // gearup
      repeat = 1;   // do it 1 time
     }
     else if (dif== -1){
      state = 1;    // gearup
      repeat = 2;   // do it 2 times
      }
     else if (dif == 0)
      { 
        state = 1;    // gearup
        repeat = 3;   // do it 3 times
        }
     count=0;
     Isfirst=1;
     tcount2=0;

        }
        tcount2=count;
      }
    else if (Isfirst==1){
        Isfirst=0;
      }
    }

   if (angle_roll_output <= -2 && angle_roll_output > -6 && gear != 4)
   {

    if (Isfirst==0){
      count=tcount3;
      count=count+1;
        Serial.print(" \t  \t  \t  \t count= "); Serial.println(count);
      if(count>=WAIT){
        
     dif=3-gear;
     if (dif==-3){
      state = 2;    // geardown
      repeat = 2;   // do it 2 times
     }
     else if (dif ==-2)
     {
      state = 2;    // geardown
      repeat = 1;   // do it 1 time
      }
     else if (dif== 0){
      state = 1;    // gearup
      repeat = 1;   // do it 1 time
      }
     else if (dif == 1)
      { 
        state = 1;    // gearup
        repeat = 2;   // do it 2 times
        }
     count=0;
     Isfirst=1;
     tcount3=0;

        }
        tcount3=count;
      }
    else if (Isfirst==1){
        Isfirst=0;
      }
    }

   if (angle_roll_output <= 2 && angle_roll_output > -2 && gear != 3)
   {

    if (Isfirst==0){
      count=tcount4;
      count=count+1;
        Serial.print(" \t  \t  \t  \t count= "); Serial.println(count);
      if(count>=WAIT){
        
     dif=4-gear;
     if (dif==-2){
      state = 2;    // geardown
      repeat = 3;   // do it 3 times
     }
     else if (dif ==-1)
     {
      state = 2;    // geardown
      repeat = 2;   // do it 2 times
     }
      
     else if (dif==0){
      state = 2;    // geardown
      repeat = 1;   // do it 1 time
      }
     else if (dif == 2)
      { 
        state = 1;    // gearup
        repeat = 1;   // do it 1 time
      }
     count=0;
     Isfirst=1;
     tcount4=0;

        }
        tcount4=count;
      }
    else if (Isfirst==1){
        Isfirst=0;
      }
    }

   if (angle_roll_output  >= 2 && gear != 2)
   {

    if (Isfirst==0){
      count=tcount5;
      count=count+1;
        Serial.print(" \t  \t  \t  \t count= "); Serial.println(count);
      if(count>=WAIT){
        
     dif=5-gear;
     if (dif==-1){
      state = 2;    // geardown
      repeat = 4;   // do it 4 times
     }
     else if (dif ==0){
      state = 2;    // geardown
      repeat = 3;   // do it 3 times
      }
     else if (dif== 1){
      state = 2;    // geardown
      repeat = 2;   // do it 2 times
      }
     else if (dif == 2)
      { 
        state = 2;    // geardown
        repeat = 1;   // do it 1 time
        }
     count=0;
     Isfirst=1;
     tcount5=0;

        }
        tcount5=count;
      }
    else if (Isfirst==1){
        Isfirst=0;
      }
    }
 }
    
  count=0;
  if (!(angle_roll_output<-10) && tcount1 != 0)
  tcount1=0;
  if (!(angle_roll_output<=-6 && angle_roll_output>-10) && tcount2 !=0)
  tcount2=0;
  if (!(angle_roll_output <= -2 && angle_roll_output > -6) && tcount3 != 0)
  tcount3=0;
  if (!(angle_roll_output <= 2 && angle_roll_output > -2 ) && tcount4 != 0)
  tcount4=0;
  if (!(angle_roll_output >= 2) && tcount5 != 0)
  tcount5=0;
 while(micros() - loop_timer < 4000);                                 //Wait until the loop_timer reaches 4000us (250Hz) before starting the next loop


 
 loop_timer = micros();//Reset the loop timer
  
}


void setup_mpu_6050_registers(){
  //Activate the MPU-6050
  Wire.beginTransmission(0x68);                                        //Start communicating with the MPU-6050
  Wire.write(0x6B);                                                    //Send the requested starting register
  Wire.write(0x00);                                                    //Set the requested starting register
  //Wire.endTransmission();                                             
  //Configure the accelerometer (+/-8g)
  //Wire.beginTransmission(0x68);                                        //Start communicating with the MPU-6050
  Wire.write(0x1C);                                                    //Send the requested starting register
  Wire.write(0x10);                                                    //Set the requested starting register
  //Wire.endTransmission();                                             
  //Configure the gyro (500dps full scale)
  //Wire.beginTransmission(0x68);                                        //Start communicating with the MPU-6050
  Wire.write(0x1B);                                                    //Send the requested starting register
  Wire.write(0x08);                                                    //Set the requested starting register
  Wire.endTransmission();                                             
}


void read_mpu_6050_data(){                                             //Subroutine for reading the raw gyro and accelerometer data
  Wire.beginTransmission(0x68);                                        //Start communicating with the MPU-6050
  Wire.write(0x3B);                                                    //Send the requested starting register
  Wire.endTransmission();                                              //End the transmission
  Wire.requestFrom(0x68,14);                                           //Request 14 bytes from the MPU-6050
  while(Wire.available() < 14);                                        //Wait until all the bytes are received
  acc_x = Wire.read()<<8|Wire.read();                                  
  acc_y = Wire.read()<<8|Wire.read();                                  
  acc_z = Wire.read()<<8|Wire.read();                                  
  temp = Wire.read()<<8|Wire.read();                                   
  gyro_x = Wire.read()<<8|Wire.read();                                 
  gyro_y = Wire.read()<<8|Wire.read();                                 
  gyro_z = Wire.read()<<8|Wire.read();                                 
}

void gearupPull(){
  myservo1.write(SERVO1_MAX);
  }

void gearupRelease(){
  myservo1.write(SERVO1_MIN);
  gear=gear+1;
  }
  
void geardownPull(){
  myservo2.write(SERVO2_MAX);
  }

  void geardownRelease(){
  myservo2.write(SERVO2_MIN);
  gear=gear-1;
  }
