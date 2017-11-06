//Libraries used
#include <VNH3SP30.h>
#include "hardware.h"
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    #include "Wire.h"
#endif
#include <Kalman.h> 
#include <PID_v1.h>
#include <SPI.h>
#include <Encoder_Buffer.h>
#include <Timer.h>
#include <LiquidCrystal_I2C.h>

VNH3SP30 MotorR(PWM_R, InARPin, InBRPin);
VNH3SP30 MotorL(PWM_L, InALPin, InBLPin);

Kalman kalmanX; 

PID SpeedPID(&SpeedInput, &SpeedOutput, &SpeedSetpoint, SpeedKp, SpeedKi, SpeedKd, DIRECT);
PID AnglePID(&AngleInput, &AngleOutput, &AngleSetpoint, AngleKp, AngleKi, AngleKd, DIRECT);

Encoder_Buffer EncoderR(EncoderCSR);
Encoder_Buffer EncoderL(EncoderCSL);

Timer t;

LiquidCrystal_I2C lcd(0x27, 20, 4);

////*Setup*////
void setup() {
  Serial.begin(38400); 
  while(Serial.available())  Serial.read();         // empty RX buffer
  
  beginIMU();           //Begin IMU comunication

  SPI.begin();
  EncoderR.initEncoder(); // Init quadrature encoder
  EncoderL.initEncoder(); // Init quadrature encoder

/*****LCD Initial message*****/
  lcd.begin();
  lcd.backlight();
  lcd.print("SuperDroid Robots");
  lcd.setCursor(0,1);
  lcd.print("Order Number: 50710");
  lcd.setCursor(0,2);
  lcd.print("Balancing Robot");
  delay(2000);
  lcd.clear();
  
  MotorR.Stop(); //Stop motor
  MotorL.Stop(); //Stop motor

  SpeedPID.SetOutputLimits(-15, 15); // Speed PID output limits+- 15 degrees
  SpeedPID.SetSampleTime(5);         //Sample time for Speed PID
  SpeedPID.SetMode(AUTOMATIC);       //turn the PID on

  AnglePID.SetOutputLimits(-255, 255); // Angle PID Output Limits +- 255 
  AnglePID.SetSampleTime(10);          //Sample time for Angle PID
  AnglePID.SetMode(AUTOMATIC);         //turn the PID on
 
  SpeedSetpoint = 0;// setpoint for speed PID

  t.every(100, ReadEncoders); // timer every 100ms to read encoders
  LCDBegin();
  t.every(200, UpdateLCD); //Timer every 200ms to update lcd

}

////*Main Loop*////
void loop() {
  t.update(); // for timer.h library
  
  ReadIMU(); //Read the IMU, convert to degrees and filter with kalman filter here is the input for the Angle PID
  
  if( abs(AngleInput) < 30 ){ //balance if angle input is les than 30 deg
        if(SpeedSetpoint == 0){
        //  EncoderL.clearEncoderCount();// Clear Encoders
         // EncoderR.clearEncoderCount();// Clear Encoders
         // encoderLeftReadingLast = 0;
         // encoderRightReadingLast = 0;
        }
        
        SpeedPID.SetMode(AUTOMATIC); //turn the PID on
        AnglePID.SetMode(AUTOMATIC); //turn the PID on
        SpeedPID.Compute(); // compute speed PID
        
        AngleSetpoint = SpeedOutput; // The output of the speed PID is the input of the angle PID
        AnglePID.Compute(); // the output of the angle PID is the motor command
        
        MoveMotors(); // command motors with the output of the angle PID
              
  }
  else{
    MotorR.Stop();
    MotorL.Stop();  
    SpeedPID.SetMode(MANUAL); //turn the PID off
    AnglePID.SetMode(MANUAL); //turn the PID off
    SpeedInput = 0;
    SpeedOutput = 0;
    EncoderL.clearEncoderCount();// Clear Encoders
    EncoderR.clearEncoderCount();// Clear Encoders
    encoderLeftReadingLast = 0;
    encoderRightReadingLast = 0;
  }

}

//Move motor is called in main loop
void MoveMotors(){
 // if(turn >50 || turn < -50) {
   //       turn = 0;
  //}
  int motorPower = AngleOutput;
  motorPowerRight = constrain(motorPower*1.08, -255, 255);
  motorPowerLeft = constrain(motorPower, -255, 255);

  
  if (motorPowerRight >= 0){ // if positive move one direction
      MotorR.Move(abs(motorPowerRight), HIGH);
      //MotorL.Move(abs(motorPower - turn), HIGH);
  }
  else{ // if negative move oposite direction
      MotorR.Move(abs(motorPowerRight), LOW);
      //MotorL.Move(abs(motorPower - turn), LOW);
    
  }

  if (motorPowerLeft >= 0){ // if positive move one direction
      //MotorR.Move(abs(motorPowerRight), HIGH);
      MotorL.Move(abs(motorPowerLeft), HIGH);
  }
  else{ // if negative move oposite direction
      //MotorR.Move(abs(motorPowerRight), LOW);
      MotorL.Move(abs(motorPowerLeft), LOW);
    
  }
  
 /* Serial.print(AngleInput); Serial.print("\t");
  Serial.print(SpeedInput); Serial.print("\t");
  Serial.print(SpeedOutput); Serial.print("\t");
  Serial.println(AngleOutput); */
}

// Read Encoders function is called every 100ms
void ReadEncoders(){

    encoderLeftReading = -1*EncoderL.readEncoder();//Read Encoder 

    LeftMotorSpeed = (encoderLeftReading - encoderLeftReadingLast); // calculate left speed
    encoderLeftReadingLast = encoderLeftReading;

    encoderRightReading = EncoderR.readEncoder();//Read Encoder 

    RightMotorSpeed = (encoderRightReading - encoderRightReadingLast); // calculate right speed
    encoderRightReadingLast = encoderRightReading;
    
    SpeedInput = RightMotorSpeed + LeftMotorSpeed; // input for speed PID 

    if (SpeedInput < 50) {
      EncoderL.clearEncoderCount();// Clear Encoders
      EncoderR.clearEncoderCount();// Clear Encoders
      encoderLeftReadingLast = 0;
      encoderRightReadingLast = 0;
     }
    
    //Serial.print(LeftMotorSpeed); Serial.print("\t"); //debug encoders
    //Serial.println(RightMotorSpeed); //Serial.print("\t");
    
}
