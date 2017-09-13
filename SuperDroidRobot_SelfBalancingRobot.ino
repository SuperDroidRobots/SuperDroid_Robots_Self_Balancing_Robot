//Libraries used
#include <VNH3SP30.h>
#include "hardware.h"
#include <Wire.h>
#include <Kalman.h> 
#include <PID_v1.h>
#include <SPI.h>
#include <Encoder_Buffer.h>
#include <Timer.h>

VNH3SP30 MotorR(PWM_R, InARPin, InBRPin);
VNH3SP30 MotorL(PWM_L, InALPin, InBLPin);

Kalman kalmanX; 

PID SpeedPID(&SpeedInput, &SpeedOutput, &SpeedSetpoint, SpeedKp, SpeedKi, SpeedKd, DIRECT);
PID AnglePID(&AngleInput, &AngleOutput, &AngleSetpoint, AngleKp, AngleKi, AngleKd, DIRECT);

Encoder_Buffer EncoderR(EncoderCSR);
Encoder_Buffer EncoderL(EncoderCSL);

Timer t;

////*Setup*////
void setup() {
  Serial.begin(115000); //Our debug Output
  beginIMU();           //Begin IMU comunication

  SPI.begin();
  EncoderR.initEncoder(); // Init quadrature encoder
  EncoderL.initEncoder(); // Init quadrature encoder
  
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

}

////*Main Loop*////
void loop() {
  t.update(); // for timer.h library
  
  ReadIMU(); //Read the IMU, convert to degrees and filter with kalman filter here is the input for the Angle PID
  
  if( abs(AngleInput) < 30 ){ //balance if angle input is les than 30 deg
        
        SpeedPID.SetMode(AUTOMATIC); //turn the PID on
        AnglePID.SetMode(AUTOMATIC); //turn the PID on
        SpeedPID.Compute(); // compute speed PID
        
        AngleSetpoint = SpeedOutput ; // The output of the speed PID is the input of the angle PID
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
  }
  
}

//Move motor is called in main loop
void MoveMotors(){
  
  int motorPower = AngleOutput;
  
  if (motorPower >= 0){ // if positive move one direction
      MotorR.Move(abs(motorPower), HIGH);
      MotorL.Move(abs(motorPower), HIGH);
    
  }
  else if(motorPower < 0){ // if negative move oposite direction
      MotorR.Move(abs(motorPower), LOW);
      MotorL.Move(abs(motorPower), LOW);
    
  }
  
  /*Serial.print(SpeedSetpoint); Serial.print("\t");
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
    
    //Serial.print(LeftMotorSpeed); Serial.print("\t"); //debug encoders
    //Serial.println(RightMotorSpeed); //Serial.print("\t");
    
}
