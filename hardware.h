//Connections to the arduino
//Left PWM motor controller pin out
#define PWM_L   6 
#define InALPin 5
#define InBLPin 4

//Right PWM motor controller pin out
#define PWM_R   3
#define InARPin 7
#define InBRPin 8

//Chip select for dual encoder buffer board to read encoders
#define EncoderCSL 9
#define EncoderCSR A0

//PID variables setpoints, outputs, inputs
double SpeedSetpoint , SpeedInput, SpeedOutput; 
double AngleSetpoint , AngleInput, AngleOutput;

// PIDs tune values
double SpeedKp = 0.04, SpeedKi = 0.06, SpeedKd = 0 ; // outer loop
double AngleKp = 35, AngleKi = 30, AngleKd = 0.35; // iner loop 

/* IMU Data and Kalman variables*/
double accY, accZ, gyroX;
double gyroXangle, gyroYangle; // Angle calculate using the gyro only
double compAngleX, compAngleY; // Calculated angle using a complementary filter
double kalAngleX, kalAngleY; // Calculated angle using a Kalman filter
uint32_t timer; // used for Kalman filter math
uint8_t i2cData[10]; // Buffer for I2C data
double dt, roll, gyroXrate;

// Encoders
long encoderRightReading = 0, encoderRightReadingLast = 0, RightMotorSpeed = 0;
long encoderLeftReading = 0, encoderLeftReadingLast = 0, LeftMotorSpeed = 0;



