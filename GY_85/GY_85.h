/*
This library using pin 
*/
#include "mbed.h"

#ifndef GY_85_h
#define GY_85_h

//Sensor Address
#define HMC5883 (0x3C)  //Magneto or Magneto meter
#define ADXL345 (0xA6)  //Accelerometer
#define ITG3200 (0xD0)   //Gyro
// Sensor I2C addresses 
#define ADXL345_READ  0xA7
#define HMC5883_READ   0x3D
#define ITG3200_READ   0xD1

// Gain for gyroscope (ITG-3200)
#define GYRO_GAIN 0.06957 // Same gain on all axes
#define GYRO_SCALED_RAD(x) (x * TO_RAD(GYRO_GAIN)) // Calculate the scaled gyro readings in radians per second

// DCM parameters
#define Kp_ROLLPITCH 0.02f
#define Ki_ROLLPITCH 0.00002f
#define Kp_YAW 1.2f
#define Ki_YAW 0.00002f
// When set to true, gyro drift correction will not be applied
#define DEBUG__NO_DRIFT_CORRECTION false

// Stuff
#define GRAVITY 256.0f // "1G reference" used for DCM filter and accelerometer calibration
#define TO_RAD(x) (x * 0.01745329252)  // *pi/180
#define TO_DEG(x) (x * 57.2957795131)  // *180/pi
#define NEW_LINE "\r\n"

class GY_85
{
public:
	//Initialization fuction
    GY_85(PinName sda, PinName scl);
    void init();
	//Basic Sensor functions
    void initMagneto();
    void initGyro();
    void initAccel();
    void updateMagneto();
    void updateGyro();
    void updateAccel();
    void calculateHeading();
	void compassHeading();
	void updateALL();
	//Sensor Variable
	int16_t accel[3];
	int16_t magneto[3];
	int16_t gyro[3];
	// Euler angles
    float yaw;
    float pitch;
    float roll;
	//Misc
	Timer timer;
	//Debug purpose only
	void debug();
	int dbg;
private:
    //Communication variable
    I2C i2c;
    //Gyro offset
    int16_t g_offx;
    int16_t g_offy;
    int16_t g_offz;
    // DCM variables
    float MAG_Heading;
    float Accel_Vector[3]; // Store the acceleration in a vector
    float Gyro_Vector[3];  // Store the gyros turn rate in a vector
    float Omega_Vector[3]; // Corrected Gyro_Vector data
    float Omega_P[3];//= {0, 0, 0}; // Omega Proportional correction
    float Omega_I[3];//= {0, 0, 0}; // Omega Integrator
    float Omega[3];//= {0, 0, 0};
    float errorRollPitch[3];// = {0, 0, 0};
    float errorYaw[3];// = {0, 0, 0};
    float DCM_Matrix[3][3];// = {{1, 0, 0}, {0, 1, 0}, {0, 0, 1}};
    float Update_Matrix[3][3];// = {{0, 1, 2}, {3, 4, 5}, {6, 7, 8}};
    float Temporary_Matrix[3][3];// = {{0, 0, 0}, {0, 0, 0}, {0, 0, 0}};
	// DCM timing in the main loop
    long timestamp;
    long timestamp_old;
    float G_Dt; // Integration time for DCM algorithm
	// DCM.cpp
    void Normalize();
    void Drift_correction();
    void Matrix_update();
    void Euler_angles();
	
};

//Vector calculation functions
float Vector_Dot_Product(float vector1[3], float vector2[3]);
void Vector_Cross_Product(float vectorOut[3], float v1[3], float v2[3]);
void Vector_Scale(float vectorOut[3], float vectorIn[3], float scale2);
void Vector_Add(float vectorOut[3], float vectorIn1[3], float vectorIn2[3]);
void Matrix_Multiply(float a[3][3], float b[3][3],float mat[3][3]);
void init_rotation_matrix(float m[3][3], float yaw, float pitch, float roll);
float constrain(float in, float min, float max);
float absolut(float x);

#endif