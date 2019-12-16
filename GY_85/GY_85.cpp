#include "GY_85.h"
#include <math.h>

GY_85::GY_85(PinName sda, PinName scl): i2c(sda,scl)
{
    i2c.frequency(400000);
	//Sensor variable initialization
	accel[0]=accel[1]=accel[2]=0;
	magneto[0]=magneto[1]=magneto[2]=0;
	gyro[0]=gyro[1]=gyro[2]=0;
	
    //Sensor offset initiation
    g_offx=0;
    g_offy=0;
    g_offz=0;
	
	//DCM variable initialization
	Accel_Vector[0] = Gyro_Vector[0] = Omega_Vector[0] = Omega_P[0] = Omega_I[0] = Omega[0] = errorRollPitch[0] = errorYaw[0] = 0;
    Accel_Vector[1] = Gyro_Vector[1] = Omega_Vector[1] = Omega_P[1] = Omega_I[1] = Omega[1] = errorRollPitch[1] = errorYaw[1] = 0;
    Accel_Vector[2] = Gyro_Vector[2] = Omega_Vector[2] = Omega_P[2] = Omega_I[2] = Omega[2] = errorRollPitch[2] = errorYaw[2] = 0;

    DCM_Matrix[0][0] = 1;
    DCM_Matrix[0][1] = 0;
    DCM_Matrix[0][2] = 0;
    DCM_Matrix[1][0] = 0;
    DCM_Matrix[1][1] = 1;
    DCM_Matrix[1][2] = 0;
    DCM_Matrix[2][0] = 0;
    DCM_Matrix[2][1] = 0;
    DCM_Matrix[2][2] = 1;

    Update_Matrix[0][0] = 0;
    Update_Matrix[0][1] = 1;
    Update_Matrix[0][2] = 2;
    Update_Matrix[1][0] = 3;
    Update_Matrix[1][1] = 4;
    Update_Matrix[1][2] = 5;
    Update_Matrix[2][0] = 6;
    Update_Matrix[2][1] = 7;
    Update_Matrix[2][2] = 8;

    Temporary_Matrix[0][0] = 0;
    Temporary_Matrix[0][1] = 0;
    Temporary_Matrix[0][2] = 0;
    Temporary_Matrix[1][0] = 0;
    Temporary_Matrix[1][1] = 0;
    Temporary_Matrix[1][2] = 0;
    Temporary_Matrix[2][0] = 0;
    Temporary_Matrix[2][1] = 0;
    Temporary_Matrix[2][2] = 0;
	
	timer.start();
	
	//Debug purpose only
	dbg=0;
}

void GY_85::init()
{   
    //Sensor initialization
    initMagneto();
    initGyro();
    initAccel();
}

void GY_85::initMagneto()
{
	char cmd[2];
    //Magnetometer initiation set default
    cmd[0]=0x02; cmd[1]=0x00;
    i2c.write(HMC5883, cmd, 2);
    wait(0.1);
    
    //Set 50Hz
    cmd[0]=0x00;cmd[1]=0b00011000;  // Set 50Hz
    i2c.write(HMC5883, cmd, 2);
    wait(0.1);
}
    

void GY_85::updateMagneto()
{
    char cmd[2];
    char tmp[6];
     
    //Request data magnetometer
    cmd[0]=0x03;
    i2c.write(HMC5883, cmd, 1);
            
    //Read data from each axis, 2 registers per axis
    i2c.read(HMC5883,tmp, 6 );
    magneto[0] =((tmp[0]<<8)|tmp[1]);    //X axis (internal sensor x axis)
    magneto[2] =-1*((tmp[2]<<8)|tmp[3]); //Z axis (internal sensor -y axis)
    magneto[1] =-1*((tmp[4]<<8)|tmp[5]); //Y axis (internal sensor -z axis)      
}

void GY_85::initGyro()
{
	char cmd[2];
    // Power up reset defaults
    cmd[0]=0x3E; cmd[1]=0x80;
    i2c.write(ITG3200, cmd, 2);
    wait(0.1);
    
    // Select full-scale range of the gyro sensors
    // Set LP filter bandwidth to 42Hz
    cmd[0]=0x16; cmd[1]=0x1B;
    i2c.write(ITG3200, cmd, 2);
    wait(0.1);
    
    // Set sample rate to 50Hz
    cmd[0]=0x15; cmd[1]=0x0A;       // +/- 2000 dgrs/sec, 1KHz, 1E, 19
    i2c.write(ITG3200, cmd, 2);
    wait(0.1);
    
    //gak tahu apa
    cmd[0]=0x17; cmd[1]=0x00;
    i2c.write(ITG3200, cmd, 2);
    wait(0.1);
    
    // Set clock to PLL with z gyro reference
    cmd[0]=0x3E; cmd[1]=0x00;
    i2c.write(ITG3200, cmd, 2);
    wait(0.1);
    
    // Gyro Calibration
    int16_t tmpx = 0;
    int16_t tmpy = 0;
    int16_t tmpz = 0;
    g_offx=0;
    g_offy=0;
    g_offz=0;
    for( int i = 0; i < 10; i ++ ) //take the mean from 10 gyro probes and divide it from the current probe
    {
        wait(0.01);
        updateGyro();
        tmpx += magneto[0];
        tmpy += magneto[1];
        tmpz += magneto[2];
    }
    g_offx = tmpx/10;
    g_offy = tmpy/10;
    g_offz = tmpz/10;
}

void GY_85::updateGyro()
{
    char cmd[2];
    char tmp[8];
    static int16_t data[4];
    
    //request data gyro
    cmd[0]= 0x1B;
    i2c.write(ITG3200, cmd, 1);
    
    //take data gyro
    i2c.read(ITG3200,tmp, 8 );   
    gyro[0] = ((tmp[4] << 8) | tmp[5])-g_offx;   //X
    gyro[1] = ((tmp[2] << 8) | tmp[3])-g_offy;   //Y
    gyro[2] = ((tmp[6] << 8) | tmp[7])-g_offz;   //Z
    gyro[3] = (tmp[0] << 8) | tmp[1];   // temperature
}

void GY_85::initAccel()
{
    char cmd[2];
    //Power register seting for maesurement mode
    cmd[0]=0x2D; cmd[1]=0x08;
    i2c.write(ADXL345, cmd, 2);
    wait(0.1);
    
    //Set full scale resolution
    cmd[0]=0x31; cmd[1]=0x08;
    i2c.write(ADXL345, cmd, 2);
    wait(0.1);
    
    // Because our main loop runs at 50Hz we adjust the output data rate to 50Hz (25Hz bandwidth)
    cmd[0]=0x2C; cmd[1]=0x09;
    i2c.write(ADXL345, cmd, 2);
    wait(0.1);
}

void GY_85::updateAccel()
{
    char cmd = 0x32;
    char tmp[6];
    
    //request data from accelerometer
    i2c.write(ADXL345, &cmd, 1);
    wait (0.01);
    //take data from accelerometer    
    if(i2c.read(ADXL345_READ,tmp, 6 )==0){   
    accel[1] =(((int)tmp[1] << 8) | (int)tmp[0]); // X axis (internal sensor y axis)
    accel[0] =(((int)tmp[3] << 8) | (int)tmp[2]); // Y axis (internal sensor x axis)
    accel[2] =(((int)tmp[5] << 8) | (int)tmp[4]); // Z axis (internal sensor z axis)
	}
}

void GY_85::calculateHeading()
{
    //to degug change the accelerometer sign or swap the place 
    float temp0[3] = { accel[0], accel[1], accel[2] };
    
    float temp1[3];
    float temp2[3];
    float xAxis[] = {1.0f, 0.0f, 0.0f};
    static float data[3];

    // GET PITCH
    // Using y-z-plane-component/x-component of gravity vector
    pitch = -atan2((double)temp0[0], sqrt((double)(temp0[1] * temp0[1] + temp0[2] * temp0[2])));

    // GET ROLL
    // Compensate pitch of gravity vector
    Vector_Cross_Product(temp1, temp0, xAxis);
    Vector_Cross_Product(temp2, xAxis, temp1);
    // Normally using x-z-plane-component/y-component of compensated gravity vector
    // roll = atan2(temp2[1], sqrt(temp2[0] * temp2[0] + temp2[2] * temp2[2]));
    // Since we compensated for pitch, x-z-plane-component equals z-component:
    roll = atan2(temp2[1], temp2[2]);

    // GET YAW
    // Calculate trigonometry
    float cos_roll = cos(data[1]);
    float sin_roll = sin(data[1]);
    float cos_pitch = cos(data[0]);
    float sin_pitch = sin(data[0]);
    // Tilt compensated magnetic field X
    float mag_x = magneto[0]*cos_pitch + magneto[1]*sin_roll*sin_pitch + magneto[2]*cos_roll*sin_pitch;
    // Tilt compensated magnetic field Y
    float mag_y = magneto[1]*cos_roll - magneto[2]*sin_roll;
    // Magnetic Heading
    MAG_Heading =  atan2(-mag_y, mag_x);
    yaw = MAG_Heading;
	
	init_rotation_matrix(DCM_Matrix, yaw, pitch, roll);
}

void GY_85::compassHeading()
{
  float mag_x;
  float mag_y;
  float cos_roll;
  float sin_roll;
  float cos_pitch;
  float sin_pitch;
  
  cos_roll = cos(roll);
  sin_roll = sin(roll);
  cos_pitch = cos(pitch);
  sin_pitch = sin(pitch);
  
  // Tilt compensated magnetic field X
  mag_x = magneto[0]*cos_pitch + magneto[1]*sin_roll*sin_pitch + magneto[2]*cos_roll*sin_pitch;
  // Tilt compensated magnetic field Y
  mag_y = magneto[1]*cos_roll - magneto[2]*sin_roll;
  // Magnetic Heading
  MAG_Heading = atan2(-mag_y, mag_x);
}

void GY_85::updateALL() {
    timestamp_old = timestamp;
    timestamp = timer.read_ms();
    if (timestamp > timestamp_old)
        G_Dt = (float) (timestamp - timestamp_old) / 1000.0f; // Real time of loop run. We use this on the DCM algorithm (gyro integration time)
    else
        G_Dt = 0;

    // Update sensor readings
	updateMagneto();
    updateGyro();
    updateAccel();

    // Run DCM algorithm
	compassHeading(); // Calculate magnetic heading
	Matrix_update();
	Normalize();
	Drift_correction();
	Euler_angles();
}

//Debug purpose only
void GY_85::debug(){
	updateAccel();
	dbg++;
}