#include "mbed.h"
#include "math.h"
#include "GY_85.h"
#include "GPS_UBLOX.h"
#include "VC0706.h"
#include "BufferedSerial.h"

//variable define
#define delay_us_tele 160
#define pi 3.14159265359F

//Funcstion
//extern "C" void mbed_reset();	//reset
void send_mode_s();             //Mode sync
void send_mode_compre();        //mode compressed

//Global Variable
int32_t longitude=0, latitude=0; 
int32_t heigh=0;

//temporary variables
int8_t* data;
int16_t temp;
uint8_t tempuint8_1,tempuint8_2;

//Control mode variable
//0=iddle do not send any data
//10=send data compre
//20=continously send picture with data
//30=continously send picture witout data
//40=send data compre once
//50=send picture with data once
//60=Send picture without data once
//100=Reset
uint8_t mode=10; 		//present mode
uint8_t last_mode=10;	//one mode before


//Polymorphism
class Camera: public VC0706 {
  public:
	int dataEnable=1;
    Camera(PinName tx, PinName rx): VC0706(tx,rx, PC_10, PC_11) {}
    void insertedTask()
	{
		if (dataEnable) send_mode_compre();
	}
};

GY_85 imu(PB_9,PB_8);
Camera cam(PA_9,PA_10);
GPS_UBLOX gps(PA_0,PA_1);

Ticker IMUloop;

int main ()
{
	int photoCount=0; //number of photo taken
	volatile int commandSense=0; //command header received
	char tempReceive; //for command parsing
	
    imu.init();
    //IMUloop.attach(&imu, &GY_85::updateALL, 0.01); // 100Hz update
	wait(0.1);
	//cam.photo(27);
	while(1)
    {
		switch(mode)
        {
			//0=iddle do not send any data
			case 0		:   // doing nothing
                                    break;			
		    //10=send data compre
			case 10		:   send_mode_compre();
                                    break;			
			//20=continously send picture with data
			case 20		:   cam.dataEnable=1;
							photoCount++;
							cam.photo(photoCount);
                                    break;									
			//30=continously send picture without data
			case 30		:   cam.dataEnable=0;
							photoCount++;
							cam.photo(photoCount);
                                    break;									
			//40=send data compre once
			case 40		:   send_mode_compre();
							mode=last_mode;
                                    break;
			//50=send picture with data once
			case 50		:   cam.dataEnable=1;
							cam.photo(0);
							mode=last_mode;
                                    break;
			//60=Send picture without data once
			case 60		:   cam.dataEnable=0;
							cam.photo(0);
							mode=last_mode;
                                    break;
			//100=Reset
            case 100	:   NVIC_SystemReset();
                                    break;
			//dafault
            default		:   mode=last_mode;
                                    break;
		}
		
		last_mode=mode;
		
		//Get Cammand from telemetry
		while(cam.pc.readable()>0)
		{
			tempReceive = cam.pc.getc();
			
			if(tempReceive =='1' && commandSense==0) {commandSense=1;}
			else if(tempReceive =='I' && commandSense==1) {commandSense=2;}
			else if(tempReceive =='t' && commandSense==2) {commandSense=3;}
			else if(tempReceive =='B' && commandSense==3)
			{
				//reset command detector
				commandSense=0;

				//wait for command data
				wait_ms(2);
				if(cam.pc.readable()<2) //timeout
				{
					cam.pc.putc(0x49);cam.pc.putc(0x74);cam.pc.putc(0x42);cam.pc.putc(0x32); //'ItB2'
					cam.pc.putc(98); //timeout command
					break;
				}
				//Get command data
				tempReceive = cam.pc.getc();
				if (cam.pc.getc()==tempReceive+7) mode = tempReceive; //data checking
				//Rebound command to the operator
				while(cam.pc.writeable()<5);
				cam.pc.putc(0x49);cam.pc.putc(0x74);cam.pc.putc(0x42);cam.pc.putc(0x32); //'ItB2'
				if(mode == tempReceive)cam.pc.putc(tempReceive);
				else cam.pc.putc(99);
				//Get the command data
				break;
			}
			else{commandSense=0;}
		}
    }
}

//compressed
void send_mode_compre()
{    
	imu.updateALL();
    //enter
    cam.tele_putc_new(0x0D); 
    
    //Header Code 018
    cam.tele_putc_new(0x30);
    cam.tele_putc_new(0x31);
    cam.tele_putc_new(0x38);
       
    //Acc-x
    data = (int8_t*) &imu.accel[0];
    cam.tele_putc_new(data[0]);
    cam.tele_putc_new(data[1]);
    
    //"
    cam.tele_putc_new(data[0]^data[1]^0x01);
    
    //Acc-y
    data = (int8_t*) &imu.accel[1];
    cam.tele_putc_new(data[0]);
    cam.tele_putc_new(data[1]);
    
    //#
    cam.tele_putc_new(data[0]^data[1]^0x02);
    
    //Acc-z
    data = (int8_t*) &imu.accel[2];
    cam.tele_putc_new(data[0]);
    cam.tele_putc_new(data[1]);
    
    //$
    cam.tele_putc_new(data[0]^data[1]^0x03);
    
    //Gyro-x
    data = (int8_t*) &imu.gyro[0];
    cam.tele_putc_new(data[0]);
    cam.tele_putc_new(data[1]);

    //%
    cam.tele_putc_new(data[0]^data[1]^0x04);
    
    //Gyro-y
    data = (int8_t*) &imu.gyro[1];
    cam.tele_putc_new(data[0]);
    cam.tele_putc_new(data[1]);
    
    //&
    cam.tele_putc_new(data[0]^data[1]^0x05);
    
    //Gyro-z
    data = (int8_t*) &imu.gyro[2];
    cam.tele_putc_new(data[0]);
    cam.tele_putc_new(data[1]);
    
    //'
    cam.tele_putc_new(data[0]^data[1]^0x06);
    
    //0 - was roll
    temp = (int16_t) (imu.roll/pi*180);
    //temp=90-temp;
    data = (int8_t*) &temp;
    cam.tele_putc_new(data[0]);
    cam.tele_putc_new(data[1]);
    
    //(
    cam.tele_putc_new(data[0]^data[1]^0x07);
    
    //1 - was pitch
    //temp=0;
    temp = (int16_t) (-1*imu.pitch/pi*180);
    //temp=-90-temp;
    data = (int8_t*) &temp;
    cam.tele_putc_new(data[0]);
    cam.tele_putc_new(data[1]);
    
    //)
    cam.tele_putc_new(data[0]^data[1]^0x08);
    
    //2 - was yaw
    temp = (int16_t) (imu.yaw/pi*180);
    //if (temp<0) temp=360+temp;
    data = (int8_t*) &temp;
    cam.tele_putc_new(data[0]);
    cam.tele_putc_new(data[1]);
       
    //_
    cam.tele_putc_new(data[0]^data[1]^0x09);
    
	gps.Read();
	
    //Lat-4byte
    latitude = gps.Lattitude;
    data = (int8_t*) &latitude;
    cam.tele_putc_new(data[0]);
    cam.tele_putc_new(data[1]);
    cam.tele_putc_new(data[2]);
    cam.tele_putc_new(data[3]);
    cam.tele_putc_new(data[0]^data[1]^data[2]^data[3]^0x0A);
    
    //Long-4byte
    longitude =gps.Longitude;
    data = (int8_t*) &longitude;
    cam.tele_putc_new(data[0]);
    cam.tele_putc_new(data[1]);
    cam.tele_putc_new(data[2]);
    cam.tele_putc_new(data[3]);
    cam.tele_putc_new(data[0]^data[1]^data[2]^data[3]^0x0B);
    
    //Alt-2 byte
    heigh =gps.Altitude;
    data = (int8_t*) &heigh;
    cam.tele_putc_new(data[0]);
    cam.tele_putc_new(data[1]);
	cam.tele_putc_new(data[2]);
	cam.tele_putc_new(data[3]);
    cam.tele_putc_new(data[0]^data[1]^data[2]^data[3]^0x0C);
    
    //-
	tempuint8_1= gps.Fix; //Gps satus
	tempuint8_2 = tempuint8_1;
    cam.tele_putc_new(tempuint8_1);
	tempuint8_1 = gps.NumFixSat; //number of gps fixed satelite
	tempuint8_2^=tempuint8_1;
	cam.tele_putc_new(tempuint8_1);
	tempuint8_1 = gps.MeanDetectedCNO; //Mean CNO Detected satelite
	tempuint8_2^=tempuint8_1;
	cam.tele_putc_new(tempuint8_1);
	tempuint8_1 = gps.MeanFixCNO; // Mean CNO Fixed satelite
	tempuint8_2^=tempuint8_1;
	tempuint8_2^=0x0D;
	cam.tele_putc_new(tempuint8_1);
	cam.tele_putc_new(tempuint8_2);
}

