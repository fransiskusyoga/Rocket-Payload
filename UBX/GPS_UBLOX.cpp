/*
	GPS_UBLOX.cpp - Ublox GPS library for Arduino
	Code by Jordi Mu�oz and Jose Julio. DIYDrones.com
	This code works with boards based on ATMega168/328 and ATMega1280 (Serial port 1)

	This library is free software; you can redistribute it and/or
    modify it under the terms of the GNU Lesser General Public
    License as published by the Free Software Foundation; either
    version 2.1 of the License, or (at your option) any later version.

	GPS configuration : Ublox protocol
	Baud rate : 38400
	Active messages : 
		NAV-POSLLH Geodetic Position Solution, PAGE 66 of datasheet
		NAV-VELNED Velocity Solution in NED, PAGE 71 of datasheet
		NAV-STATUS Receiver Navigation Status
		  or 
		NAV-SOL Navigation Solution Information

	Methods:
		Init() : GPS Initialization
		Read() : Call this funcion as often as you want to ensure you read the incomming gps data
		
	Properties:
		Lattitude : Lattitude * 10000000 (long value)
		Longitude : Longitude * 10000000 (long value)
		Altitude :  Altitude * 100 (meters) (long value)
		Ground_speed : Speed (m/s) * 100 (long value)
		Ground_course : Course (degrees) * 100 (long value)
		NewData : 1 when a new data is received.
		          You need to write a 0 to NewData when you read the data
		Fix : 1: GPS FIX, 0: No Fix (normal logic)
			
*/

#include "GPS_UBLOX.h"

template<typename T> inline const T _abs(T const & x)
{
    return ( x<0 ) ? -x : x;
}

// Constructors ////////////////////////////////////////////////////////////////
GPS_UBLOX::GPS_UBLOX(PinName tx, PinName rx, int baud)
	:_gps(tx, rx,2048,1)
{
	ck_a=0;
	ck_b=0;
	UBX_step=0;
	Fix=0;
	timestamps=0;
	GPS_timer.start();   //Start Timer
	_gps.baud(baud);
}


// Public Methods //////////////////////////////////////////////////////////////

// optimization : This code don�t wait for data, only proccess the data available
// We can call this function on the main loop (50Hz loop)
// If we get a complete packet this function calls parse_ubx_gps() to parse and update the GPS info.
void GPS_UBLOX::Read(void)
{
  uint8_t data;
  int numc;
  
  numc = _gps.readable();
  
  if (numc > 0)
    for (int i=0;i<numc;i++)  // Process bytes received
      {
	  data = _gps.getc();
      switch(UBX_step)     //Normally we start from zero. This is a state machine
      {
      case 0:  
        if(data==0xB5)  // UBX sync char 1
          UBX_step++;   //OH first data packet is correct, so jump to the next step
        break; 
      case 1:  
        if(data==0x62)  // UBX sync char 2
          UBX_step++;   //ooh! The second data packet is correct, jump to the step 2
        else 
          UBX_step=0;   //Nop, is not correct so restart to step zero and try again.     
        break;
      case 2:
        UBX_class=data;
        ubx_checksum(UBX_class);
        UBX_step++;
        break;
      case 3:
        UBX_id=data;
        ubx_checksum(UBX_id);
        UBX_step++;
        break;
      case 4:
        UBX_payload_length_hi=data;
        ubx_checksum(UBX_payload_length_hi);
        UBX_step++;
        break;
      case 5:
        UBX_payload_length_lo=data;
        ubx_checksum(UBX_payload_length_lo);
		((uint8_t*)(&UBX_payload_length))[0] = UBX_payload_length_hi;
		((uint8_t*)(&UBX_payload_length))[1] = UBX_payload_length_lo;
        UBX_step++;
		UBX_payload_counter=0;
		// We check if the payload lenght is valid...
		if (UBX_payload_length>=UBX_MAXPAYLOAD)
        {
		  UBX_step=0;   //Bad data, so restart to step zero and try again.     
          ck_a=0;
          ck_b=0;
        }
        break;
      case 6:         // Payload data read...
	if (UBX_payload_counter < UBX_payload_length)  // We stay in this state until we reach the payload_length
        {
          UBX_buffer[UBX_payload_counter] = data;
          ubx_checksum(data);
          UBX_payload_counter++;
          if (UBX_payload_counter==UBX_payload_length)
            UBX_step++;
        }
        break;
      case 7:
        UBX_ck_a=data;   // First checksum byte
        UBX_step++;
        break;
      case 8:
        UBX_ck_b=data;   // Second checksum byte
       
	  // We end the GPS read...
        if((ck_a==UBX_ck_a)&&(ck_b==UBX_ck_b))  // Verify the received checksum with the generated checksum.. 
	  	{
	  		parse_ubx_gps();               // Parse the new GPS packet
        }
        // Variable initialization
        UBX_step=0;
        ck_a=0;
        ck_b=0;
        timestamps = GPS_timer.read_ms(); //Restarting timer...
        break;
	  }
    }    // End for...
  // If we don�t receive GPS packets in 2 seconds => Bad FIX state
  if ((GPS_timer.read_ms() - timestamps)>2000)
    {
	Fix = 0;
	}
}

/****************************************************************
 * 
 ****************************************************************/
// Private Methods //////////////////////////////////////////////////////////////
void GPS_UBLOX::parse_ubx_gps(void)
{
  	//Verifing if we are in class 1, you can change this "IF" for a "Switch" in case you want to use other UBX classes.. 
	//In this case all the message im using are in class 1, to know more about classes check PAGE 60 of DataSheet.
  	if(UBX_class==0x01) 
	{
		switch(UBX_id)//Checking the UBX ID
    	{
    		//ID NAV-POSLLH
    		case 0x02: 	Longitude = join_4_bytes(&UBX_buffer[4]); // lon*10000000
      					Lattitude = join_4_bytes(&UBX_buffer[8]); // lat*10000000
      					Altitude = join_4_bytes(&UBX_buffer[16]);  // MSL heigth mm
      					break;
      		//ID NAV-STATUS
      		case 0x03: 	Fix = UBX_buffer[4];
						break;
			//ID NAV-SVINFO
			case 0x30: 	uint8_t TotDetectedCNO=0; //carier to noise ratio
						uint8_t TotFixCNO=0;
						uint8_t TotDetectedQuality=0;
						uint8_t TotFixQuality=0;
						NumFixSat=0;
						NumDetectedSat = (UBX_payload_length-8)/12;
						for (int i =0; i<NumDetectedSat; i++)
						{
							TotDetectedCNO+=UBX_buffer[12+12*i];
							TotDetectedQuality+=UBX_buffer[11+12*i];
							if ((UBX_buffer[10+12*i]&0x01)==1)
							{
								TotFixCNO+=UBX_buffer[12+12*i];
								TotFixQuality+=UBX_buffer[11+12*i];
								NumFixSat++;
							}
						}
						MeanDetectedCNO = TotDetectedCNO/NumDetectedSat;
						MeanFixCNO = TotFixCNO/NumFixSat;
						MeanDetectedQuality = TotDetectedQuality/NumDetectedSat;
						MeanFixQuality = TotFixQuality/NumFixSat;
    	}
    }   
}


/****************************************************************
 * 
 ****************************************************************/
 // Join 4 bytes into a long
long GPS_UBLOX::join_4_bytes(unsigned char Buffer[])
{
  union long_union {
	int32_t dword;
	uint8_t  byte[4];
} longUnion;

  longUnion.byte[0] = *Buffer;
  longUnion.byte[1] = *(Buffer+1);
  longUnion.byte[2] = *(Buffer+2);
  longUnion.byte[3] = *(Buffer+3);
  return(longUnion.dword);
}

/****************************************************************
 * 
 ****************************************************************/
// Ublox checksum algorithm
void GPS_UBLOX::ubx_checksum(uint8_t ubx_data)
{
  ck_a+=ubx_data;
  ck_b+=ck_a; 
}
