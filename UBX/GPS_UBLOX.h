#ifndef GPS_UBLOX_h
#define GPS_UBLOX_h

#include <inttypes.h>
#include "mbed.h"
#include "BufferedSerial.h"

//Protocol header checking 
#define HEADER_NAV_1 0xB5
#define HEADER_NAV_2 0x62
#define HEADER_NAV_3 0x01
#define HEADER_NAV_POSLLH 0x02
#define HEADER_NAV_STATUS 0x03

//GPS Status
#define GPS_NO_FIX 0x00
#define GPS_DEAD_RECONING_ONLY 0x01
#define GPS_2D_FIX 0x02
#define GPS_3D_FIX 0x03
#define GPS_DEAD_RECONING_COMBINED 0x04
#define GPS_TIME_ONLY_FIX 0x05

#define UBX_MAXPAYLOAD 500

class GPS_UBLOX
{
  private:
    // Internal variables
	uint8_t ck_a;     // Packet checksum
	uint8_t ck_b;
	uint8_t UBX_step;
	uint8_t UBX_class;
	uint8_t UBX_id;
	uint8_t UBX_payload_length_hi;
	uint8_t UBX_payload_length_lo;
	uint16_t UBX_payload_length;
	uint16_t UBX_payload_counter;
	uint8_t UBX_buffer[UBX_MAXPAYLOAD];
	uint8_t UBX_ck_a;
	uint8_t UBX_ck_b;
	Timer GPS_timer;
	long timestamps;
	void parse_ubx_gps();
	void ubx_checksum(unsigned char ubx_data);
	long join_4_bytes(unsigned char Buffer[]);	
  public:
  	BufferedSerial _gps;
    // Methods
	GPS_UBLOX(PinName tx, PinName rx, int baud=115200);
	void Read();			// Properties
	int32_t Lattitude;		// Geographic coordinates
	int32_t Longitude;
	int32_t Altitude;
	uint8_t Fix;        	// 3:3D GPS FIX, 2:2D Fix, 1:Invalid, 0:No FIX (normal logic)
	uint8_t MeanDetectedCNO; //carier to noise ratio
	uint8_t MeanFixCNO;
	uint8_t NumDetectedSat;
	uint8_t NumFixSat;
	uint8_t MeanDetectedQuality;
	uint8_t MeanFixQuality;
};

#endif
