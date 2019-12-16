/*Simple Arduino Library for Mini JPEG Camera VC0706
 By: Fransiskus Yoga Esa Wibowo
 ITB Komurindo-Kombat Team 
 Date: 
 
 
 Any other settings need special calibration.
*/
#include "mbed.h"
#include "FATFileSystem.h"
#include "SDBlockDevice.h"
#include <stdio.h>
#include <errno.h>
// mbed_retarget.h is included after errno.h so symbols are mapped to
// consistent values for all toolchains 
#include "platform/mbed_retarget.h"

#include "BufferedSerial.h"

#ifndef VC0706_h
#define VC0706_h

#define PIC_BUF_LEN 200
#define PIC_NAME_LEN 20

class VC0706
{
  public:
    VC0706(PinName tx, PinName rx, PinName tele_tx=USBTX, PinName tele_rx=USBRX);
	void restart();
	void compress();
	void pause();
	void resume();
	void nextpic();
	uint32_t getSize();
	uint32_t photo(int number);
	virtual void insertedTask(){};//implemented in main program
	BufferedSerial pc;
	void tele_putc_new(int c);
  private:
	//Unstated but important variable in SD card library
	SDBlockDevice sd;
	FATFileSystem fs;
	//file object
	FILE* sdcard;
	//Serial object
	BufferedSerial camserial;
	//Command list
	char cmdRST[4] = {
		0x56,0x00,0x26,0x00};        //Reset command
	char cmdCompress[10] = {
	  0x56,0x00,0x31,0x05,0x01,0x01,0x12,0x04,0xff}; //compress maksimal(0xff)
	char cmdCAP[5] = {      
	  0x56,0x00,0x36,0x01,0x00};   //pause video
	char cmdGetLen[5] = {
	  0x56,0x00,0x34,0x01,0x00};   //Read JPEG file size command
	char cmdGetDat[16] = {           
	  0x56,0x00,0x32,0x0c,0x00,0x0a,0x00,0x00,0x08,0x00,0x00,0x00,0x00,0x00,0x00,0x00}; //Read JPEG data command based PIC_BUF_LENGTH
	char cmdConti[5] = {           
	  0x56,0x00,0x36,0x01,0x03};
	char cmdNextPic[5] = {           
	  0x56,0x00,0x36,0x01,0x02};
	char cmdImageSize[10] = {           
	  0x56,0x00,0x31,0x05,0x04,0x01,0x00,0x19,0x22};	  //resume video
	//private function
	void getData (char name[PIC_NAME_LEN]);
	void readCamSaveToFile(FILE* myFile,int toBeReadLen);
	uint32_t Size = 0;
	char picName[PIC_NAME_LEN];// nama file yg mau di write
	//dynamic tele speed
	bool photo_mode;
	int tele_count;
};
#endif

