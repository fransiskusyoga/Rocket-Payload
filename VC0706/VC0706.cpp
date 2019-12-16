#include "VC0706.h"
VC0706::VC0706(PinName tx, PinName rx, PinName tele_tx, PinName tele_rx):
	sd(MBED_CONF_SD_SPI_MOSI, MBED_CONF_SD_SPI_MISO, MBED_CONF_SD_SPI_CLK, MBED_CONF_SD_SPI_CS),
	fs("sd", &sd),
	pc(tele_tx,tele_rx),
	camserial(tx, rx)
{
	//Set Serial Speed
	camserial.baud(57600);
	pc.baud(57600);

	//Serial setiing
	//If you change the file name, you also need to change pic name in VC0706:photo()
	picName[0]='/';
	picName[1]='s';
	picName[2]='d';
	picName[3]='/';
	picName[4]='p';
	picName[5]='i';
	picName[6]='c';
	picName[7]='0';
	picName[8]='0';
	picName[9]='.';
	picName[10]='j';
	picName[11]='p';
	picName[12]='g';
	picName[13]='\0';
	
	restart();
	compress();
	wait(0.2);
	pause();
	
	photo_mode = false;
}

void VC0706::restart()
{
	char temp;
	int i=0;
	//pc.printf("Res"); //debuging
	camserial.write(cmdRST,sizeof(cmdRST)); //sending command restart
	while (1)
	{
		if (camserial.readable())
		{
			temp=camserial.getc();
			if(temp=='e' && i==0) i=1;
			else if (temp=='n' && i==1) i=2;
			else if (temp=='d' && i==2) i=3;
			else if (temp=='\r' && i==3) i=4;
			else if (temp=='\n' && i==4) break;
			else i=0;
			//pc.printf("%d",i); //debug
		}
	}
	//pc.printf("tart\r\n"); // debuging
}

void VC0706::compress()
{
	char temp;
	int i=0;
	//pc.printf("compr");
	camserial.write(cmdCompress,sizeof(cmdCompress)); //sending command compress
	//Detect ending 0x76, 0x00, 0x31, 0x00, 0x00
	while(1) //wait the answer (5 byte)
	{
		if (camserial.readable())
		{
			temp=camserial.getc();
			if(temp==0x76 && i==0) i=1;
			else if (temp==0x00 && i==1) i=2;
			else if (temp==0x31 && i==2) i=3;
			else if (temp==0x00 && i==3) i=4;
			else if (temp==0x00 && i==4) break;
			else i=0;
			//pc.printf("%d",i);
		}
	}
	//pc.printf("essed\r\n");//debuging
}

void VC0706::pause()
{
	char temp;
	int i=0;
	//pc.printf("pau"); //debugging
	camserial.write(cmdCAP,sizeof(cmdCAP)); //sending command compress
	//Detect ending 0x76, 0x00, 0x36, 0x00, 0x00
	while(1) //wait the answer (5 byte)
	{
		if (camserial.readable())
		{
			temp=camserial.getc();
			if(temp==0x76 && i==0) i=1;
			else if (temp==0x00 && i==1) i=2;
			else if (temp==0x36 && i==2) i=3;
			else if (temp==0x00 && i==3) i=4;
			else if (temp==0x00 && i==4) break;
			else i=0;
			//pc.printf("%d",i);
		}
	}
	//pc.printf("sed\r\n");//debuging
}

void VC0706::resume()
{
	char temp;
	int i=0;
	//pc.printf("res");
	camserial.write(cmdConti,sizeof(cmdConti)); //sending command compress
	//Detect ending 0x76, 0x00, 0x36, 0x00, 0x00
	while(1) //wait the answer (5 byte)
	{
		if (camserial.readable())
		{
			temp=camserial.getc();
			if(temp==0x76 && i==0) i=1;
			else if (temp==0x00 && i==1) i=2;
			else if (temp==0x36 && i==2) i=3;
			else if (temp==0x00 && i==3) i=4;
			else if (temp==0x00 && i==4) break;
			else i=0;
			//pc.printf("%d",i);
		}
	}
	//pc.printf("umed\r\n");//debuging
}

void VC0706::nextpic()
{
	char temp;
	int i=0;
	//pc.printf("next");
	camserial.write(cmdNextPic,sizeof(cmdNextPic)); //sending command next picture
	//Detect ending 0x76, 0x00, 0x36, 0x00, 0x00
	while(1) //wait the answer (5 byte)
	{
		if (camserial.readable())
		{
			temp=camserial.getc();
			if(temp==0x76 && i==0) i=1;
			else if (temp==0x00 && i==1) i=2;
			else if (temp==0x36 && i==2) i=3;
			else if (temp==0x00 && i==3) i=4;
			else if (temp==0x00 && i==4) break;
			else i=0;
			//pc.printf("%d",i);
		}
	}
	//pc.printf(" picure\r\n");//debuging
}

uint32_t VC0706::getSize()//get the data size
{
	wait(0.1);
	uint32_t picTotalLen; // total byte
	char temp;
	int i=0;
	//pc.printf("size");
	camserial.write(cmdGetLen,sizeof(cmdGetLen)); //sending command compress
	//recieved 76 00 34 00 04 00 00 xx yy, detected ending 0x34, 0x00, 0x04, 0x00, 0x00
	while(1) //wait the answer (5 byte)
	{
		if (camserial.readable())
		{
			temp=camserial.getc();
			if(temp==0x34 && i==0) i=1;
			else if (temp==0x00 && i==1) i=2;
			else if (temp==0x04 && i==2) i=3;
			else if (temp==0x00 && i==3) i=4;
			else if (temp==0x00 && i==4) break;
			else i=0;
			//pc.printf("%d",i);
		}
	}
	while(!camserial.readable());  //wait until the data readable
	int high = camserial.getc();   // baca high byte (xx)
	while(!camserial.readable());
	int low  = camserial.getc();   // bava low byte  (yy)
	picTotalLen = high*256 + low;  //get the length of picture
	
	//pc.printf("%d\r\n",picTotalLen);//debuging
	return(picTotalLen);
}
void VC0706::getData (char name[PIC_NAME_LEN])//save data to sd card 
{
	//pc.printf("Photo "); //debigging
	
	uint32_t addr = 0; //address dari file yg mo diambil
	cmdGetDat[8] = 0; //address awal 0
	cmdGetDat[9] = 0; //address awal 0
	
	uint32_t count = Size / PIC_BUF_LEN; //berapa kali ngambil data (size / buffer size)
	char tail = Size % PIC_BUF_LEN; //sisa
	int sisa = Size % PIC_BUF_LEN; //sisa
	
	cmdGetDat[13] = PIC_BUF_LEN; //the length of each read data

	sdcard = fopen(picName, "w+"); //open file
	wait(0.01);
	if(!sdcard){
		pc.printf("myFile open fail...\n");
		return;
	}
	
	for(uint32_t i = 0; i < count; i++) //get and save count*PIC_BUF_LEN data
	{	
		//sending command to get data as big as buffer
		camserial.write(cmdGetDat,sizeof(cmdGetDat)); 
		
		//picture header
		tele_putc_new(0x27);tele_putc_new(0x04);tele_putc_new(0x19);tele_putc_new(0x96);
		//save buffer to sd card 
		readCamSaveToFile(sdcard, PIC_BUF_LEN); 
		//picture tail
		tele_putc_new(0x01);tele_putc_new(0x22);tele_putc_new(0x02);tele_putc_new(0x07);
		
		//address increment
		addr += PIC_BUF_LEN; //buat address yg selanjutnya
		cmdGetDat[8] = addr >> 8;  // buat address yg selanjutnya
		cmdGetDat[9] = addr & 0x00FF;  // buat address yg selanjutnya
		//pc.printf("c%d\n",i);
		
		insertedTask();
	}
	
	cmdGetDat[13] = tail;	 //get rest part of the pic data
	
	//sending command to get the rest of data
	camserial.write(cmdGetDat,sizeof(cmdGetDat));
	
	//picture header
	tele_putc_new(0x27);tele_putc_new(0x04);tele_putc_new(0x19);tele_putc_new(0x96);
	//save buffer to sd card
	readCamSaveToFile(sdcard, sisa); 
	//picture tail
	tele_putc_new(0x03);tele_putc_new(0x19);tele_putc_new(0x04);tele_putc_new(0x95);
	
	// file close to make sure all data written to sd card
	fclose(sdcard); 
	//pc.printf("saved\n"); //debuging
}

void VC0706::readCamSaveToFile(FILE* myFile,int toBeReadLen)
{
	char temp;
	int i=0;
	//emptying the buffer from unecessary byte (protokol data)(76 00 32 00 00 - data - 76 00 32 00 00)
	int readLen = 0;
	
	while(1) //wait the answer (5 byte)
	{
		if (camserial.readable())
		{
			temp=camserial.getc();
			if(temp==0x76 && i==0) i=1;
			else if (temp==0x00 && i==1) i=2;
			else if (temp==0x32 && i==2) i=3;
			else if (temp==0x00 && i==3) i=4;
			else if (temp==0x00 && i==4) break;
			else i=0;
			//pc.printf("a%d",i);
		}
	}
	
	while(readLen < toBeReadLen)//read and store the JPG data that starts with 0xFF 0xDB to sd card
	{
		while(!camserial.readable());
		temp=camserial.getc();
		tele_putc_new(temp);
		fprintf(myFile, "%c", temp);
		readLen++;
	}
	
	//reset flag
	i=0;
	
	//emptying the buffer from unecessary byte (protokol data) (76 00 32 00 00 - data - 76 00 32 00 00)
	while(1) //wait the answer (5 byte)
	{
		if (camserial.readable())
		{
			temp=camserial.getc();
			if(temp==0x76 && i==0) i=1;
			else if (temp==0x00 && i==1) i=2;
			else if (temp==0x32 && i==2) i=3;
			else if (temp==0x00 && i==3) i=4;
			else if (temp==0x00 && i==4) break;
			else i=0;
			//pc.printf("b%x",temp);
		}
	}

}

uint32_t VC0706::photo(int number)
{
	photo_mode = true;
	picName[7] = number/10 + '0';
	picName[8] = number%10 + '0';
	//Serial.println(picName); //debbug
	nextpic();
	wait_ms(100);
	Size = getSize(); 
	getData(picName);
	photo_mode = false;
	return(Size); 
}

void VC0706::tele_putc_new(int c)
{
	tele_count++;
	if (photo_mode)
	{
		if (tele_count>=200)
		{
			wait_ms(250);
			tele_count=0;
			pc.putc(c);
		}
		else 
		{
			pc.putc(c);
		}
	}
	else
	{
		while (pc.writeable()<10);
		pc.putc(c);
	}
}
