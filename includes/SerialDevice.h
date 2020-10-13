#ifndef __SERIALDEVICE_H__
#define __SERIALDEVICE_H__ 1


//StaticFactory Singleton Pattern with Final Specifier makes the class Factory Sealed, client/customer should use the factory product 
//without any modifications , No Interfaces and overrides single instance per device i.e for /dev/ttyUSB0..9 or /dev/ttyS0..9 across the system scope
//Lock file with the used interface is created in /tmp folder
//Author: Satya 


#include <termios.h>
#include <fcntl.h>
#include <stdio.h>
#include <poll.h>
#include <assert.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <sys/types.h>
#include <stdint.h>
#include <linux/serial.h>
#include <unistd.h>
#include <regex.h>
#include <errno.h>
#include <iostream>
#include <fstream>
#include <exception>
#include <cassert>
#include <string>
#include <cstring>
#include <stdexcept>
#include <typeinfo>
#include <vector>
#include <algorithm>



namespace SerialDevice {
//#defines are not encouraged but they are handy

#define SUCCESS 0
#define FAILURE 1
#define DATA_READY 1
#define NODATA -1
#define LOCKFILE_DIR "tmp"
#define LOCKFILE_EXT ".LCK"
#define ErrInputDevice "Invalid Input Device Received...! => "
#define ErrDeviceBusy  "Device is Active and Busy ...! => "
#define ErrNoMemory    "No Memory Allocated ...!"
#define ErrDevOpen     "Unable to Open Device ...!"
#define ErrGetPort     "Unable to Get Settings Reason: "
#define ErrSetPort     "Unable to Set Settings Reason: "


using namespace std;
class SerialDevice final {

	
	public:
		typedef uint16_t deviceHandle_t;
		typedef uint16_t returnCode_t;
		~SerialDevice();
		
	
		static uint16_t ValidateInputStringPattern(const char *input,const char *pattern);
		static SerialDevice* GetDeviceHandle(const char *deviceName, string&  errorString);	
		returnCode_t SetBaudRate(const uint32_t baudRate,string&  errorString);
		size_t ReceiveResponse(void *responsebuffer,size_t respLen=256,uint16_t maxTimeoutSeconds=1);  
			//arg1-> input responsebuffer to fill 
			//arg2-> Max expected data size to receive , default was 256 but may be less or greater based on the baudrate
			//arg3-> Specify Timeout in seconds i.e maximum timeout in between each byte based on the baudrate

		size_t SendCommand(const void *command,uint16_t cmdLen=256);
	
	

	private: 
		
		string m_prLockFile;
		struct pollfd m_prPollmsp;
		

		int  m_prDeviceFD;


		static SerialDevice *m_pdeviceHandle;
	

		SerialDevice() {
		}
		//SerialDevice(const SerialDevice&);
		//SerialDevice operator=(const SerialDevice);

		



};

}

typedef SerialDevice::SerialDevice SerialPort;
using namespace std;

#endif 
