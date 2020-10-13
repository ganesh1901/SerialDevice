
#include <SerialDevice.h>
using namespace std;
using namespace SerialDevice;

//Complete Reusable Independent Static Function Can be used by anybody else
uint16_t SerialPort::ValidateInputStringPattern(const char *input,const char *pattern) 
{
	 regex_t frame_expr;
         uint16_t rc=FAILURE;
 

	 rc = regcomp(&frame_expr, pattern, REG_EXTENDED | REG_NOSUB) ;
         if (rc) {
        	rc = FAILURE;	
         } else { // 0 -> success 1 - Fail
        	 rc = regexec(&frame_expr, input, 0, NULL, 0);    
	 }
         regfree(&frame_expr);
	 return(rc);
}

SerialPort::~SerialDevice() {

	close(m_prDeviceFD);
	remove(m_prLockFile.c_str());	
	m_pdeviceHandle = NULL;

}

SerialPort* SerialPort::GetDeviceHandle(const char *dev,string& lastError)
{
	//validates the Input deviceName from /dev/tty[0-9] and /dev/ttyUSB[0-9] Key Pattern for Serial Port
        const char *pattern="^((/dev/)(tty|ttyUSB))([0-9]{1,1})$";
	uint16_t retCode=FAILURE;
        string lock_file=dev;
	


        
	
	retCode=ValidateInputStringPattern(dev,pattern);		
	if((retCode != SUCCESS) || (access(dev,F_OK)!= SUCCESS)) {
		lastError=ErrInputDevice;
		lastError.append(dev);		
		return NULL;
	}

	lock_file.replace(1,3,LOCKFILE_DIR); lock_file.append(LOCKFILE_EXT);


	// Lock File Persistent i.e across system single instance and Handle already opened for the device then return Device Busy Error
	if((access(lock_file.c_str(),F_OK)) == SUCCESS) {
		lastError=ErrDeviceBusy;
		lastError.append(dev);
		return NULL;

	}

	m_pdeviceHandle = new SerialDevice();
	if(m_pdeviceHandle == NULL) {
		lastError=ErrNoMemory;
		return NULL;
	}
	

	if((m_pdeviceHandle->m_prDeviceFD = open (dev, O_RDWR | O_NOCTTY )) < SUCCESS) {
		lastError=ErrDevOpen; lastError.append(dev);
                return NULL;
        }
	
	
	//create Handle and persist system scope if ther was no lock file avaliable for that resource
	if(m_pdeviceHandle->m_prDeviceFD) {
			fstream lock_fp;   lock_fp.open(lock_file,ios::out); lock_fp.close();
	}

	m_pdeviceHandle->m_prLockFile = lock_file;

	return m_pdeviceHandle;
		
}	

SerialPort::returnCode_t SerialPort::SetBaudRate(const uint32_t baudRate, string& lastError) {

	struct termios options;
	struct serial_struct old_serinfo;
	struct serial_struct new_serinfo;
	struct termios old_termios;
	struct termios new_termios;
	returnCode_t retError=false;	
	vector<uint32_t> standardBaudList{110,300,600,1200,2400,4800,9600,14400,19200,28800,38400,56000,57600,115200};
	bool standardBaud=false;



        	tcgetattr(m_prDeviceFD, &options);  
                //cfsetspeed(&options, B38400);        //Default Baud Rate Override with the given baudRate
                cfsetspeed(&options, B115200);        //Default Baud Rate Override with the given baudRate




                options.c_cflag &= ~PARENB;
                options.c_cflag &= ~CSTOPB;
                options.c_cflag &= ~CSIZE;
                options.c_cflag |=  CS8;
                options.c_iflag &= ~ICRNL;
                options.c_cflag &= ~CRTSCTS;
                options.c_lflag &= ~(ICANON | ECHO | ECHOE| ISIG);
                options.c_iflag &= ~(IGNBRK | BRKINT | PARMRK | ISTRIP | INLCR | IGNCR | ICRNL | IXON);
                options.c_oflag &= ~OPOST;
                options.c_lflag &= ~(ECHO | ECHONL | ICANON | ISIG | IEXTEN);
                options.c_cc[VMIN] = 1;
                options.c_cc[VTIME] =100;

		
		//Polling Options
                m_prPollmsp.fd = m_prDeviceFD;
                m_prPollmsp.events = POLLIN;
                tcsetattr(m_prDeviceFD,TCSANOW,&options);
                tcflush(m_prDeviceFD, TCIOFLUSH);


#if 0
		//Setting and Overiding with Specified Baud Rates and  Get old serial settings. 
        	if (ioctl(m_prDeviceFD, TIOCGSERIAL, &old_serinfo) < SUCCESS) {
                	lastError=ErrGetPort; lastError.append(strerror(errno)); retError=errno;
               	
        	} else 
#endif
		{
	

			//verify is the given baudrate was standard or nonstandard
			standardBaud = (find(begin(standardBaudList),end(standardBaudList),baudRate) != end(standardBaudList));

			new_serinfo = old_serinfo;
        	        new_serinfo.custom_divisor = new_serinfo.baud_base / baudRate;
 	        	if(standardBaud) 
			       new_serinfo.flags = (new_serinfo.flags & ~ASYNC_SPD_MASK);
                        else 
          		       new_serinfo.flags = (new_serinfo.flags & ~ASYNC_SPD_MASK) | ASYNC_SPD_CUST; //Specify for Custom BaudRate...!

#if 0

		        // Get Updated serial settings with changes. 
		        if (ioctl(m_prDeviceFD, TIOCSSERIAL, &new_serinfo) < SUCCESS)
		        {
		            lastError=ErrSetPort; lastError.append(strerror(errno)); retError=errno;
		        }
#endif
	
		        if (tcgetattr(m_prDeviceFD, &old_termios) < SUCCESS)
		        {
			    lastError=ErrGetPort; lastError.append(strerror(errno)); retError=errno;
		        }

		        // Change settings to 38400 baud.  The driver will substitute this with the custom baud rate.  
		        new_termios = old_termios;
		        //cfsetispeed(&new_termios, B0);
		        //cfsetospeed(&new_termios, B38400);
		        cfsetspeed(&new_termios, B115200);
		        /* Update terminal attributes. */
		        if (tcsetattr(m_prDeviceFD, TCSANOW, &new_termios) < SUCCESS)
		        {
		            lastError=ErrSetPort; lastError.append(strerror(errno)); retError=errno;
		        }

		
		}


		return(retError);


}


size_t SerialPort::ReceiveResponse(void *responsebuffer,size_t respLen,uint16_t maxTimeoutMillisec) {
	int noof_bytes = 0, cont = 0, Total = 0;
	printf("In Read_port \n");

	tcflush(m_prDeviceFD,TCIOFLUSH);
	
        while(1)
        {
                if(cont == 0) 
                        //poll(&m_prPollmsp, 1, (maxTimeoutSeconds*1000));    //First Data Read 
                        poll(&m_prPollmsp, 1, (maxTimeoutMillisec*2));    //First Data Read  Wait for 2 Times 
                else
                        //poll(&m_prPollmsp, 1, (maxTimeoutSeconds*100));     //Consecutive Read Interval
                        poll(&m_prPollmsp, 1, (maxTimeoutMillisec));     //Consecutive Read Interval

                if((m_prPollmsp.revents & POLLIN) == DATA_READY) {
                        noof_bytes=read(m_prDeviceFD, responsebuffer+Total, respLen);
                        Total+=noof_bytes; cont++;
                }
                else{
                        if(cont == 0) {
                                return NODATA;
                        }
                        break;
                }
        }
        return Total;
}	


size_t SerialPort::SendCommand(const void *cmd,uint16_t Len) {

        if(write(m_prDeviceFD, cmd, Len) == NODATA)                   //pushing data over serial port.
                return NODATA;

        return SUCCESS;
}


#if 0

char buffer[16*1024*1024]; 
int main()
{

    string errorString;    
    SerialPort::returnCode_t ret;	

    try {
	SerialPort *deviceHandle=SerialPort::GetDeviceHandle("/dev/ttyUSB0",errorString);
	if(deviceHandle == NULL) {		
		throw errorString;
	}

	ret = deviceHandle->SetBaudRate(2000000,errorString);
	if(ret != SUCCESS) {
		throw errorString;
	}



	unsigned int bytesReceived=0;
	FILE *fp;


	fp = fopen("/tmp/recpp.log","w");

	assert(fp != NULL);



	while(1) {
  	      bytesReceived = deviceHandle->ReceiveResponse(buffer,256,1); 	
	      if(bytesReceived == -1 ) {
		continue;
	      }
	      if((fwrite(&buffer[0],bytesReceived,1,fp)) != 1) {
                        printf("Unable to Write Data @ i =   ...! \n");
                        perror("Reason:");
                        break;
              }
	      printf("Data Read is this OK  == %ld Bytes \n",bytesReceived);	
	      break;
	
	}

	



	delete deviceHandle;
     } catch (string lastError) {

	cout<<"Exception Raised ....! "<<lastError<<"\n";


     }

}

#endif 
