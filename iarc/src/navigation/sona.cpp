#include <ros/ros.h>

#include <iostream>
#include <fstream>
#include <string>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <sys/types.h>  
#include <sys/stat.h>  
#include <fcntl.h>  
#include <unistd.h>  
#include <termios.h>  
#include <signal.h>

#include <iarc/sona.h>

using namespace std;

//$ setOpt: config params of ttyUSB device
//! fd:     device id
//! nSpeed: baudrate
//! nBits: 
//! nEvent: stream control
//! nStop:  stop bit
int setOpt(int fd,int nSpeed, int nBits, char nEvent, int nStop)  
{  
    struct termios newtio,oldtio;  

    if(tcgetattr(fd, &oldtio) != 0){   
        perror("ERROR: no device, reset device id");  
        return -1;  
    }
    
    bzero(&newtio, sizeof(newtio));
      
    newtio.c_cflag |= CLOCAL | CREAD;  
    newtio.c_cflag &= ~CSIZE;  
  
    switch(nBits)  
    {  
    case 7:  
        newtio.c_cflag |= CS7;  
        break;  
    case 8:  
        newtio.c_cflag |= CS8;  
        break;  
    }  
  
    switch(nEvent)  
    {  
    case 'O':  
        newtio.c_cflag |= PARENB;  
        newtio.c_cflag |= PARODD;  
        newtio.c_iflag |= (INPCK | ISTRIP);  
        break;  
    case 'E':   
        newtio.c_iflag |= (INPCK | ISTRIP);  
        newtio.c_cflag |= PARENB;  
        newtio.c_cflag &= ~PARODD;  
        break;  
    case 'N':    
        newtio.c_cflag &= ~PARENB;  
        break;  
    }  
  
    switch(nSpeed)  
    {  
    case 2400:  
        cfsetispeed(&newtio, B2400);  
        cfsetospeed(&newtio, B2400);  
        break;  
    case 4800:  
        cfsetispeed(&newtio, B4800);  
        cfsetospeed(&newtio, B4800);  
        break;  
    case 9600:  
        cfsetispeed(&newtio, B9600);  
        cfsetospeed(&newtio, B9600);  
        break;  
    case 115200:  
        cfsetispeed(&newtio, B115200);  
        cfsetospeed(&newtio, B115200);  
        break;  
    case 460800:  
        cfsetispeed(&newtio, B460800);  
        cfsetospeed(&newtio, B460800);  
        break;  
    default:  
        cfsetispeed(&newtio, B9600);  
        cfsetospeed(&newtio, B9600);  
        break;  
    }  
    
    if(nStop == 1)  
        newtio.c_cflag &=  ~CSTOPB;  
    else if(nStop == 2)  
    	newtio.c_cflag |=  CSTOPB;
    	 
    newtio.c_cc[VTIME]  = 0;//重要  
    newtio.c_cc[VMIN] = 100;//返回的最小值重要
      
    tcflush(fd,TCIFLUSH);
      
    if((tcsetattr(fd,TCSANOW,&newtio))!=0)  
    {  
        perror("com set error");  
        return -1;  
    } 
     
	cout << "--------device setup ok-----------" << endl;
	
    return 0;  
}  
  
//$ charToInt: convert char to an interger
int charToInt(char buf)
{
	int highLevel = 0;
	
	if((buf & 0x01) != 0)
		highLevel += 1;
	if((buf & 0x02) != 0)
		highLevel += 2;
	if((buf & 0x04) != 0)
		highLevel += 4;
	if((buf & 0x08) != 0)
		highLevel += 8;
	if((buf & 0x10) != 0)
		highLevel += 16;
	if((buf & 0x20) != 0)
		highLevel += 32;
	if((buf & 0x40) != 0)
		highLevel += 64;
	if((buf & 0x80) != 0)
		highLevel += 128;     
		
	return highLevel;
}

//$: signal subscribe function
typedef void (*signalHandler)(int );

bool loopFlag = true;

//$: signal subscribe function pointer
void signalHandlerFunc(int nSignal){
	loopFlag =false;
	cout << " interupt program: " << nSignal << endl;
}

int main(int argc,char** argv)  
{  
	ros::init(argc,argv,"sona");
	ros::NodeHandle my_node;
	
    int fd1,nset1,nread;  
    char buf[1024];  

  	char inbuf = 0x55;
	//char inbuf = 0x50;
	ofstream out;  
	out.open("sona.txt",ios::out);
	
	if(!out.is_open())
	{
		cout << "file open fail" << endl;
		return -1;
	}
	
	signalHandler pSignal = signalHandlerFunc;
	signal(SIGINT,pSignal);
	
	//& open device by reading and writing mode
    fd1 = open("/dev/ttyUSB0",O_RDWR|O_NONBLOCK);
    if (fd1 == -1){
    	cout << "ttyUSB open fail!" << endl;
        exit(1);
    }
	//& set the ttyUSB configruation: baudrate-9600 
    nset1 = setOpt(fd1,9600, 8, 'N', 1);
    if (nset1 == -1)  
        exit(1);  
    
    iarc::sona sona_height;
    ros::Publisher height_pub = my_node.advertise<iarc::sona>("/iarc/sona", 50);
  
    while(loopFlag)  
    {  
    	
    	write(fd1,&inbuf,1);
    	
    	usleep(25*1000);
    	
        memset(buf,0,2);   
        nread = read(fd1, buf, 2); 
        
        if(nread == 2){
			
			int highLevel = charToInt(buf[0]);
			//cout << highLevel - 45<< endl;
			int lowLevel = charToInt(buf[1]);
        	//cout << highLevel << " " << lowLevel << endl;
        	cout << "height: " << (float)(highLevel*256+lowLevel)/1000.0 << "\t" << (int)buf[0] << "\t" << (int)buf[1]<< endl; 
        	out << (float)(highLevel*256+lowLevel)/1000.0 << "\t" << (int)buf[0] << "\t" << (int)buf[1] << endl;
        	sona_height.height = (float)(highLevel*256+lowLevel)/1000.0;
        	height_pub.publish(sona_height);
        	
        }
        else{
        	cout << "read error: " << nread << endl;
        }
        
        usleep(25*1000);
    }  
    
	out.close();
    close(fd1);
    
    return 0;  
}  
