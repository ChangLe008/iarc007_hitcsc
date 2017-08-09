#include <iostream>
#include <string>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <sys/types.h>  
#include <sys/stat.h>  
#include <fcntl.h>  
#include <unistd.h>  
#include <termios.h>  
#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h> 
#include <fstream>
#include <network_client/Optitrack_data.h>
#include <dji_sdk/GlobalPosition.h>
using namespace std;

sensor_msgs::LaserScan ultra;
ros::Publisher sona_pub;
dji_sdk::GlobalPosition global_position,last_global_position;

//$ setOpt: config params of ttyUSB device
//! fd:     device id
//! nSpeed: baudrate
//! nBits: 
//! nEvent: stream control
//! nStop:  stop bit
ros::Subscriber ultrasonic_subscriber;
ros::Subscriber Opti_pos;
ros::Subscriber g_position;
sensor_msgs::LaserScan ultrasonic;
network_client::Optitrack_data opti_pos;
// float guidance_last = 0.;
// void ultrasonic_subscriber_callback(sensor_msgs::LaserScan g_ul)
// {
//     ultrasonic.ranges.resize(3);
// 	ultrasonic = g_ul;
// 	float ultrasonic_original=ultrasonic.ranges[0];
// 	if(ultrasonic_original<=0.05)
// 	{
// 		ultrasonic.ranges[1]=guidance_last;
// 	}
// 	else
// 	{
// 		guidance_last = ultrasonic_original;
// 		ultrasonic.ranges[1] = ultrasonic_original;
// 	}	
// }

// void opti_position_callback(network_client::Optitrack_data position)
// {
// 	opti_pos = position;               
// }


void g_pose_callback(const dji_sdk::GlobalPosition g_pos)
{
	last_global_position = global_position;
    global_position = g_pos;
}
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
    	 
    newtio.c_cc[VTIME]  = 0;//��Ҫ  
    newtio.c_cc[VMIN] = 100;//���ص���Сֵ��Ҫ
      
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

int main(int argc, char **argv)  
{  
    int fd1,nset1,nread;  
    char buf[1024];  
	ofstream init_log("/home/hitcsc/catkin_ws/log/test_client/sona.txt");
  	char inbuf = 0x55;
    int ulseq = 0;
	//& open device by reading and writing mode
    fd1 = open("/dev/ttyUSB0", O_RDWR|O_NONBLOCK);
    // fd1 = open("/dev/ttyUSB0", O_RDWR);
    if (fd1 == -1)  
        exit(1);  
	//& set the ttyUSB configruation: baudrate-9600 
    nset1 = setOpt(fd1,9600, 8, 'N', 1);
    if (nset1 == -1)  
        exit(1); 

    ros::init(argc, argv, "sona");
    ROS_INFO("top test");
    ros::NodeHandle sona;
    sona_pub = sona.advertise<sensor_msgs::LaserScan>("/ultrasonic",1);
	g_position		= sona.subscribe("/dji_sdk/global_position",10,g_pose_callback);
    // ultrasonic_subscriber = sona.subscribe<sensor_msgs::LaserScan>("/guidance/ultrasonic", 100,ultrasonic_subscriber_callback);
	// Opti_pos      = sona.subscribe("/network_client/network_optitrack_data", 10, opti_position_callback);
    ros::Rate rate(10);
    ultra.header.frame_id = "ultrasonic";
    float last_ul = 0.04;
    float ul = 0.;
    int first_time = 1;
    float bar = 0.;
	while(ros::ok())  
    {  
    	
        ulseq++;
        write(fd1,&inbuf,1);
    	
    	usleep(100*1000);
    	
        memset(buf,0,2);   
        nread = read(fd1, buf, 2);  
        ultrasonic.ranges.resize(2);
        bool wrong=0;
        if(nread == 2)
        {
			int highLevel = charToInt(buf[0]);
			int lowLevel = charToInt(buf[1]);
            ul = (float)(highLevel*256+lowLevel)/1000.0;
            if(first_time)
            {
                first_time = 0;
                last_ul = ul;
            }
            else
            {
                if((abs(last_ul-ul)>=0.1 || ul >= 3.) && last_ul!=0)
                {
                    ul = last_ul+10./3.*(global_position.altitude-last_global_position.altitude);
                    wrong  = 1;
                    cout << "too big change "<< (ul > 3.)*1;
                    if(last_ul<=0.05)
                        cout << "error1"<<endl;			
                }
            }
        }
        else
        {
        	cout << "read error: " << nread << endl;
            ul = last_ul+10./3.*(global_position.altitude-last_global_position.altitude);
            // init_log << ul <<"\t"<<  opti_pos.posz<< endl;
        }
        ultra.ranges.resize(1);
        ultra.ranges[0] = ul;
        ultra.header.stamp  = ros::Time::now();
        ultra.header.seq    = ulseq;
        bar +=10./3.*(global_position.altitude-last_global_position.altitude);
        init_log << ul <<"\t"<<   bar <<"\t"<<wrong<< std::endl;
        sona_pub.publish(ultra);
        last_ul = ul;

        ros::spinOnce();
		rate.sleep();
    }  
    init_log.close();
    close(fd1);
    
    return 0;  
}  
