#include <ros/ros.h>
#include <stdio.h>
#include <cstdlib>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <stdlib.h>
#include <string.h>
#include <errno.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>

#include <network_client/Optitrack.h>
#include <network_client/Optitrack_data.h>


ros::Publisher  network_cmd_pub;
ros::Publisher  network_data_pub;

/**TCP/IP Data Trans**/
union Receive
{
	char chRecBuf[256];
	float flRecBuf[64];
};

union test
{
	char a[2];
	short data;
};
typedef enum
{
	NET_LISTENING,
	NET_GETHEAD,
	NET_GETPACKSTART,
	NET_GETDATA,
	NET_GETEND,
	NET_ERROR,
	NET_RETRY
}EN_NETLINK_STATE;

EN_NETLINK_STATE enNetLinkState;
test testdata;
typedef struct
{
	unsigned char Head;
	unsigned char Len;
	unsigned char Seq;
	unsigned char SysID;
	unsigned char CompID;
	unsigned char MsgID;
	unsigned char CheckA;
	unsigned char CheckB;
	Receive Data;
	unsigned char End;
}NETLINK_DATA;
NETLINK_DATA net_link_data;
NETLINK_DATA net_link_data_prv;
NETLINK_DATA net_link_data_send;
char RecvBuf[256];
char SendBuf[265];
bool net_firstrun;

float delaytime;
int sockClient;
sockaddr_in addrServer;

/***Velt Filter***/
#define N_FILTER_X 5
#define N_FILTER_Y 5
#define N_FILTER_Z 5
#define FILTER_DATA_MAX 256
typedef struct
{
	bool First;
	int N;
	int Num;
	float Output;
	float Sum;
	float Velt[FILTER_DATA_MAX];
}VELT_FILTER;
VELT_FILTER VeltFilterX = {1,N_FILTER_X,0,0,0,{0}};
VELT_FILTER VeltFilterY = {1,N_FILTER_Y,0,0,0,{0}};
VELT_FILTER VeltFilterZ = {1,N_FILTER_Z,0,0,0,{0}};


network_client::Optitrack optitrack_cmd;
network_client::Optitrack_data optitrack_data;

void Init_Internet_Client(void);
void Network_CMD_Publish(void);
void Network_Data_Publish(void);
float MovingAvgFilter(VELT_FILTER *pVeltFilter,float VeltTemp);

int main(int argc, char **argv) 
{

	ros::init(argc, argv, "network_client");
	ROS_INFO("network_client start");

	ros::NodeHandle m_network_nh;

	network_cmd_pub	= m_network_nh.advertise<network_client::Optitrack>("/network_client/network_cmd", 10);
	network_data_pub = m_network_nh.advertise<network_client::Optitrack_data>("/network_client/network_optitrack_data", 10);

	Init_Internet_Client();

	while(ros::ok())
	{
		ros::spinOnce();
		if(enNetLinkState == NET_LISTENING)
		{
			int Recv = recv(sockClient,RecvBuf,1,0);
	        if(Recv > 0)
	        {
	        	net_link_data.Head = RecvBuf[0];
	        	if(net_link_data.Head == 0xFE)
	        	{
	        		enNetLinkState = NET_GETHEAD;
	        		//ROS_INFO("NET_GETHEAD");
	        	}
	        }
	        else if(Recv == 0)
	        {
	        	enNetLinkState = NET_ERROR;
	        }
		}
		else if(enNetLinkState == NET_GETHEAD)
		{
			int Recv = recv(sockClient,RecvBuf,5,0);
	        if(Recv > 0)
	        {
	        	net_link_data.Len = RecvBuf[0];
	        	net_link_data.Seq = RecvBuf[1];
	        	net_link_data.SysID = RecvBuf[2];
	        	net_link_data.CompID = RecvBuf[3];
	        	net_link_data.MsgID = RecvBuf[4];
	        	enNetLinkState = NET_GETPACKSTART;
	        	//ROS_INFO("NET_GETPACKSTART");
	        }
	        else if(Recv == 0)
	        {
	        	enNetLinkState = NET_ERROR;
	        }
		}
		else if(enNetLinkState == NET_GETPACKSTART)
		{
			int Recv = recv(sockClient,RecvBuf,net_link_data.Len,0);
	        if(Recv > 0)
	        {
	        	for(int data_i=0;data_i<net_link_data.Len;data_i++)
	        	{
	        		net_link_data.Data.chRecBuf[data_i] = RecvBuf[data_i];
	        	}
	        	enNetLinkState = NET_GETDATA;
	        	//ROS_INFO("NET_GETDATA");
	        }
	        else if(Recv == 0)
	        {
	        	enNetLinkState = NET_ERROR;
	        }
		}
		else if(enNetLinkState == NET_GETDATA)
		{
			int Recv = recv(sockClient,RecvBuf,3,0);
	        if(Recv > 0)
	        {
	        	net_link_data.CheckA = RecvBuf[0];
	        	net_link_data.CheckB = RecvBuf[1];
	        	net_link_data.End = RecvBuf[2];
	        	if(net_link_data.End == 0xAA)
	        	{
	        		enNetLinkState = NET_GETEND;
	        		//ROS_INFO("NET_GETEND");
	        		if(net_link_data.MsgID == 0x05)
					{
	        			if(net_firstrun == 1)
	        			{
	        				optitrack_data.lost_data = 0;
	        				net_firstrun = 0;
	        			}
	        			else
	        			{
							if(net_link_data.Seq == 0)
							{
								if(net_link_data_prv.Seq != 255)
								{
									ROS_WARN("Lose data:%d!",net_link_data.Seq - net_link_data_prv.Seq + 254);
									optitrack_data.lost_data = net_link_data.Seq - net_link_data_prv.Seq + 254;
								}
								else
								{
									optitrack_data.lost_data = 0;
								}
							}
							else
							{
								if(net_link_data.Seq - net_link_data_prv.Seq != 1)
								{
									ROS_WARN("Lose data:%d!",net_link_data.Seq - net_link_data_prv.Seq - 1);
									optitrack_data.lost_data = net_link_data.Seq - net_link_data_prv.Seq - 1;
								}
								else
								{
									optitrack_data.lost_data = 0;
								}
							}
	        			}
					}
	        	}
	        	else
	        	{
	        		ROS_WARN("Netdata is error!");
	        		enNetLinkState = NET_LISTENING;
	        	}
	        }
	        else if(Recv == 0)
	        {
	        	enNetLinkState = NET_ERROR;
	        }
		}
        else if(enNetLinkState == NET_GETEND)
        {
        	if(net_link_data.MsgID == 0x05)
        	{
        		net_link_data_prv = net_link_data;
        	}
			optitrack_cmd.health_flag = 1;
			optitrack_data.network_health_flag = 1;
			if(net_link_data.MsgID == 0x01) //Velt
			{
				optitrack_cmd.cmdtype = 1;
				optitrack_cmd.x = net_link_data.Data.flRecBuf[0];
				optitrack_cmd.y = net_link_data.Data.flRecBuf[1];
				optitrack_cmd.z = net_link_data.Data.flRecBuf[2];
				Network_CMD_Publish();
			}
			else if(net_link_data.MsgID == 0x02) //Command
			{
				optitrack_cmd.cmdtype = 2;
				optitrack_cmd.control_cmd = net_link_data.Data.chRecBuf[0];
				Network_CMD_Publish();
			}
			else if(net_link_data.MsgID == 0x03) //Attitude
			{
				optitrack_cmd.cmdtype = 3;
				optitrack_cmd.x = net_link_data.Data.flRecBuf[0];
				optitrack_cmd.y = net_link_data.Data.flRecBuf[1];
				optitrack_cmd.z = net_link_data.Data.flRecBuf[2];
				Network_CMD_Publish();
			}
			else if(net_link_data.MsgID == 0x04) //Pos
			{
				optitrack_cmd.cmdtype = 4;
				optitrack_cmd.x = net_link_data.Data.flRecBuf[0];
				optitrack_cmd.y = net_link_data.Data.flRecBuf[1];
				optitrack_cmd.z = net_link_data.Data.flRecBuf[2];
				Network_CMD_Publish();
			}
			else if(net_link_data.MsgID == 0x05) //data
			{
				optitrack_data.posx = net_link_data.Data.flRecBuf[0];
				optitrack_data.posy = net_link_data.Data.flRecBuf[1];
				optitrack_data.posz = net_link_data.Data.flRecBuf[2];
				optitrack_data.roll = net_link_data.Data.flRecBuf[3];
				optitrack_data.yaw = net_link_data.Data.flRecBuf[4];
				optitrack_data.pitch = net_link_data.Data.flRecBuf[5];
				optitrack_data.veltx = net_link_data.Data.flRecBuf[6];
				optitrack_data.velty = net_link_data.Data.flRecBuf[7];
				optitrack_data.veltz = net_link_data.Data.flRecBuf[8];
				optitrack_data.pendulum_posx = net_link_data.Data.flRecBuf[9];
				optitrack_data.pendulum_posy = net_link_data.Data.flRecBuf[10];
				optitrack_data.pendulum_posz = net_link_data.Data.flRecBuf[11];
				optitrack_data.pendulum_attx = net_link_data.Data.flRecBuf[12];
				optitrack_data.pendulum_atty = net_link_data.Data.flRecBuf[13];
				optitrack_data.delay = net_link_data.Data.flRecBuf[14];

				optitrack_data.health_flag = net_link_data.Data.chRecBuf[76];
				optitrack_data.pendulum_health_flag = net_link_data.Data.chRecBuf[77];

				optitrack_data.filterveltx = net_link_data.Data.flRecBuf[16];
				optitrack_data.filtervelty = net_link_data.Data.flRecBuf[17];
				optitrack_data.filterveltz = net_link_data.Data.flRecBuf[18];


				net_link_data_send.Data.flRecBuf[0] = net_link_data.Data.flRecBuf[15];
				net_link_data_send.Head = 0xFE;
				net_link_data_send.Seq = net_link_data.Seq;
				net_link_data_send.MsgID = 0x07;
				net_link_data_send.Len = 4;
				net_link_data_send.SysID = 0;
				net_link_data_send.CompID = 0;
				net_link_data_send.End = 0xAA;

				SendBuf[0] = net_link_data_send.Head;
				SendBuf[1] = net_link_data_send.Len;
				SendBuf[2] = net_link_data_send.Seq;
				SendBuf[3] = net_link_data_send.SysID;
				SendBuf[4] = net_link_data_send.CompID;
				SendBuf[5] = net_link_data_send.MsgID;

				for(int data_i=0;data_i<net_link_data_send.Len;data_i++)
				{
					SendBuf[6+data_i] = net_link_data_send.Data.chRecBuf[data_i];
				}
				SendBuf[6+net_link_data_send.Len] = net_link_data_send.CheckA;
				SendBuf[7+net_link_data_send.Len] = net_link_data_send.CheckB;
				SendBuf[8+net_link_data_send.Len] = net_link_data_send.End;

				send(sockClient,SendBuf,net_link_data_send.Len+9,0);
				Network_Data_Publish();
			}
			else if(net_link_data.MsgID == 0x06) //delay test
			{
				net_link_data_send.Data.flRecBuf[0] = net_link_data.Data.flRecBuf[0];
				//printf("time is:%f\n",net_link_data.Data.flRecBuf[0]);
				optitrack_data.delay = net_link_data.Data.flRecBuf[1];
				net_link_data_send.Head = 0xFE;
				net_link_data_send.Seq = 0;
				net_link_data_send.MsgID = 0x06;
				net_link_data_send.Len = 4;
				net_link_data_send.SysID = 0;
				net_link_data_send.CompID = 0;
				net_link_data_send.End = 0xAA;

				SendBuf[0] = net_link_data_send.Head;
				SendBuf[1] = net_link_data_send.Len;
				SendBuf[2] = net_link_data_send.Seq;
				SendBuf[3] = net_link_data_send.SysID;
				SendBuf[4] = net_link_data_send.CompID;
				SendBuf[5] = net_link_data_send.MsgID;

				for(int data_i=0;data_i<net_link_data_send.Len;data_i++)
				{
					SendBuf[6+data_i] = net_link_data_send.Data.chRecBuf[data_i];
				}
				SendBuf[6+net_link_data_send.Len] = net_link_data_send.CheckA;
				SendBuf[7+net_link_data_send.Len] = net_link_data_send.CheckB;
				SendBuf[8+net_link_data_send.Len] = net_link_data_send.End;

				send(sockClient,SendBuf,net_link_data_send.Len+9,0);
			}
			else if(net_link_data.MsgID == 0x08) //pid
			{
				FILE *fp;
				fp=fopen("/home/ubuntu/catkin_ws/src/mav_client/src/pid_pos_velt.txt","r+");
				printf("get pid data of pos and velt\n");
				double data_temp;
				for(int i=0;i<net_link_data.Len/4;i++)
				{
					data_temp = (double)net_link_data.Data.flRecBuf[i];
					fprintf(fp,"%f",data_temp);
					fprintf(fp,"%s"," ");
					printf("%f\t",net_link_data.Data.flRecBuf[i]);
					if(i%3 == 2)
					{
						printf("\n");
					}
				}
				printf("\n\n");
				fclose(fp);
				net_link_data_send.Data.chRecBuf[0] = 1;
				net_link_data_send.Head = 0xFE;
				net_link_data_send.Seq = 0;
				net_link_data_send.MsgID = 0x08;
				net_link_data_send.Len = 1;
				net_link_data_send.SysID = 0;
				net_link_data_send.CompID = 0;
				net_link_data_send.End = 0xAA;

				SendBuf[0] = net_link_data_send.Head;
				SendBuf[1] = net_link_data_send.Len;
				SendBuf[2] = net_link_data_send.Seq;
				SendBuf[3] = net_link_data_send.SysID;
				SendBuf[4] = net_link_data_send.CompID;
				SendBuf[5] = net_link_data_send.MsgID;

				for(int data_i=0;data_i<net_link_data_send.Len;data_i++)
				{
					SendBuf[6+data_i] = net_link_data_send.Data.chRecBuf[data_i];
				}
				SendBuf[6+net_link_data_send.Len] = net_link_data_send.CheckA;
				SendBuf[7+net_link_data_send.Len] = net_link_data_send.CheckB;
				SendBuf[8+net_link_data_send.Len] = net_link_data_send.End;

				send(sockClient,SendBuf,net_link_data_send.Len+9,0);
				optitrack_cmd.cmdtype = 2;
				optitrack_cmd.control_cmd = 'n';
				Network_CMD_Publish();
			}
			else if(net_link_data.MsgID == 0x09) //pid of pendulum
			{
				FILE *fp;
				fp=fopen("/home/ubuntu/catkin_ws/src/mav_client/src/pid_pendulum.txt","r+");
				printf("get pid data of pendulum\n");
				double data_temp;
				for(int i=0;i<net_link_data.Len/4;i++)
				{
					data_temp = (double)net_link_data.Data.flRecBuf[i];
					fprintf(fp,"%f",data_temp);
					fprintf(fp,"%s"," ");
					printf("%f\t",net_link_data.Data.flRecBuf[i]);
					if(i%3 == 2)
					{
						printf("\n");
					}
				}
				printf("\n");
				fclose(fp);
				net_link_data_send.Data.chRecBuf[0] = 1;
				net_link_data_send.Head = 0xFE;
				net_link_data_send.Seq = 0;
				net_link_data_send.MsgID = 0x09;
				net_link_data_send.Len = 1;
				net_link_data_send.SysID = 0;
				net_link_data_send.CompID = 0;
				net_link_data_send.End = 0xAA;

				SendBuf[0] = net_link_data_send.Head;
				SendBuf[1] = net_link_data_send.Len;
				SendBuf[2] = net_link_data_send.Seq;
				SendBuf[3] = net_link_data_send.SysID;
				SendBuf[4] = net_link_data_send.CompID;
				SendBuf[5] = net_link_data_send.MsgID;

				for(int data_i=0;data_i<net_link_data_send.Len;data_i++)
				{
					SendBuf[6+data_i] = net_link_data_send.Data.chRecBuf[data_i];
				}
				SendBuf[6+net_link_data_send.Len] = net_link_data_send.CheckA;
				SendBuf[7+net_link_data_send.Len] = net_link_data_send.CheckB;
				SendBuf[8+net_link_data_send.Len] = net_link_data_send.End;

				send(sockClient,SendBuf,net_link_data_send.Len+9,0);
				optitrack_cmd.cmdtype = 2;
				optitrack_cmd.control_cmd = 'm';
				Network_CMD_Publish();
			}
			enNetLinkState = NET_LISTENING;
		}
        else if(enNetLinkState == NET_ERROR)
        {
        	ROS_ERROR("Lost Connect!");
            optitrack_cmd.health_flag = 0;
            optitrack_cmd.cmdtype = 0;
            optitrack_cmd.x = 0;
            optitrack_cmd.y = 0;
            optitrack_cmd.z = 0;
            Network_CMD_Publish();
            optitrack_data.network_health_flag = 0;
            Network_Data_Publish();
            close(sockClient);
            enNetLinkState = NET_RETRY;
        }
        else if(enNetLinkState == NET_RETRY)
        {
        	ROS_INFO("retry connect!");
            optitrack_cmd.health_flag = 0;
            optitrack_cmd.cmdtype = 0;
            optitrack_cmd.x = 0;
            optitrack_cmd.y = 0;
            optitrack_cmd.z = 0;
            Network_CMD_Publish();
            optitrack_data.network_health_flag = 0;
            Network_Data_Publish();
            Init_Internet_Client();
        }
	}
	close(sockClient);
	printf("close client.\n");
}

void Init_Internet_Client(void)
{
	sockClient=socket(AF_INET,SOCK_STREAM,0);
	addrServer.sin_addr.s_addr = inet_addr("192.168.1.105");
	addrServer.sin_family=AF_INET;
	addrServer.sin_port=htons(6000);
	while(ros::ok())
	{
		//printf("connecting....\n");
		int connect_flag = connect(sockClient,(sockaddr*)&addrServer,sizeof(sockaddr));
		if(connect_flag == 0)
		{
			//printf("connect server succeed\n");
			net_firstrun = 1;
			enNetLinkState = NET_LISTENING;
			ROS_INFO("connect server succeed");
			break;
		}
		else
		{
			//printf("connect server failed,retry\n");
		}
	}
}

void Network_CMD_Publish(void)
{
    network_client::Optitrack net_cmd;
    net_cmd.header.frame_id = "network_client";
    net_cmd.header.stamp    = ros::Time::now();

    net_cmd.cmdtype =  optitrack_cmd.cmdtype;
    net_cmd.control_cmd = optitrack_cmd.control_cmd;
    net_cmd.x = optitrack_cmd.x;
    net_cmd.y = optitrack_cmd.y;
    net_cmd.z = optitrack_cmd.z;
    net_cmd.health_flag = optitrack_cmd.health_flag;
    network_cmd_pub.publish(net_cmd);
}

void Network_Data_Publish(void)
{
    network_client::Optitrack_data net_data;
    net_data.header.frame_id = "network_client";
    net_data.header.stamp    = ros::Time::now();

    net_data.posx = optitrack_data.posx;
    net_data.posy = optitrack_data.posy;
    net_data.posz = optitrack_data.posz;
    net_data.roll = optitrack_data.roll;
    net_data.yaw = optitrack_data.yaw;
    net_data.pitch = optitrack_data.pitch;
    net_data.veltx = optitrack_data.veltx;
    net_data.velty = optitrack_data.velty;
    net_data.veltz = optitrack_data.veltz;
    net_data.pendulum_attx = optitrack_data.pendulum_attx;
    net_data.pendulum_atty = optitrack_data.pendulum_atty;
    net_data.pendulum_posx = optitrack_data.pendulum_posx;
    net_data.pendulum_posy = optitrack_data.pendulum_posy;
    net_data.pendulum_posz = optitrack_data.pendulum_posz;
    net_data.filterveltx = optitrack_data.filterveltx;
    net_data.filtervelty = optitrack_data.filtervelty;
    net_data.filterveltz = optitrack_data.filterveltz;
    net_data.health_flag = optitrack_data.health_flag;
    net_data.network_health_flag = optitrack_data.network_health_flag;
    net_data.delay = optitrack_data.delay;
    net_data.pendulum_health_flag = optitrack_data.pendulum_health_flag;

    network_data_pub.publish(net_data);
}

float MovingAvgFilter(VELT_FILTER *pVeltFilter,float VeltTemp)
{
	pVeltFilter->Sum = 0;
	pVeltFilter->Velt[pVeltFilter->Num] = VeltTemp;
	pVeltFilter->Num++;
	if(pVeltFilter->First == 1)
	{
		if(pVeltFilter->Num == pVeltFilter->N)
		{
			pVeltFilter->Num = 0;
			pVeltFilter->First = 0;
		}
		return VeltTemp;
	}
	else
	{
		if(pVeltFilter->Num == pVeltFilter->N)
		{
			pVeltFilter->Num = 0;
		}
	}
	for(int i=0;i<pVeltFilter->N;i++)
	{
		pVeltFilter->Sum = pVeltFilter->Sum + pVeltFilter->Velt[i];
	}

	pVeltFilter->Output = pVeltFilter->Sum / pVeltFilter->N;
	//printf("%f\n",pVeltFilter->Output);
	return pVeltFilter->Output;
}

