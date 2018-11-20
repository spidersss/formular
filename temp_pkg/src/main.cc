#include<ros/ros.h>
#include "serial_open.h"
#include<std_msgs/Float64.h>
#include<cstring>

class lower_control
{
public:
	lower_control()
	{
		n.param<std::string>("port_name",port_name,std::string("/dev/ttyUSB0"));
		sub = n.subscribe("lidar_steer",10,&lower_control::Callback,this);
		send_Cmd_timer = n.createTimer(ros::Duration(0.02),&lower_control::send_Cmd_Callback,this);
		
		fd = dev_open(port_name.c_str());
		if(fd < 0)
		{
			ROS_ERROR("open %s failed",port_name.c_str());
			ros::shutdown();
		}
		set_Parity(fd, 8, 1, 'N');
		set_speed(fd, 115200);
		sendBuf[0] = 0x12 ; sendBuf[1] = 0x34; sendBuf[2] =0x56 ;
		//sendBuf[3] = 127;//speed =0
		sendBuf[3] = 0;
		sendBuf[4] = 127; //the forth byte is speed(127-255)
						//the fifth byte is angle(0-255) left is 0-127 and right is 128-255
	}
	
	~lower_control()
	{
		close(fd);
	}
	
private:
	ros::NodeHandle n;
	ros::Subscriber sub;
	ros::Timer send_Cmd_timer;
	
	int fd;
	std::string port_name;
	unsigned char sendBuf[5];
	void Callback(const std_msgs::Float64 &msg)
	{
		ROS_INFO("receive msg....");
		double Kp = 0.6;
		if(msg.data > 999.0) sendBuf[3] = 127;//speed =0
		sendBuf[4] = msg.data + 127;
		if(sendBuf[4] < 1) sendBuf[4] = 1;
		if(sendBuf[4] > 254) sendBuf[4] = 254;
	}
	
	void send_Cmd_Callback(const ros::TimerEvent&)
	{
		write(fd,sendBuf,5);
		ROS_INFO("timer callback...");
	}
};


int main(int argc, char** argv)
{
	ros::init(argc,argv,"lower_control_node");
	
	lower_control l_ctr;
	
	ros::spin();
}

