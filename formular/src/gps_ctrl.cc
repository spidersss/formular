#include <ros/ros.h>
#include <tf/tf.h>
#include <cmath>
#include "nav_msgs/Odometry.h"
#include "sensor_msgs/NavSatFix.h"
#include "std_msgs/Float64.h"

float yawErrToSteerAngle_Kp=1.0;
class gpsHandler
{
public:
	gpsHandler()
	{
		odom_sub = n.subscribe("gps_odom", 10, &gpsHandler::odom_cb, this);
		pos_sub = n.subscribe("gps_fix_psr", 10, &gpsHandler::pos_cb, this);
		steer_pub = n.advertise<std_msgs::Float64> ("gps_steer", 10);
		gps_yaw = 0.0;
		t_yaw = 0.0;
		yaw_error = 0.0;
		endlat = 31.8881588221;
		endlon = 118.810255524;
		disToend = 0.0;
	}
	void odom_cb(const nav_msgs::Odometry::ConstPtr& msg)
	{
		tf::Quaternion q(
    		msg->pose.pose.orientation.x,
    		msg->pose.pose.orientation.y,
    		msg->pose.pose.orientation.z,
    		msg->pose.pose.orientation.w);
		tf::Matrix3x3 m(q);
		double roll, pitch, yaw;
		m.getRPY(roll, pitch, yaw);
		gps_yaw = yaw/M_PI*180.0;
		std::cout<<gps_yaw<<std::endl;
		//pub_steer.publish(steer);
	}
	void pos_cb(const sensor_msgs::NavSatFix::ConstPtr& gps_msg)
	{
		double gpslon = gps_msg->longitude;
		double gpslat = gps_msg->latitude;
		ROS_INFO("endlat:%f\tendlon:%f\t", endlat, endlon);
		t_yaw = atan2((cos(endlat)*(endlon - gpslon)),(endlat - gpslat))*180/M_PI;
		if(t_yaw < 0) t_yaw = t_yaw + 360;
		yaw_error = t_yaw - gps_yaw;
		  
		steer.data = yaw_error*(-5.0) + 90;
		ROS_INFO("yaw_error:%f\t", yaw_error);
		if(steer.data < 60) steer.data = 60;
		if(steer.data > 120) steer.data = 120;
		double func_k = (endlat -gpslat) / (endlon- gpslon);//直线的斜率：y:lat x:lon
		double func_b = endlat - func_k * endlon;
		disToend = (func_k * gpslon - gpslat + func_b) /(sqrt((func_k * func_k)+1))*100000;
		ROS_INFO("t_yaw: %f\tnowYaw: %f\tdisToend: %f", t_yaw,gps_yaw, disToend);	
		steer_pub.publish(steer);
	}
private:
	ros::NodeHandle n;
	ros::Subscriber odom_sub;
	ros::Subscriber pos_sub;
	ros::Publisher steer_pub;
	std_msgs::Float64 steer;
	double gps_yaw;
	double t_yaw;
	double yaw_error;
	double endlat;
	double endlon;
	double disToend;
};
int main(int argc, char** argv)
{
	ros::init (argc, argv, "gps_steer");
	gpsHandler handler;
	ros::spin();
	return 0;
}
