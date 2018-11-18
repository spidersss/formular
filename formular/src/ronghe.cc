#include "formular/formular.h"
class pathHandler
{
public:
	pathHandler()
	{	
		sub_point = n.subscribe("cluster_points", 10, &pathHandler::path_cb, this);
		sub_steer = n.subscribe("lidar_steer", 10, &pathHandler::steer_cb, this);
		path_pub = n.advertise<nav_msgs::Path>("trajectory",1, true);
		steer_sub = 0.0;
	}
	void steer_cb(const std_msgs::Float64 &steer_msg)
	{
		steer_sub = steer_msg.data;
	
	}
	void path_cb(const sensor_msgs::PointCloud2 &cloud_msg)
	{
		PointCloud cloud_center;
		pcl::fromROSMsg(cloud_msg, cloud_center);
	
		Point point;
	
		vector<Point>side_1, side_2;
		double x = 0;
		double y = 0;
		double z = 0;
		int len = cloud_center.points.size();
		cout<<len<<endl;
		if(len < 2) return ;
		path_pub.publish(path); // check for incoming messages 
	
		//std_msgs::Float64 steer;
		//steer.data = cloud_center.points[0].x;
		//pub_steer.publish(steer);

	}
	private:
	ros::NodeHandle n;
	ros::Subscriber sub_point;
	ros::Subscriber sub_steer;
	ros::Publisher path_pub;
	double steer_sub;
};

int main(int argc, char** argv)
{
	ros::init (argc, argv, "ronghe");
	pathHandler handler;
	ros::spin();
	return 0;
}
