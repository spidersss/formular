#include "formular/formular.h"
class rongheHandler
{
public:
	rongheHandler()
	{	
		sub_color = n.subscribe("color_points", 10, &rongheHandler::color_cb, this);
		sub_lidar = n.subscribe("cluster_points", 10, &rongheHandler::lidar_cb, this);
		ronghe_pub = n.advertise<sensor_msgs::PointCloud2>("ronghe_points",1, true);
	}
	void color_cb(const sensor_msgs::PointCloud2 &cloud_msg)
	{
		color_msg = cloud_msg;
	}
	void lidar_cb(const sensor_msgs::PointCloud2 &cloud_msg)
	{
		PointCloud cloud_lidar;
		PointCloud cloud_color;
		pcl::fromROSMsg(cloud_msg, cloud_lidar);
		pcl::fromROSMsg(color_msg, cloud_color);
		for(int i = 0; i < cloud_lidar.points.size(); i++)
		{
			for(int j = 0; j < cloud_color.points.size(); j++)
			{
				//if(cloud_color.points[j].z < -9.0) continue;
				if(fabs(cloud_lidar.points[i].x - cloud_color.points[j].x) < 1.0 && fabs(cloud_lidar.points[i].y - cloud_color.points[j].y) < 1.0)	
				{
					cloud_lidar.points[i].z = cloud_color.points[j].z + cloud_color.points[j].z * cloud_lidar.points[i].z;//1.0 + 1.0*0.0
					//cloud_color.points[j].z = -10.0;
					break;		
				}
			
			}
		}
		sensor_msgs::PointCloud2 output;
		pcl::toROSMsg(cloud_lidar, output);
		ronghe_pub.publish(output);
	}
	private:
	ros::NodeHandle n;
	ros::Subscriber sub_color;
	ros::Subscriber sub_lidar;
	ros::Publisher ronghe_pub;
	sensor_msgs::PointCloud2 color_msg;
};

int main(int argc, char** argv)
{
	ros::init (argc, argv, "ronghe_points");
	rongheHandler handler;
	ros::spin();
	return 0;
}
