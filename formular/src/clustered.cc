#include "formular/formular.h"
ros::Publisher pubxyz;
float cluster_search_radius = 0.0;


void cloud_cb(const sensor_msgs::PointCloud2 &cloud_msg)
{
	PointCloud cloud_parted;
	pcl::fromROSMsg(cloud_msg, cloud_parted);
	
	PointCloud cloud_center;
/***********/
	cloud_center = center_cluster(cloud_parted, cluster_search_radius, 10, 2500);
/***********聚类阈值: 搜索半径、最少的点数目、最大点数目***************/	
	sensor_msgs::PointCloud2 output;
	pcl::toROSMsg(cloud_center, output);
	pubxyz.publish(output);
}

int main(int argc, char** argv)
{
	ros::init (argc, argv, "clustered");
	ros::NodeHandle n;
	n.param<float>("cluster_search_radius",cluster_search_radius,0.2);
	//ROS_INFO("%f",cluster_search_radius);
	
	ros::Subscriber sub = n.subscribe("parted_points", 10, cloud_cb);
	pubxyz = n.advertise<sensor_msgs::PointCloud2> ("cluster_points", 10);
	ros::spin();
}
