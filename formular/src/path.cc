//#include "formular/path_planning.h"
//#include "formular/formular.h"
#include <ros/ros.h>
#include "std_msgs/Float64.h"
#include <iostream>
#include <vector>
#include <math.h>
#include <sensor_msgs/PointCloud2.h>
#include <std_msgs/Float64MultiArray.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/features/normal_3d.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl_ros/transforms.h>
#include <pcl/filters/statistical_outlier_removal.h>
#ifndef M_PI
#define M_PI 3.1415926
#endif

#include <ros/console.h> 
#include <nav_msgs/Path.h> 
#include <std_msgs/String.h> 
#include <geometry_msgs/Quaternion.h> 
#include <geometry_msgs/PoseStamped.h> 
#include <tf/transform_broadcaster.h> 
#include <tf/tf.h> 
#include <algorithm>

using namespace std;

typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;  
struct Point{
	double x;
	double y;
};

/**************/
ros::Publisher pub_steer;
ros::Publisher path_pub;
void cloud_cb(const sensor_msgs::PointCloud2 &cloud_msg)
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
  	for(int i = 0; i < len - 1; i ++){
  		if(i == len - 2){
  			if(cloud_center.points[i].x < cloud_center.points[i+1].x){
  				x = cloud_center.points[i].x;
				y = cloud_center.points[i].y;
				cout<<'a'<<x<<y<<endl;
				point.x = x;
				point.y = y;
				side_1.push_back(point);
				x = cloud_center.points[i+1].x;
				y = cloud_center.points[i+1].y;
				cout<<'b'<<x<<y<<endl;
				point.x = x;
				point.y = y;
				side_2.push_back(point);
  			}
  			else{
  				x = cloud_center.points[i].x;
				y = cloud_center.points[i].y;
				cout<<'c'<<x<<y<<endl;
				point.x = x;
				point.y = y;
				side_2.push_back(point);
				x = cloud_center.points[i+1].x;
				y = cloud_center.points[i+1].y;
				cout<<'d'<<x<<y<<endl;
				point.x = x;
				point.y = y;
				side_1.push_back(point);
  			}
  		}
  		else {
  			if(cloud_center.points[i].x < cloud_center.points[i+1].x){
  				x = cloud_center.points[i].x;
				y = cloud_center.points[i].y;
				cout<<'e'<<x<<y<<endl;
				point.x = x;
				point.y = y;
				side_1.push_back(point);
  			}
  			else{
  				x = cloud_center.points[i].x;
				y = cloud_center.points[i].y;
				cout<<'f'<<x<<y<<endl;
				point.x = x;
				point.y = y;
				side_2.push_back(point);
  			}
  		}
	} 
	
	
	//**最终路径绘制**rviz 版本//
	nav_msgs::Path path; //nav_msgs::Path path; 
	path.header.stamp=cloud_msg.header.stamp; 
	path.header.frame_id="pandar"; 
	path.poses.clear();
	
	vector<Point> target_point;
	point.x = 0.0;
	point.y = 0.0;
	target_point.push_back(point);
	for (int i = 0; i < min(side_1.size(), side_2.size() ); ++i)
	{
		point.x = (side_1[i].x + side_2[i].x) / 2.0; 
		point.y = (side_1[i].y + side_2[i].y) / 2.0;
		target_point.push_back(point);
	}
	geometry_msgs::PoseStamped this_pose_stamped; 
	geometry_msgs::Quaternion goal_quat;
	for (int i = 0; i < target_point.size(); ++i)//配对两边椎捅//////////////////////////////
	{	
		this_pose_stamped.pose.position.x = target_point[i].x; 
		this_pose_stamped.pose.position.y = target_point[i].y;
		cout<<"目标点:"<<target_point[i].x<<","<<target_point[i].y<<endl;
		if(i < target_point.size() - 1){
			goal_quat = tf::createQuaternionMsgFromYaw(atan2(target_point[i+1].y-target_point[i].y, target_point[i+1].x-target_point[i].x));
			this_pose_stamped.pose.orientation.x = goal_quat.x; 
			this_pose_stamped.pose.orientation.y = goal_quat.y; 
			this_pose_stamped.pose.orientation.z = goal_quat.z; 
			this_pose_stamped.pose.orientation.w = goal_quat.w; 
		}
		else{
			this_pose_stamped.pose.orientation = path.poses[i-1].pose.orientation;	
		}
		this_pose_stamped.header.stamp=ros::Time::now(); 
		this_pose_stamped.header.frame_id="pandar"; 
		path.poses.push_back(this_pose_stamped); 
		
	}
	path_pub.publish(path); // check for incoming messages 
	
	std_msgs::Float64 steer;
	steer.data = cloud_center.points[0].x;
	pub_steer.publish(steer);

}

int main(int argc, char** argv)
{
	ros::init (argc, argv, "steer");
	ros::NodeHandle n;
	
	ros::Subscriber sub = n.subscribe("cluster_points", 10, cloud_cb);
	pub_steer = n.advertise<std_msgs::Float64> ("formular_steer", 10);
	path_pub = n.advertise<nav_msgs::Path>("trajectory",1, true);
	ros::spin();
}
