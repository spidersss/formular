#include "formular/formular.h"
ros::Publisher pub_steer;
float yawErrToSteerAngle_Kp=1.0;

double ronghe_steerCreator(PointCloud cloud)
{
	double x = 0;
	double y = 0;
	double z = 0;
	double center_x = 0;
	double center_y = 0;
	double disToNext = 0;
	int flagl = 0;
	int flagr = 0;
	double lx = 0.0;
	double ly = 0.0;
	double rx = 0.0;
	double ry = 0.0;
	//if(cloud.points.size()<2 || ((cloud.points[0].x*cloud.points[0].x +cloud.points[0].y*cloud.points[0].y) > 100.0)) return 10000.;
	std::vector<pcl::PointXYZ, Eigen::aligned_allocator_indirection<pcl::PointXYZ> >::iterator iter;
	for(iter = cloud.points.begin(); iter != cloud.points.end(); iter++){
		std::cout<<"x:"<<iter->x<<"\t"<<"y:"<<iter->y<<std::endl;
	}
  	for(int i = 0; i < cloud.points.size(); i ++){
  		x = cloud.points[i].x;
  		y = cloud.points[i].y;
  		z = cloud.points[i].z;
  		if(z < -1.4 && flagl ==0 ){
  			lx = x;
  			ly = y;
  		}
  		if(z > -1.4 && flagr ==0 ){
  			rx = x;
  			ry = y;
  		}
	  	center_x = (lx + rx)/2.0;
	  	center_y = (ly + ry)/2.0;
		disToNext = sqrt(center_x*center_x + center_y*center_y);
		if(disToNext > 1.0){
		double theta = (atan2(-1.0, 0) - atan2(center_y, center_x))/M_PI*180.0;	
		std::cout<<"center_x: "<<center_x<<" center_y: "<<center_y<<"theta: "<<theta<<"\tdisToNext:"<<disToNext<<std::endl;
		if(z >= 10.0) theta = 10000.0;
		return theta;
		}		
	}
	return -1;	
}

void cloud_cb(const sensor_msgs::PointCloud2 &cloud_msg)
{
	PointCloud cloud_center;
	pcl::fromROSMsg(cloud_msg, cloud_center);
	
	std_msgs::Float64 steer;
/***********/
	steer.data = yawErrToSteerAngle_Kp * ronghe_steerCreator(cloud_center);
/************/
	pub_steer.publish(steer);

}

int main(int argc, char** argv)
{
	ros::init (argc, argv, "steer");
	ros::NodeHandle n;
	n.param<float>("yawErrToSteerAngle_Kp",yawErrToSteerAngle_Kp,1.0);
	ros::Subscriber sub = n.subscribe("ronghe_points", 10, cloud_cb);
	pub_steer = n.advertise<std_msgs::Float64> ("ronghe_steer", 10);
	ros::spin();
}
