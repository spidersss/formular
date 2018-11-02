#include "formular/formular.h"
class cloudHandler
{
public:
	cloudHandler()
	{
		sub = n.subscribe("velodyne_points", 10,  &cloudHandler::cloud_cb, this);
		pubxyz = n.advertise<sensor_msgs::PointCloud2> ("parted_points", 10);
		n.param<double>("slope",slope,0.1);
		n.param<double>("widthOfRalatedRegion",widthOfRalatedRegion,10.0);
		n.param<double>("distanceOfDetection",distanceOfDetection,20.0);
		n.param<double>("radiusOfUnrelatedRegion",radiusOfUnrelatedRegion,0.3);
		n.param<double>("thresholdOfheight",thresholdOfheight,-0.0);
		//ROS_INFO("slope=%f",slope);
	}
	
	void cloud_cb(const sensor_msgs::PointCloud2 &cloud_msg)
	{
		PointCloud cloud_init;
		pcl::fromROSMsg(cloud_msg, cloud_init);
		
		PointCloud cloud_parted;
/*********************/
		cloud_parted = space_part(cloud_init, slope,widthOfRalatedRegion,distanceOfDetection,radiusOfUnrelatedRegion,thresholdOfheight);
/*******************the second parameter is slope*/
		sensor_msgs::PointCloud2 output;
		pcl::toROSMsg(cloud_parted, output);
		pubxyz.publish(output);
	}
	
	
private:
	ros::NodeHandle n;
	ros::Subscriber sub;
	ros::Publisher pubxyz;
	std::vector<double> heights;
	double slope;
	double widthOfRalatedRegion;
	double distanceOfDetection;
	double radiusOfUnrelatedRegion;
	double thresholdOfheight;
};

int main(int argc, char** argv)
{
	ros::init (argc, argv, "parted");
	cloudHandler handler;
	ros::spin();
	return 0;
}
