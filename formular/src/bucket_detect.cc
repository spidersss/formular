#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>

#include "opencv2/core.hpp"
#include "opencv2/imgproc.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/videoio.hpp"
#include <iostream>
#include <string>
#include <vector>
#include <sys/time.h>
#include <time.h>


using namespace cv;
using namespace std;


float a=13;//摄像头倾角
float alpha=a*3.1415926/180;//倾角弧度
float belta = (90-a)*3.1415926/180;
float height=620;//安装高度mm
float Zc=height/sin(alpha) ;//视距
int u = -67;
int v = 442;
int fx = 4649;
int fy = 4617;


double get_wall_time() 
{ 
  struct timeval time ; 
  if (gettimeofday(&time,NULL)){ 
    return 0; 
  } 
  return (double)time.tv_sec + (double)time.tv_usec * .000001; 
} 


bool SetSortRule(const Point pt1, const Point pt2)
{
    if ( pt1.y > pt2.y)
    {
        return true;
    }
    else
    {
        return false;
    }
}


vector<Point> CoordinateCalculate(vector<Point> &pole,vector<Point> & )
{
	vector<Point> worldpoint;
	float X=0.0;
	float Y=0.0;
	//计算X分量 Y 分量
	for(int i=0;i<pole.size();i++)
	{
		
		//cout<<pole[i].x<<"  "<<pole[i].y<<endl;
				
		Y = height / tan(alpha - atan(double((v - pole[i].y)/fy)));
		cout.setf(ios::showpoint);
		//cout<<float((v - pole[i].y)/fy)<<endl;
		X = height*(pole[i].x-320)/(sin(alpha - atan((pole[i].y-240)/510))*507*sqrt(((pole[i].y-240)/510)*((pole[i].y-240)/510)+1));
		
		
		//Y =  height/tan(alpha - atan((pole[i].y -240.0)/510.0)) ;
		//cout<<X<<"  "<<Y<<endl;
		worldpoint.push_back(Point(X,Y));
	}
	return worldpoint;

}

// 线性计算
vector<Point> CoordinateCalculate_2(vector<Point> &pole,vector<Point> & )
{
	vector<Point> worldpoint;
	float X=0.0;
	float Y=0.0;
	//计算x分量 y分量
	for(int i=0;i<pole.size();i++)
	{
		
		int delta_y = 480 - pole[i].y;
		Y = (40.0/8515)*delta_y*delta_y + delta_y - (5306.0/1703) + 143;                           // 143cm is base_y to lidar
		X = ((92.0/291) + (92.0/291)*(43.0/11656)*delta_y) * (pole[i].x - 320);
		//Y = 248 - (pole[i].y - 90)*(98.0/90);
		//X = (pole[i].x - 160)*(17.0/50)*((180.0+(90-pole[i].y))/180.0);
		cout<<"X:"<<X / 100.0 * -1.0<<"  "<<"Y:"<<Y / 100.0 * -1.0<<endl;
		worldpoint.push_back(Point(X,Y));

	}
	return worldpoint;

}


vector<Point> detect_bucket(Mat red)
{

		//开操作 
	   	//Mat element = getStructuringElement(MORPH_RECT, Size(5, 5));  
		//morphologyEx(red, red, MORPH_OPEN, element); 
		//morphologyEx(red, red, MORPH_OPEN, element); 
		//imshow("Bucket2",red);  
	  
	   	//闭操作  
	   	//morphologyEx(red, red, MORPH_CLOSE, element); 
		//imshow("Red Bucket3",red); 



		//contours是目标轮廓，contoursp是筛选后的轮廓	
	   	vector<vector<Point> > contours_red;
	   	vector<vector<Point> > contoursP_red;
	   	//vector<vector<Point>> approx;
		//轮廓面积
	   	vector<double> area_red;
	   	vector<double> areaP_red;
		vector<Point> POLE_red;
	   	vector<Vec4i> hierarchy_red;
	   	findContours(red,contours_red,hierarchy_red,CV_RETR_EXTERNAL,CV_CHAIN_APPROX_SIMPLE,Point());
		Mat imgContours_red=Mat::zeros(red.size(),CV_8UC1);  
		Mat Contours_red=Mat::zeros(red.size(),CV_8UC1);  
		//绘制并筛选轮廓  
		for(int i=0;i<contours_red.size();i++)  
		{  
			//contours[i]代表的是第i个轮廓，contours[i].size()代表的是第i个轮廓上所有的像素点数  
			for(int j=0;j<contours_red[i].size();j++)   
			{  
			    	//绘制出contours向量内所有的像素点  
			    	Point P=Point(contours_red[i][j].x,contours_red[i][j].y);  
			    	Contours_red.at<uchar>(P)=255;  
			}  
		  
			area_red.push_back(contourArea(contours_red[i],false));         
	   		//cout << "area_red "<<i<<"=" << area_red[i] << endl;
			if(area_red[i]>80)
			{
			    	contoursP_red.push_back(contours_red[i]);
			    	areaP_red.push_back(area_red[i]);
			    	//cout << "areaP_red "<<i<<"=" << areaP_red[i] << endl;
			} 	       
			//绘制轮廓 筛选 未筛选 
			drawContours(imgContours_red,contoursP_red,-1,Scalar(255),1,8);  
	   	}	
		//找出极点像素位置
	   	for(int i=0;i<contoursP_red.size();i++)
	   	{   
			sort(contoursP_red[i].begin(),contoursP_red[i].end(),SetSortRule);
	   
		   	Point PolePoint = Point(contoursP_red[i][0].x,contoursP_red[i][0].y);
			cout<<"Pole"<<i<<"="<<PolePoint<<endl;
			POLE_red.push_back(PolePoint);
		  		
	   	}
		// 画出像素点
		for(int i=0;i<POLE_red.size();i++){
			circle(imgContours_red,POLE_red[i],4,Scalar(255,255,255),-1);
		}



		vector<Point> WorldPoint_red;

		//计算世界坐标
		WorldPoint_red = CoordinateCalculate_2(POLE_red,WorldPoint_red);

		
		imshow("counters_red",imgContours_red);	

		return WorldPoint_red;
}


vector<Point> detect_bucket_blue(Mat red)
{

		//开操作 
	   	//Mat element = getStructuringElement(MORPH_RECT, Size(5, 5));  
		//morphologyEx(red, red, MORPH_OPEN, element); 
		//morphologyEx(red, red, MORPH_OPEN, element); 
		//imshow("Bucket2",red);  
	  
	   	//闭操作  
	   	//morphologyEx(red, red, MORPH_CLOSE, element); 
		//imshow("Red Bucket3",red); 



		//contours是目标轮廓，contoursp是筛选后的轮廓	
	   	vector<vector<Point> > contours_red;
	   	vector<vector<Point> > contoursP_red;
	   	//vector<vector<Point>> approx;
		//轮廓面积
	   	vector<double> area_red;
	   	vector<double> areaP_red;
		vector<Point> POLE_red;
	   	vector<Vec4i> hierarchy_red;
	   	findContours(red,contours_red,hierarchy_red,CV_RETR_EXTERNAL,CV_CHAIN_APPROX_SIMPLE,Point());
		Mat imgContours_red=Mat::zeros(red.size(),CV_8UC1);  
		Mat Contours_red=Mat::zeros(red.size(),CV_8UC1);  
		//绘制并筛选轮廓  
		for(int i=0;i<contours_red.size();i++)  
		{  
			//contours[i]代表的是第i个轮廓，contours[i].size()代表的是第i个轮廓上所有的像素点数  
			for(int j=0;j<contours_red[i].size();j++)   
			{  
			    	//绘制出contours向量内所有的像素点  
			    	Point P=Point(contours_red[i][j].x,contours_red[i][j].y);  
			    	Contours_red.at<uchar>(P)=255;  
			}  
		  
			area_red.push_back(contourArea(contours_red[i],false));         
	   		//cout << "area_red "<<i<<"=" << area_red[i] << endl;
			if(area_red[i]>80)
			{
			    	contoursP_red.push_back(contours_red[i]);
			    	areaP_red.push_back(area_red[i]);
			    	//cout << "areaP_red "<<i<<"=" << areaP_red[i] << endl;
			} 	       
			//绘制轮廓 筛选 未筛选 
			drawContours(imgContours_red,contoursP_red,-1,Scalar(255),1,8);  
	   	}	
		//找出极点像素位置
	   	for(int i=0;i<contoursP_red.size();i++)
	   	{   
			sort(contoursP_red[i].begin(),contoursP_red[i].end(),SetSortRule);
	   
		   	Point PolePoint = Point(contoursP_red[i][0].x,contoursP_red[i][0].y);
			cout<<"Pole"<<i<<"="<<PolePoint<<endl;
			POLE_red.push_back(PolePoint);
		  		
	   	}
		// 画出像素点
		for(int i=0;i<POLE_red.size();i++){
			circle(imgContours_red,POLE_red[i],4,Scalar(255,255,255),-1);
		}



		vector<Point> WorldPoint_red;

		//计算世界坐标
		WorldPoint_red = CoordinateCalculate_2(POLE_red,WorldPoint_red);

		
		//imshow("counters_red",imgContours_red);	

		return WorldPoint_red;
}

typedef pcl::PointCloud<pcl::PointXYZ> PointCloud; 

int main(int argc, char** argv)
{
	PointCloud cloud_color;
	sensor_msgs::PointCloud2 output;
	ros::init (argc, argv, "color");
	ros::NodeHandle n;
	ros::Publisher pub = n.advertise<sensor_msgs::PointCloud2> ("color_points", 10);
	ros::Rate loop_rate(20);
	
	Mat img;
	VideoCapture capture(0);
	
	while(ros::ok()){
	
		capture >> img;
		if(!img.empty()){
			//imshow("video",img);
			
			//Mat img = imread("/home/kevin/Desktop/SEU_dRACING/test_pics/test8.jpg", IMREAD_COLOR);
			resize(img, img, Size(img.cols , img.rows ), 0, 0, INTER_LINEAR);
			cout<<"cols:"<<img.cols<<endl;
			cout<<"rows:"<<img.rows<<endl;
			//imshow("src",img);
			
			
			double start_time = get_wall_time();


			///set threshold
			//red
			int red_low_h = 0;
			int red_high_h = 28;
			int red_low_s = 217;
			int red_high_s = 255;
			int red_low_v = 230;
			int red_high_v = 255;
			//blue
			int blue_low_h = 169;
			int blue_high_h = 240;
			int blue_low_s = 56;
			int blue_high_s = 255;
			int blue_low_v = 0;
			int blue_high_v = 255;
			//yellow
			int yellow_low_h = 5;
			int yellow_high_h = 0;
			int yellow_low_s = 5;
			int yellow_high_s = 0;
			int yellow_low_v = 5;
			int yellow_high_v = 0;
			//white
			int white_low_h = 5;
			int white_high_h = 0;
			int white_low_s = 5;
			int white_high_s = 0;
			int white_low_v = 5;
			int white_high_v = 0;


			Mat bgr;
			Mat imgHSV;
			//彩色图像的灰度值归一化
			img.convertTo(bgr, CV_32FC3, 1.0 / 255, 0);
		//	vector<Mat> hsvSplit;
			cvtColor(bgr,imgHSV,COLOR_BGR2HSV);
		//	split(imgHSV,hsvSplit);
		//	equalizeHist(hsvSplit[2],hsvSplit[2]);
		//	merge(hsvSplit,imgHSV);


			Mat red_mask,blue_mask,yellow_mask,white_mask;
			Mat red,blue,yellow,white;
			red = Mat::zeros(img.size(), CV_32FC3);
			blue = Mat::zeros(img.size(), CV_32FC3);
			yellow = Mat::zeros(img.size(), CV_32FC3);
			white = Mat::zeros(img.size(), CV_32FC3);
			
			inRange(imgHSV, Scalar(red_low_h, red_low_s/float(255), red_low_v/float(255)), Scalar(red_high_h, red_high_s/float(255), red_high_v/float(255)), red_mask);
			inRange(imgHSV, Scalar(blue_low_h, blue_low_s/float(255), blue_low_v/float(255)), Scalar(blue_high_h, blue_high_s/float(255), blue_high_v/float(255)), blue_mask);
			inRange(imgHSV, Scalar(yellow_low_h, yellow_low_s/float(255), yellow_low_v/float(255)), Scalar(yellow_high_h, yellow_high_s/float(255), yellow_high_v/float(255)), yellow_mask);
			inRange(imgHSV, Scalar(white_low_h, white_low_s/float(255), white_low_v/float(255)), Scalar(white_high_h, white_high_s/float(255), white_high_v/float(255)), white_mask);
	

	
			addWeighted(red_mask,0.5,white_mask,0.5,0.0,red);
			addWeighted(blue_mask,0.5,white_mask,0.5,0.0,blue);
			addWeighted(yellow_mask,0.5,white_mask,0.5,0.0,yellow);
			
		  
			imshow("Red mask",red_mask);
			imshow("Red ",red);
			imshow("Blue ",blue);
			imshow("Yellow ",yellow);


			vector<Point> WorldPoint_red = detect_bucket(red);
			vector<Point> WorldPoint_blue = detect_bucket(blue);
			vector<Point> WorldPoint_yellow = detect_bucket(yellow);
/********************************/
			pcl::PointXYZ point_color;
			cloud_color.points.clear();
			for(int i = 0; i < WorldPoint_red.size(); i++){
				point_color.x = WorldPoint_red[i].x / 100.0 * -1.0;
				point_color.y = WorldPoint_red[i].y / 100.0 * -1.0;
				point_color.z = 1.0;
				cloud_color.points.push_back(point_color);
			}
			for(int i = 0; i < WorldPoint_blue.size(); i++){
				point_color.x = WorldPoint_blue[i].x / 100.0 * -1.0;
				point_color.y = WorldPoint_blue[i].y / 100.0 * -1.0;
				point_color.z = -1.0;
				cloud_color.points.push_back(point_color);
			}
			for(int i = 0; i < WorldPoint_yellow.size(); i++){
				point_color.x = WorldPoint_yellow[i].x / 100.0 * -1.0;
				point_color.y = WorldPoint_yellow[i].y / 100.0 * -1.0;
				point_color.z = 10.0;
				cloud_color.points.push_back(point_color);
			}
			cloud_color.header.frame_id = "pandar";
  			cloud_color.width = cloud_color.points.size ();
  			cloud_color.height = 1;
  			cloud_color.is_dense = false;


/********************************/

			//显示锥桶和车辆位置信息
			
			Mat map(500,500,CV_8UC3,Scalar(255,255,255,0.5));
			Point lidar = Point(250,500);
			circle(map,lidar,10,Scalar(0,255,0),-1);
			for(int i=0;i<WorldPoint_red.size();i++){
				circle(map,Point(250+WorldPoint_red[i].x, 500-WorldPoint_red[i].y),3,Scalar(0,0,255),-1);
				
				
				putText(map,"red",Point(255+WorldPoint_red[i].x, 495-WorldPoint_red[i].y),FONT_HERSHEY_COMPLEX,0.3,Scalar(0, 0, 255),1,8,0);
			}
			for(int i=0;i<WorldPoint_blue.size();i++){
				circle(map,Point(250+WorldPoint_blue[i].x, 500-WorldPoint_blue[i].y),3,Scalar(255,0,0),-1);
				putText(map,"blue",Point(255+WorldPoint_blue[i].x, 495-WorldPoint_blue[i].y),FONT_HERSHEY_COMPLEX,0.3,Scalar(255, 0, 0),1,8,0);
			}
			//for(int i=0;i<WorldPoint_yellow.size();i++){
			//	circle(map,Point(250+WorldPoint_yellow[i].x, 500-WorldPoint_yellow[i].y),3,Scalar(0,255,255),-1);
			//}
			imshow("map",map);
			

			//显示
			//imshow("counters_red",imgContours_red);	
			//imshow("counters_blue",imgContours_blue);
			//imshow("counters_yellow",imgContours_yellow);
	
			//waitKey(0);


			double end_time = get_wall_time();
			cout<<"times:"<<end_time-start_time<<"ms"<<endl; 	
		
								
		}
		else{
			cout<<"error"<<endl;
		}
		
				
		if(waitKey(30)==27)
		break;
		pcl::toROSMsg(cloud_color, output);
		pub.publish(output);
		ros::spinOnce();
		loop_rate.sleep();
	}
	return 0;
}



