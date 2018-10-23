#ifndef __PATH_PLANNING_H_
#define __PATH_PLANNING_H_

#include <iostream>
#include <vector>
#include<ctime>  
#include<math.h>  
#include<fstream>
#include<opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp> 
#include <opencv2/core/core.hpp> 
#include <opencv2/imgproc/imgproc.hpp>
#include <algorithm>

#include <ros/console.h> 
#include <nav_msgs/Path.h> 
#include <std_msgs/String.h> 
#include <geometry_msgs/Quaternion.h> 
#include <geometry_msgs/PoseStamped.h> 
#include <tf/transform_broadcaster.h> 
#include <tf/tf.h> 



#define STEP 50
#define CAR_SIZE 20
#define SEARCH_REGION 120  
#define big_M 1000
#define random_int(a,b) (rand()%(b-a+1)+a)  
#define random_double(a,b) (rand()/double(RAND_MAX))  

using namespace cv;
using namespace std;

/************************º¯ÊýÉùÃ÷************************/
void bias_extend_tree(int(&x_picture), int(&y_picture), Mat(&map), const int endnode[2], double(&newpoint)[2], vector<vector<int> >(&tree), vector<double>y1, vector<double>y2, int(&filled_num), int(&greed_flag), int(&x_start), int(&x_end));
void find_path(vector<int>(&path_x), vector<int>(&path_y), vector<vector<int> >(&tree), int(&startnode)[2], int(&x_start), int(&x_end));
int collision(double(&newpoint)[2], Mat(&map), int x_picture, int y_picture, vector<vector<int> >(&tree), vector<double>y1, vector<double>y2, int(&filled_num), int(&x_start), int(&x_end));
int search_nearnode(vector<vector<int> >(&tree), vector<int>(&path_x), vector<int>(&path_y), int(&startnode)[2], int(&x_start), int(&x_end));
int fit_num(vector<Point>data_set);
Mat polyfit(vector<Point>&(point_set), int n);
void string_to_num(string s_temp, int* input_data);
void insert_sort(vector<Point>&a);
#endif// !Â·Ÿ¶¹æ»®Í·ÎÄŒþ

/*************************º¯Êý¶šÒå*************************/
/**žÄœøºóµÄÆ«ÏòÄ¿±êÐÍRRTËã·š**/
/**žÄœøºóµÄÆ«ÏòÄ¿±êÐÍRRTËã·š**/
void bias_extend_tree(int(&x_picture), int(&y_picture), Mat(&map), const int endnode[2], double(&newpoint)[2], vector<vector<int> >(&tree), vector<double>y1, vector<double>y2, int(&filled_num), int(&greed_flag), int(&x_start), int(&x_end))
{
	int find_onenode_flag = 0;
	int randpoint[2] = { 0,0 };
	while (find_onenode_flag != 1)  //每次while循环，都会找到一个可行的节点，从而拓展路线
	{
		double rand_num = random_double(0, 1);
		if ((greed_flag == 1) || (rand_num < 0.5))  //上次往目标点方向的拓展未受到阻碍,或产生的随机数小于偏置概率，直接以终点为目标点进行拓展
		{
			randpoint[0] = endnode[0];
			randpoint[1] = endnode[1];
		}
		else
		{
			randpoint[0] = random_int(tree[filled_num - 1][0], x_end);
			int a = y1[randpoint[0]], b = y2[randpoint[0]];  //把double转化成int
			randpoint[1] = random_int(a, b);
		}
		double x_deviation = 0;
		double y_deviation = 0;
		if ((randpoint[0] == tree[filled_num - 1][0]) || (randpoint[1] == tree[filled_num - 1][1]))
		{
			x_deviation = (randpoint[0] + 2) - tree[filled_num - 1][0];
			y_deviation = (randpoint[1] + 2) - tree[filled_num - 1][1];
		}
		else
		{
			x_deviation = randpoint[0] - tree[filled_num - 1][0];
			y_deviation = randpoint[1] - tree[filled_num - 1][1];
		}
		double x_deviation_abs = fabs(x_deviation);
		double y_deviation_abs = fabs(y_deviation);
		newpoint[0] = tree[filled_num - 1][0] + (x_deviation / x_deviation_abs)*(random_int(1, STEP));  //从搜索到的相邻节点向随机节点方向拓展随机步长距离
		if (newpoint[0] > x_end)
		{
			newpoint[0] = x_end;
		}
		newpoint[1] = tree[filled_num - 1][1] + (y_deviation / y_deviation_abs)*(random_int(1, STEP));
		if (collision(newpoint, map, x_picture, y_picture, tree, y1, y2, filled_num, x_start, x_end) == 0)
		{
			for (int k = 0; k != tree.size(); ++k)
			{
				if ((tree[k][0] == 0) && (tree[k][1] == 0))  //每找到一个可行的节点，将其加入tree中
				{
					tree[k][0] = newpoint[0];
					tree[k][1] = newpoint[1];
					break;
				}
			}
			if ((randpoint[0] == endnode[0]) && (randpoint[1] == endnode[1]))
			{
				greed_flag = 1;
			}
			else
			{
				greed_flag = 0;
			}
			find_onenode_flag = 1;  //找到一个新节点，结束上面的while循环
		}
		else
		{
			find_onenode_flag = 0;  //重复循环直到找到可行节点
		}
	}
}
/**新节点选择函数**/
int search_nearnode(vector<vector<int> >(&tree), vector<int>(&path_x), vector<int>(&path_y), int(&startnode)[2], int(&x_start), int(&x_end))
{
	vector<double>distance;
	vector<int>d_num;
	double distance_min = big_M;
	int lastnode_num = 0;
	for (int i = 0; i != tree.size(); ++i)
	{
		if ((tree[i][0] != 0) && (tree[i][1] != 0))
		{
			lastnode_num = lastnode_num + 1;  //统计tree写到哪一行
		}
	}
	int path_num = 0;
	for (int i = 0; i != path_x.size(); ++i)
	{
		path_num = path_num + 1;  //统计path_x写到哪一行
	}
	int block_r = path_x[path_num - 1];
	int block_l = max(block_r - SEARCH_REGION, x_start);
	for (int i = 1; i != lastnode_num; ++i)
	{
		if ((tree[i][0]>block_l) && (tree[i][0] <= block_r))
		{
			double temp_distance = sqrt((tree[i][0] - startnode[0])*(tree[i][0] - startnode[0]) + (tree[i][1] - startnode[1])*(tree[i][1] - startnode[1]));
			distance.push_back(temp_distance);
			d_num.push_back(i);
		}
	}
	int min_row_num = 0;
	for (int i = 0; i != distance.size(); ++i)
	{
		if (distance[i] < distance_min)
		{
			distance_min = distance[i];
			min_row_num = d_num[i];
		}
	}
	return min_row_num;  //返回距离最近的点在tree中对应的行数
}
/**碰撞检测函数**/
int collision(double(&newpoint)[2], Mat(&map), int x_picture, int y_picture, vector<vector<int> >(&tree), vector<double>y1, vector<double>y2, int(&filled_num), int(&x_start), int(&x_end))
{
	int collision_flag_1 = 0;  //判断新点附近是否存在障碍物
	int x_temp = newpoint[0];
	int y_temp = newpoint[1];
	for (int i = (max(x_temp - CAR_SIZE, x_start)); i <= (min(x_temp + CAR_SIZE, x_end)); ++i)
	{
		if (((y_temp - CAR_SIZE) > y1[i]) && ((y_temp + CAR_SIZE) < y2[i]))
		{
			collision_flag_1 = 0;  //不会碰撞，该点可行
		}
		else
		{
			collision_flag_1 = 1;
			break;
		}
	}
	return collision_flag_1;
}
/**随机搜索完成后，从终点追溯可行路径**/
void find_path(vector<int>(&path_x), vector<int>(&path_y), vector<vector<int> >(&tree), int(&startnode)[2], int(&x_start), int(&x_end))
{
	int min_row_num = 0;
	double distance = big_M;
	while (distance>STEP)
	{
		min_row_num = search_nearnode(tree, path_x, path_y, startnode, x_start, x_end);  //加入追溯路径的节点序号
		path_x.push_back(tree[min_row_num][0]);
		path_y.push_back(tree[min_row_num][1]);
		int path_num = 0;
		for (int i = 0; i != path_x.size(); ++i)
		{
			path_num = path_num + 1;  //统计path_x写到哪一行
		}
		distance = sqrt((startnode[0] - path_x[path_num - 1])*(startnode[0] - path_x[path_num - 1]) + (startnode[1] - path_y[path_num - 1])*(startnode[1] - path_y[path_num - 1]));
	}
	double temp_distance = sqrt((path_x[path_x.size() - 1] - startnode[0])*(path_x[path_x.size() - 1] - startnode[0]) + (path_y[path_y.size() - 1] - startnode[1])* (path_y[path_y.size() - 1] - startnode[1]));
	if (temp_distance <40)
	{
		path_x.pop_back();
		path_y.pop_back();
	}
	cout << "路径回溯结束！" << endl;
}
int fit_num(vector<Point>data_set)
{
	int fit_num = 0;
	if (data_set.size() >= 8)  //根据拟合点数量决定多项式的拟合阶数
	{
		fit_num = 8;
	}
	else
	{
		if (data_set.size() <= 2)
		{
			fit_num = 1;
		}
		else
		{
			fit_num = 2;
		}
	}
	return fit_num;
}
Mat polyfit(vector<Point>&(point_set), int n)
{
	int size = point_set.size();  //拟合点的数目
	int x_num = n + 1;
	Mat mat_u(size, x_num, CV_64F);  //构造矩阵U和Y
	Mat mat_y(size, 1, CV_64F);
	for (int i = 0; i < mat_u.rows; ++i)
	{
		for (int j = 0; j < mat_u.cols; ++j)
		{
			mat_u.at<double>(i, j) = pow(point_set[i].x, j);
		}
	}
	for (int i = 0; i < mat_y.rows; ++i)
	{
		mat_y.at<double>(i, 0) = point_set[i].y;
	}
	Mat mat_k(x_num, 1, CV_64F); //矩阵运算，获得系数矩阵K
	mat_k = (mat_u.t()*mat_u).inv()*mat_u.t()*mat_y;
	cout << mat_k << endl;
	return mat_k;
}
void string_to_num(string s_temp, int* input_data)
{
	bool temp = false;		//读取一个数据标志位
	int data = 0;				//分离的一个数据
	int m = 0;				//数组索引值
	for (int i = 0; i < s_temp.length(); i++)
	{
		while ((s_temp[i] >= '0') && (s_temp[i] <= '9'))		//当前字符是数据，并一直读后面的数据，只要遇到不是数字为止
		{
			temp = true;		//读数据标志位置位
			data *= 10;
			data += (s_temp[i] - '0');		//字符在系统以ASCII码存储，要得到其实际值必须减去‘0’的ASCII值
			i++;
		}
		if (temp)		//判断是否完全读取一个数据
		{
			input_data[m] = data;		//赋值
			m++;
			data = 0;
			temp = false;		//标志位复位
		}
	}
}
/**插入排序**/
void insert_sort(vector<Point>&a) 
{
	for (int i = 1; i<a.size(); ++i) 
	{
		int tmp_x = a[i].x;
		int tmp_y = a[i].y;
		int j = i - 1;
		while (j >= 0 && tmp_x<a[j].x) 
		{
			a[j + 1].x = a[j].x;
			a[j + 1].y = a[j].y;
			j--;
		}
		a[j + 1].x = tmp_x;
		a[j + 1].y = tmp_y;
	}
}


