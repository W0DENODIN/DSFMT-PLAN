#include "ros/ros.h"
#include "std_msgs/String.h"
#include <sstream>
#include <iostream>
#include <string>
#include <vector>
#include "stdint.h"
#include "functions.h"
#include "mtrand.h"


#include "nav_msgs/OccupancyGrid.h"
#include "geometry_msgs/PointStamped.h"
#include "std_msgs/Header.h"
#include "nav_msgs/MapMetaData.h"
#include "geometry_msgs/Point.h"
#include "visualization_msgs/Marker.h"		//point----1
#include <tf/transform_listener.h>


// global variables
nav_msgs::OccupancyGrid mapData;
geometry_msgs::PointStamped clickedpoint;
geometry_msgs::PointStamped exploration_goal;
visualization_msgs::Marker points,line;		//point----1
float xdim, ydim, resolution, Xstartx, Xstarty, init_map_x, init_map_y;

rdm r; // for genrating random numbers
int n = 0;

//Subscribers callback functions---------------------------------------
void mapCallBack(const nav_msgs::OccupancyGrid::ConstPtr& msg)
{
	mapData = *msg;
}


// void rvizCallBack(const geometry_msgs::PointStamped::ConstPtr& msg)
// { 
// 	geometry_msgs::Point p;  
// 	p.x=msg->point.x;
// 	p.y=msg->point.y;
// 	p.z=msg->point.z;

// 	points.points.push_back(p);
// }



int main(int argc, char **argv)
{

  unsigned long init[4] = {0x123, 0x234, 0x345, 0x456}, length = 7;
  MTRand_int32 irand(init, length); // 32-bit int generator
  // this is an example of initializing by an array
  // you may use MTRand(seed) with any 32bit integer
  // as a seed for a simpler initialization
  MTRand drand; // double in [0, 1) generator, already init

  // generate the same numbers as in the original C test program
  ros::init(argc, argv, "local_rrt_frontier_detector");
  ros::NodeHandle nh;
  
  // fetching all parameters
  float eta,init_map_x,init_map_y,range;
//   float etai;
  std::string map_topic,base_frame_topic;
  
  std::string ns;
  ns=ros::this_node::getName();

  ros::param::param<float>(ns+"/eta", eta, 0.5);
//   ros::param::param<float>(ns+"/etai", etai, 2.0);
  ros::param::param<std::string>(ns+"/map_topic", map_topic, "/robot_1/map"); 
  ros::param::param<std::string>(ns+"/robot_frame", base_frame_topic, "/robot_1/base_link"); 

//---------------------------------------------------------------
  ros::Subscriber sub = nh.subscribe(map_topic, 100 ,mapCallBack);	
//   ros::Subscriber rviz_sub = nh.subscribe("/clicked_point", 100 ,rvizCallBack);	

  ros::Publisher targetspub = nh.advertise<geometry_msgs::PointStamped>("/detected_points", 10);
  ros::Publisher pub = nh.advertise<visualization_msgs::Marker>(ns+"_shapes", 10);				//point----1

  ros::Rate rate(100);


// wait until map is received, when a map is received, mapData.header.seq will not be < 1  
while (mapData.header.seq<1 or mapData.data.size()<1)  
{  
	ros::spinOnce();  
	ros::Duration(0.1).sleep();
}


//visualizations  points and lines..			//point--------1
points.header.frame_id=mapData.header.frame_id;
line.header.frame_id=mapData.header.frame_id;
points.header.stamp=ros::Time(0);
line.header.stamp=ros::Time(0);

points.ns=line.ns = "markers";
points.id = 0;
line.id =1;


points.type = points.POINTS;
line.type = line.LINE_LIST;

//Set the marker action.  Options are ADD, DELETE, and new in ROS Indigo: 3 (DELETEALL)
points.action =points.ADD;
line.action = line.ADD;
points.pose.orientation.w =1.0;
line.pose.orientation.w = 1.0;
line.scale.x =  0.03;
line.scale.y= 0.03;
points.scale.x=0.3; 
points.scale.y=0.3; 

line.color.r = 255.0/255.0;
line.color.g = 0.0/255.0;
line.color.b = 0.0/255.0;
points.color.r = 0.0/255.0;
points.color.g = 0.0/255.0;
points.color.b = 255.0/255.0;
points.color.a = 0.3;
line.color.a = 1.0;
points.lifetime = ros::Duration();
line.lifetime = ros::Duration();

geometry_msgs::Point p;  



init_map_x = 50;
init_map_y = 50;

Xstartx = 0.5;
Xstarty = 0;

// geometry_msgs::Point trans;
// trans = points.points[4];
std::vector< std::vector<float> > V;
std::vector<float> xnew; 
// xnew.push_back( trans.x);xnew.push_back( trans.y);  
xnew.push_back( Xstartx);	xnew.push_back( Xstarty);  
V.push_back(xnew);

// points.points.clear();
// pub.publish(points);


std::vector<float> frontiers;
int i=0;
float xr, yr;
std::vector<float> x_rand, x_nearest, x_new(2), x_new1(2), x_new2(2), x_new3(2), x_new4(2), x_new5(2), x_new6(2), x_new7(2), x_new8(2);
std::vector<float> x_check1(2), x_check2(2), x_check3(2), x_check4(2);
tf::TransformListener listener;

int aa = 120, bb = 0;		//change

// Main loop
while (ros::ok())
{
	tf::StampedTransform transform;
	int temp = 0;
	int far = 10;			//change
	while (temp==0)
	{
		try
		{
			temp = 1;
			listener.lookupTransform(map_topic, base_frame_topic , ros::Time(0), transform);
		}
		catch (tf::TransformException ex)
		{
			temp = 0;
			ros::Duration(0.1).sleep();
		}
	}
	x_new[0]=transform.getOrigin().x(); 	x_new[1]=transform.getOrigin().y();
	V.push_back(x_new);

	// std::cout<< "1******:" <<std::endl;
	// Get the minimum distance to the wall ---------------------------------------------------------------------
	x_check1[0]=transform.getOrigin().x() + 1.01*far; 	x_check1[1]=transform.getOrigin().y();
	x_check2[0]=transform.getOrigin().x() - 1.01*far; 	x_check2[1]=transform.getOrigin().y();
	x_check3[0]=transform.getOrigin().x() + 0.1; 		x_check3[1]=transform.getOrigin().y() + 1.01*far;
	x_check4[0]=transform.getOrigin().x() + 0.1; 		x_check4[1]=transform.getOrigin().y() - 1.01*far;

	// std::cout<< "2******:" <<std::endl;
	float dis1 = ObstacleOccupiedFind(x_new,x_check1,mapData);
	float dis2 = ObstacleOccupiedFind(x_new,x_check2,mapData);
	float dis3 = ObstacleOccupiedFind(x_new,x_check3,mapData);
	float dis4 = ObstacleOccupiedFind(x_new,x_check4,mapData);
	// std::cout<< "***dis1~4:  " << dis1 << " , " << dis2 << " , " <<dis3 << " , " <<dis4 <<std::endl;
/*
	// float DIS[4] = {dis1, dis2, dis3, dis4};
	// float minDis = DIS[0];
	// for (int count = 1; count < 4; count++)
	// {
	// 	if (DIS[count] < minDis)
	// 		minDis = DIS[count];
	// }
	// if (minDis > etai)	{ minDis = etai;}
	// if (minDis < 0.25*etai)	{ minDis = 0.25*etai;}
	// minDis += 0.8;	

	// Greate tree start point-------------------------------------------------------
	// x_new1[0]=x_new[0] + minDis; 	x_new1[1]=x_new[1] + minDis;		//change
	// x_new2[0]=x_new[0] + minDis; 	x_new2[1]=x_new[1] - minDis;
	// x_new3[0]=x_new[0] - minDis; 	x_new3[1]=x_new[1] - minDis;
	// x_new4[0]=x_new[0] - minDis; 	x_new4[1]=x_new[1] + minDis;

	// x_new5[0]=x_new[0] - 0.1; 		x_new5[1]=x_new[1] + minDis;
	// x_new6[0]=x_new[0] + minDis; 	x_new6[1]=x_new[1] + 0.1;
	// x_new7[0]=x_new[0] + 0.3; 		x_new7[1]=x_new[1] - minDis;
	// x_new8[0]=x_new[0] - minDis; 	x_new8[1]=x_new[1] - 0.1;	
*/
	// std::cout<< "3******:" <<std::endl;
	x_new1[0]=x_new[0] + dis1+1; 	x_new1[1]=x_new[1] + dis3+1;
	x_new2[0]=x_new[0] + dis1+1; 	x_new2[1]=x_new[1] - dis4-1;
	x_new3[0]=x_new[0] - dis2-1; 	x_new3[1]=x_new[1] - dis4-1;
	x_new4[0]=x_new[0] - dis2-1; 	x_new4[1]=x_new[1] + dis3+1;

	x_new5[0]=x_new[0] + 0.3; 		x_new5[1]=x_new[1] + dis3+1;
	x_new6[0]=x_new[0] + dis1+1; 	x_new6[1]=x_new[1];
	x_new7[0]=x_new[0] + 0.3; 		x_new7[1]=x_new[1] - dis4-1;
	x_new8[0]=x_new[0] - dis2-1; 	x_new8[1]=x_new[1];

// ------------------------------------------------------------------
	char checkingXnew5 = ObstacleFreeXnew(x_new5, mapData);
	char checkingXnew6 = ObstacleFreeXnew(x_new6, mapData);
	char checkingXnew7 = ObstacleFreeXnew(x_new7, mapData);
	char checkingXnew8 = ObstacleFreeXnew(x_new8, mapData);

	if(checkingXnew5==1 ){V.push_back(x_new5);}
	else {
		char checkingXnew1 = ObstacleFreeXnew(x_new1, mapData);
		if(checkingXnew1==1 ){V.push_back(x_new1);}
	}

	if(checkingXnew6==1 ){V.push_back(x_new6);}
	else {
		char checkingXnew2 = ObstacleFreeXnew(x_new2, mapData);
		if(checkingXnew2==1 ){V.push_back(x_new2);}
	}

	if(checkingXnew7==1 ){V.push_back(x_new7);}
	else {
		char checkingXnew3 = ObstacleFreeXnew(x_new3, mapData);
		if(checkingXnew3==1 ){V.push_back(x_new3);}
	}

	if(checkingXnew8==1 ){V.push_back(x_new8);}
	else {
		char checkingXnew4 = ObstacleFreeXnew(x_new4, mapData);
		if(checkingXnew4==1 ){V.push_back(x_new4);}
	}
/*// ------------------------------------------------------------------
	// char checkingXnew1 = ObstacleFreeXnew(x_new1, mapData);
	// char checkingXnew2 = ObstacleFreeXnew(x_new2, mapData);
	// char checkingXnew3 = ObstacleFreeXnew(x_new3, mapData);
	// char checkingXnew4 = ObstacleFreeXnew(x_new4, mapData);

	// if(checkingXnew1==1 ){V.push_back(x_new1);}
	// else {
	// 	char checkingXnew5 = ObstacleFreeXnew(x_new5, mapData);
	// 	if(checkingXnew5==1 ){V.push_back(x_new5);}
	// }

	// if(checkingXnew2==1 ){V.push_back(x_new2);}
	// else {
	// 	char checkingXnew6 = ObstacleFreeXnew(x_new6, mapData);
	// 	if(checkingXnew6==1 ){V.push_back(x_new6);}
	// }

	// if(checkingXnew3==1 ){V.push_back(x_new3);}
	// else {
	// 	char checkingXnew7 = ObstacleFreeXnew(x_new7, mapData);
	// 	if(checkingXnew7==1 ){V.push_back(x_new7);}
	// }

	// if(checkingXnew4==1 ){V.push_back(x_new4);}
	// else {
	// 	char checkingXnew8 = ObstacleFreeXnew(x_new8, mapData);
	// 	if(checkingXnew8==1 ){V.push_back(x_new8);}
	// }
// ------------------------------------------------------------------
		// char checkingXnew1 = ObstacleFreeXnew(x_new1, mapData);
		// char checkingXnew2 = ObstacleFreeXnew(x_new2, mapData);
		// char checkingXnew3 = ObstacleFreeXnew(x_new3, mapData);
		// char checkingXnew4 = ObstacleFreeXnew(x_new4, mapData);
		// char checkingXnew5 = ObstacleFreeXnew(x_new5, mapData);
		// char checkingXnew6 = ObstacleFreeXnew(x_new6, mapData);
		// char checkingXnew7 = ObstacleFreeXnew(x_new7, mapData);
		// char checkingXnew8 = ObstacleFreeXnew(x_new8, mapData);

		// if(checkingXnew1==1 ){V.push_back(x_new1);}
		// if(checkingXnew2==1 ){V.push_back(x_new2);}
		// if(checkingXnew3==1 ){V.push_back(x_new3);}
		// if(checkingXnew4==1 ){V.push_back(x_new4);}	
		// if(checkingXnew5==1 ){V.push_back(x_new5);}
		// if(checkingXnew6==1 ){V.push_back(x_new6);}
		// if(checkingXnew7==1 ){V.push_back(x_new7);}
		// if(checkingXnew8==1 ){V.push_back(x_new8);}	
// ------------------------------------------------------------------*/

	// Sample free
	x_rand.clear();
	xr = (drand() * init_map_x) - (init_map_x * 0.5) + Xstartx;
	yr = (drand() * init_map_y) - (init_map_y * 0.5) + Xstarty;
	x_rand.push_back( xr ); 
	x_rand.push_back( yr );

	// Nearest
	x_nearest = Nearest(V,x_rand);

	// Steer
	x_new  = Steer(x_nearest, x_rand, eta);

	// ObstacleFree    1:free     -1:unkown (frontier region)      0:obstacle
	char checking  = ObstacleFree(x_nearest,x_new, mapData);

	// std::cout<< "6******:" <<std::endl;

	//---------------------------------------------------------------------	
	if (checking==-1)
	{
		exploration_goal.header.stamp = ros::Time(0);
		exploration_goal.header.frame_id = mapData.header.frame_id;
		exploration_goal.point.x = x_new[0];
		exploration_goal.point.y = x_new[1];
		exploration_goal.point.z = 0.0;
		p.x = x_new[0]; 				//point----1
		p.y = x_new[1]; 
		p.z = 0.0;
		
		points.points.push_back(p);		//point----1
		pub.publish(points) ;
		targetspub.publish(exploration_goal);	//target point
		points.points.clear();
		V.clear();
		
		n++;
							
		line.points.clear();
		bb = 0;
	}
	//---------------------------------------------------------------------
	else if (checking==1)
	{	//change  || checking==0
		V.push_back(x_new);
		
		p.x=x_new[0]; 
		p.y=x_new[1]; 
		p.z=0.0;
		line.points.push_back(p);

		p.x=x_nearest[0]; 
		p.y=x_nearest[1]; 
		p.z=0.0;		
		line.points.push_back(p);			
	}

	// std::cout<< "7******:" <<std::endl;
	
	pub.publish(line);
	
	bb++;	
	//printf("start_TIME: %d \n", bb);
	if(aa <= bb)
	{
		line.points.clear();
		bb = 0;
	}

	if(n%10 == 0)
	{
		std::cout << "*LLocal_point : " << n << std::endl;
	}
// }
	ros::spinOnce();
	rate.sleep();
  }
  return 0;
}
