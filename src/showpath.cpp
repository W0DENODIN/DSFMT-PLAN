#include <ros/ros.h>
#include <ros/console.h>
#include <nav_msgs/Path.h>      //point----1
#include <std_msgs/String.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf/transform_broadcaster.h>
#include <tf/tf.h>
#include "nav_msgs/OccupancyGrid.h"
#include <tf/transform_listener.h>
#include "functions.h"

nav_msgs::OccupancyGrid mapData;
void mapCallBack(const nav_msgs::OccupancyGrid::ConstPtr& msg)
{
	mapData = *msg;
}


main (int argc, char **argv)
{
    ros::init (argc, argv, "showpath");

    ros::NodeHandle ph;

    std::string map_topic,base_frame_topic,odom_topic;
    ros::param::param<std::string>("/map_topic", map_topic, "/robot_1/map");
    ros::param::param<std::string>("/robot_frame", base_frame_topic, "/robot_1/base_link");
    ros::param::param<std::string>("/odom_topic", odom_topic, "/robot_1/odom");
    ros::Subscriber sub = ph.subscribe(map_topic, 100 ,mapCallBack);

    ros::Publisher path_pub = ph.advertise<nav_msgs::Path>("trajectory",5, true);   //point----1

    ros::Time current_time, last_time;
    current_time = ros::Time::now();
    last_time = ros::Time::now();

    nav_msgs::Path path;
    path.header.stamp=current_time;
    path.header.frame_id="/robot_1/odom";
    
    double x = 0.0;
    double y = 0.0;
    double th = 0.0;
    double vx = 0.1;
    double vy = -0.1;
    double vth = 0.1;

    int i = 0;
    float disPath = 0.0;
    std::vector<float> pose_b(2), pose_a(2);


    ros::Rate loop_rate(5);

    tf::TransformListener listener, listener_odom;
    tf::StampedTransform transform, transform_odom;


    while (ros::ok())
    {
        int  temp = 0;
        while (temp==0)
		{
			try
			{
				temp = 1;
				listener.lookupTransform(map_topic, base_frame_topic , ros::Time(0), transform);
                listener_odom.lookupTransform(odom_topic, base_frame_topic, ros::Time(0), transform_odom);
			}
			catch (tf::TransformException ex)
			{
				temp = 0;
				ros::Duration(0.1).sleep();
			}
		}

        current_time = ros::Time::now();

        geometry_msgs::PoseStamped this_pose_stamped;
        // this_pose_stamped.pose.position.x = x;
        // this_pose_stamped.pose.position.y = y;
        this_pose_stamped.pose.position.x = transform.getOrigin().x();
        this_pose_stamped.pose.position.y = transform.getOrigin().y();

        
        pose_a[0] =(int) (transform_odom.getOrigin().x() * 10000) / 10000;
        pose_a[1] =(int) (transform_odom.getOrigin().y() * 10000) / 10000;

        if(i>=1)
        {
            disPath += pow((pow((pose_a[0]-pose_b[0]),2) + pow((pose_a[1]-pose_b[1]),2)), 0.5);
        }
        pose_b[0] = pose_a[0];
        pose_b[1] = pose_a[1];
        i++;
        this_pose_stamped.pose.orientation.w = disPath;

        this_pose_stamped.header.stamp=current_time;
        this_pose_stamped.header.frame_id="/robot_1/odom";
        path.poses.push_back(this_pose_stamped);    //point----1

        path_pub.publish(path);     //point----1


        printf("---Trajectory_len: %0.3f \n", disPath);

        ros::spinOnce();               // check for incoming messages

        last_time = current_time;
        loop_rate.sleep();
    }

    return 0;
}