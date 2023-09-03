#include "ros/ros.h"
#include <iostream>
#include <Eigen/Dense>
#include <Eigen/Eigenvalues>
#include "double_camera_sub/doublecamera.h"
#include "whycon_ros/MarkerArray.h"

#define num_pointarr  10

using namespace Eigen;
using namespace std;

double a1[num_pointarr][3] = { 3,2,1, 4,0,1, 5,1,1, 2,1,2, 8,0,2, 4,-3,2, 1,2,3, -1,4,3, -1,2,4, 1,3,5 };
double b1[num_pointarr][3] = { 3,-1,3, 4,-1,1, 5,-1,2, 2,-2,2, 8,-2,1, 4,-2,-2, 1,-3,3, -1,-3,5, -1,-4,3, 1,-5,4 };

int cout1, cout2;

void get_pos1(const whycon_ros::MarkerArray::ConstPtr & pose)
{
    std::cout << pose->header.stamp<<"          ";
    std:: cout <<"position1:  "<<"( "<< pose->markers[0].position.position.x <<","<< pose->markers[0].position.position.y<<","<<pose->markers[0].position.position.z<<" )"<<std::endl;
    std::cout<<endl;
}

void get_pos2(const whycon_ros::MarkerArray::ConstPtr & pose)
{        
    std::cout << pose->header.stamp<<"          ";  
    std:: cout <<"position2:  "<<"( "<< pose->markers[0].position.position.x <<","<< pose->markers[0].position.position.y<<","<<pose->markers[0].position.position.z<<" )"<<std::endl;
    std::cout<<endl;
}

int main(int argc, char  *argv[])
{
    setlocale(LC_ALL, "  ");

    ros::init(argc, argv , "talker");

    ros::NodeHandle nh;

    ros::Subscriber sub1 = nh.subscribe("/c1/whycon_ros/markers",20, get_pos1);
    ros::Subscriber sub2 = nh.subscribe("/c2/whycon_ros/markers",20, get_pos2);
    
    cout1 = cout2 = 0;
     
     ros::Rate r(1);
     while(ros::ok())
   {
     ros::spinOnce();
     r.sleep();
   }
}