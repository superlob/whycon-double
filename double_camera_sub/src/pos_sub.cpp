#include "ros/ros.h"
#include "double_camera_sub/doublecamera.h"
// #include <message_filters/subscriber.h>
// #include <message_filters/synchronizer.h>
#include <iostream>
#include <Eigen/Dense>
#include <Eigen/Eigenvalues>
#include "whycon_ros/MarkerArray.h"

using namespace std;

#define numpos 40

double temp1[3]= {0}, temp2[3] ={0};

void get_pos1(const whycon_ros::MarkerArray::ConstPtr & pose)
{
        temp1[0] = pose->markers[0].position.position.x;
        temp1[1] = pose->markers[0].position.position.y;
        temp1[2] = pose->markers[0].position.position.z;
        //std:: cout <<"position1:  "<<"( "<< pose->markers[0].position.position.x <<","<< pose->markers[0].position.position.y<<","<<pose->markers[0].position.position.z<<" )"<<std::endl;
}

void get_pos2(const whycon_ros::MarkerArray::ConstPtr & pose)
{
        temp2[0] = pose->markers[0].position.position.x;
        temp2[1] = pose->markers[0].position.position.y;
        temp2[2] = pose->markers[0].position.position.z;
        //std:: cout <<"position2:  "<<"( "<< pose->markers[0].position.position.x <<","<< pose->markers[0].position.position.y<<","<<pose->markers[0].position.position.z<<" )"<<std::endl;
}

void trans_pos (double a[3], Matrix3d R, double t[3])
{
    cout << a[0] <<","<<a[1] <<","<<a[2]<<"     ";
  
    cout <<  "  ------> "<< R (0,0) * a[0] +  R (0,1) * a[1] + R(0,2)*a[2] +t[0]<<","<<R (1,0) * a[0] +  R (1,1) * a[1] + R(1,2)*a[2] +t[1]<<","<<R (2,0) * a[0] +  R (2,1) * a[1] + R(2,2)*a[2] +t[2]<<endl;

}

int main(int argc, char  *argv[])
{
    setlocale(LC_ALL, "  ");

    ros::init(argc, argv , "subscriber");

    ros::NodeHandle nh;

    ros::Subscriber sub1 = nh.subscribe("/c1/whycon_ros/markers",1, get_pos1);
    ros::Subscriber sub2 = nh.subscribe("/c2/whycon_ros/markers",1, get_pos2);
    
    double c1[numpos][3] = {0} ,c2[numpos][3] ={0};
    doublecamera d1;
    int Count = 0;
    int i =0;

    while( ros:: ok())
    {
        Count++;
        temp1[2] = 0, temp2[2] = 0;    //   
        ros::spinOnce();
        
        if( Count %15 == 0 && temp1[2] != 0 && temp2[2] != 0)
        {          
              if(i<numpos)
            {
                c1[i][0] = temp1[0],c1[i][1] = temp1[1],c1[i][2] = temp1[2];
                cout << c1[i][0] <<" ," <<c1[i][1] <<" ,"<<c1[i][2]<<endl;
                c2[i][0] = temp2[0],c2[i][1] = temp2[1],c2[i][2] = temp2[2];
                 cout << c2[i][0] <<" ," <<c2[i][1] <<" ,"<<c2[i][2]<<endl;
                i++;
            }
        }
        if(i >= numpos)
        {
            cout <<endl<<endl<< "position finished"<<endl<<endl;
            break;
        }
    }     

      d1.get_RT(c1,c2);
      cout << "R  : " << endl << d1.R <<endl;
      cout << "t  : "  << endl << d1.t[0] << "      "<< d1.t[1] << "        " << d1.t[2] << endl ;

      cout<<endl<<endl<<"start transformaton? "<<"   [yes  or  no]   "<<endl;
      char j;
      cin >>  j ;
      if( j == 'y')
      {
        while(ros::ok())
        {
           ros::spinOnce();
           trans_pos(temp1,d1.R, d1.t);
        }
      }
    return 0;
}
