#include "ros/ros.h"
#include "double_camera_sub/doublecamera.h"
#include <message_filters/subscriber.h>
 #include <message_filters/synchronizer.h>
 #include <message_filters/time_synchronizer.h>
 #include <message_filters/sync_policies/approximate_time.h>
 #include <tf2_ros/static_transform_broadcaster.h>
 #include <geometry_msgs/TransformStamped.h>
 #include <visualization_msgs/Marker.h>
 #include <visualization_msgs/MarkerArray.h>
#include <iostream>
#include <math.h>
#include <Eigen/Dense>
#include <Eigen/Eigenvalues>
#include "whycon_ros/MarkerArray.h"


using namespace message_filters;
using namespace std;

#define numpos 56 //点对数

double temp1[3]= {0}, temp2[3] ={0};
double c1[numpos][3] = {0} ,c2[numpos][3] ={0};  //存放n对匹配点
int i = 0;

void pos_get_callback(const whycon_ros::MarkerArray::ConstPtr & pose1,const whycon_ros::MarkerArray::ConstPtr & pose2)
{ 
       //缓存一对点
          temp1[0] = pose1->markers[0].position.position.x;
          temp1[1] = pose1->markers[0].position.position.y;
          temp1[2] = pose1->markers[0].position.position.z;

          temp2[0] = pose2->markers[0].position.position.x;
          temp2[1] = pose2->markers[0].position.position.y;
          temp2[2] = pose2->markers[0].position.position.z;

          if( temp1[2] != 0 && temp2[2] != 0)   // marker只被一个相机观测到了，而不在另一个相机的视野中，即temp1,temp2不是一对匹配点。
        {          
              if(i<numpos  )
            {
                c1[i][0] = temp1[0],c1[i][1] = temp1[1],c1[i][2] = temp1[2];
                cout<< pose1->header.stamp<<"           " << c1[i][0] <<" ," <<c1[i][1] <<" ,"<<c1[i][2]<<endl;
                c2[i][0] = temp2[0],c2[i][1] = temp2[1],c2[i][2] = temp2[2];
                cout<< pose2->header.stamp<<"           " << c2[i][0] <<" ," <<c2[i][1] <<" ,"<<c2[i][2]<<endl;
                cout<<endl;
                i++;
            }
        }
}

int main(int argc, char  *argv[])
{
    ros::init(argc,argv, "huhaha");
    
    ros::NodeHandle nh;
    
    //在一个回调函数中同步两个话题
    message_filters::Subscriber<whycon_ros::MarkerArray>  c1_pos(nh, "/c1/whycon_ros/markers",1);
    message_filters::Subscriber<whycon_ros::MarkerArray>  c2_pos(nh, "/c2/whycon_ros/markers",1);  

    typedef sync_policies ::ApproximateTime<whycon_ros::MarkerArray,whycon_ros::MarkerArray> syncPolicy;
    Synchronizer<syncPolicy> sync(syncPolicy(10), c1_pos, c2_pos);  
    sync.registerCallback(boost::bind(&pos_get_callback, _1, _2));
    
    //取匹配点，填充数组c1,c2
    doublecamera d1;
    ros::Rate loop_rate(1);
    while(ros::ok())
    {  
        temp1[2] = 0, temp2[2] = 0;
        ros::spinOnce();
        if(i >= numpos)
        {
            cout <<endl<<endl<< "position finished"<<endl<<endl;
            break;
        }
        loop_rate.sleep();
    }
     
     //计算R和t
      d1.get_RT(c1,c2);
      cout << "R  : " << endl << d1.R <<endl;
      cout << "t  : "  << endl << d1.t[0] << "      "<< d1.t[1] << "        " << d1.t[2] << endl ;
      
    //将R和t发布到rviz的tf2组件中
      tf2_ros::StaticTransformBroadcaster pub;
      Eigen::Quaterniond q_odom_tmp;
      q_odom_tmp = Eigen::Quaterniond(d1.R);
      q_odom_tmp.normalize();

      geometry_msgs::TransformStamped tfs;
      tfs.header.stamp = ros::Time::now();
      tfs.header.frame_id = "base_link";
      tfs.child_frame_id = "doub1";
      tfs.transform.translation.x = d1.t[0], tfs.transform.translation.y = d1.t[1], tfs.transform.translation.z = d1.t[2];
      tfs.transform.rotation.x = q_odom_tmp.x(), tfs.transform.rotation.y = q_odom_tmp.y(), tfs.transform.rotation.z = q_odom_tmp.z(), tfs.transform.rotation.w = q_odom_tmp.w();
      
      pub.sendTransform(tfs);

    //将c1全局坐标系的marker和c2坐标系下转化后的marker数据发布到rviz的markerarrey组件
      cout<<endl<<endl<<"start transformaton? "<<"   [yes  or  no]   "<<endl;
      char j;
      cin >>  j ;
      //ros::Publisher MarkerArreyPub1 = nh.advertise<visualization_msgs::MarkerArray> ("visualization_marker1", 10);
      //ros::Publisher MarkerArreyPub2 = nh.advertise<visualization_msgs::MarkerArray> ("visualization_marker2", 10);

      ros::Publisher MarkerArreyPub1 = nh.advertise<visualization_msgs::Marker> ("visualization_marker1", 10);
      ros::Publisher MarkerArreyPub2 = nh.advertise<visualization_msgs::Marker> ("visualization_marker2", 10);

    //  visualization_msgs::MarkerArray marker_arrey1;
    //  visualization_msgs::MarkerArray marker_arrey2;

      int k = 0,n=0;
      ros::Rate rate(1);
      while(ros::ok())
      {
               i= 0;
               temp1[2] = 0, temp2[2] = 0;
               ros::spinOnce();

               if(temp1[2] == 0 && temp2[2] == 0 )
                continue;

              visualization_msgs::Marker marker1;
              visualization_msgs::Marker marker2;
              marker1.action = marker2.action=visualization_msgs::Marker::ADD;
               marker1.header.frame_id=marker2.header.frame_id= "base_link";
               marker1.header.stamp= marker2.header.stamp=ros::Time::now();
               marker1.id =0,marker2.id = 0;
               marker1.type=marker2.type = visualization_msgs::Marker::SPHERE;
               marker1.scale.x = marker2.scale.x = 0.4;
               marker1.scale.y = marker2.scale.y = 0.4;
               marker1.scale.z = marker2.scale.z = 0.4;

               if(temp1[2] != 0)
               {
                      marker1.color.a = 2, marker1.color.r = 255, marker1.color.g = 4, marker1.color.b = 4;
                      marker1.pose.position.x = temp1[0], marker1.pose.position.y = temp1[1], marker1.pose.position.z = temp1[2];
                      //marker_arrey1.markers.push_back(marker1);
                      cout<<"push1 ++"<<endl;
               }

               if( temp2[2] != 0)
               {
                      marker2.color.a = 1, marker2.color.r = 1, marker2.color.g = 0, marker2.color.b = 255;
                      marker2.pose.position.x =  d1.R (0,0) * temp2[0] + d1. R (0,1) * temp2[1] + d1.R(0,2)*temp2[2] +d1.t[0];
                      marker2.pose.position.y =  d1.R (1,0) * temp2[0] + d1. R (1,1) * temp2[1] + d1.R(1,2)*temp2[2] +d1.t[1];
                      marker2.pose.position.z =  d1.R (2,0) * temp2[0] + d1. R (2,1) * temp2[1] + d1.R(2,2)*temp2[2] +d1.t[2];
                     // marker_arrey2.markers.push_back(marker2);
                      cout<<"push2 ++"<<endl;
               }
      MarkerArreyPub1.publish(marker1);
      MarkerArreyPub2.publish(marker2);
      rate.sleep();
      }
      cout<< "exit?"<<endl;
      int e;
      cin >> e;

    return 0;
}
