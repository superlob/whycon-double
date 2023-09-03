#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
 
int main(int argc, char** argv) {
    ros::init(argc, argv, "marker_worker");
    ros::NodeHandle nh;
    ros::Publisher markerArrayPub = nh.advertise<visualization_msgs::MarkerArray>("visualization_marker", 1);
 
    ros::Rate r(1);
    int i = 0 ,j = 0.1;
    visualization_msgs::MarkerArray marker_array;
    while(ros::ok()) {
            visualization_msgs::Marker marker;
            marker.action = visualization_msgs::Marker::ADD;
            marker.header.frame_id = "map";
            marker.header.stamp = ros::Time::now();
            marker.ns = "path";
            marker.id =i++;
            marker.type = visualization_msgs::Marker::SPHERE;
            marker.scale.x = 0.2;
            marker.scale.y = 0.2;
            marker.scale.z = 0.2;
            marker.color.a = 2;
            marker.color.r = 255;
            marker.color.g = 4;
            marker.color.b = 4;
            marker.lifetime = ros::Duration();
            marker.pose.position.y = i;
            marker.pose.position.x = j*i*i;
            j = j+0.1;
            marker_array.markers.push_back(marker);
        markerArrayPub.publish(marker_array);
        r.sleep();
    }
    return 0;
}

