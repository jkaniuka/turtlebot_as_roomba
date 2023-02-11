#include "ros/ros.h"
#include "std_msgs/String.h"
#include <sstream>
#include "sensor_msgs/LaserScan.h"
#include <package_303762/scan_min_array.h>
#include <iostream>
#include <iterator>

using namespace std;

int sector_size;
int bins; // number of sectors

package_303762::scan_min_array laser_msg; // my custom message 

void scanCallback(const sensor_msgs::LaserScan::ConstPtr& msg)
{
     
    laser_msg.minranges.resize(bins); 

    auto array_length = end(msg->ranges) - begin(msg->ranges);

    sector_size = floor(array_length/bins);

    for(int i = 0; i < bins; i++){
     float local_min_val = msg->ranges[0];
     // looking for min value
     for(int j=sector_size*i; j<sector_size*(i+1); j++) {
        if(local_min_val>msg->ranges[j]) {
           local_min_val=msg->ranges[j];
        }
     }
      laser_msg.minranges[i] = local_min_val;
    }
      

}

int main(int argc, char **argv)
{

  ros::init(argc, argv, "scan_analyzer");   
  ros::NodeHandle n("~");
  n.getParam("num_of_sectors", bins); 
  ros::Subscriber sub = n.subscribe("/scan", 1000, scanCallback);
  ros::Publisher sectors_pub = n.advertise<package_303762::scan_min_array>("/vacuum_sensors", 1000);

  ros::Rate loop_rate(5);   


  while (ros::ok())
  {

    sectors_pub.publish(laser_msg);
    ros::spinOnce();

    loop_rate.sleep();

  }


  return 0;
}
