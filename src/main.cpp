#include <iostream>
#include "cmd_interface_linux.h"
#include <stdio.h>
#include "lipkg.h"
#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include "tofbf.h"
#include <string>

int main(int argc , char **argv)
{
	ros::init(argc, argv, "ld06_lidar");
	ros::NodeHandle nh;                    /* create a ROS Node */
	ros::NodeHandle nh_private("~");
 	
	LiPkg * lidar = new LiPkg;
  
    	CmdInterfaceLinux cmd_port;
    	std::string port_name;
	std::string lidar_frame;

	nh_private.param<std::string>("lidar_frame", lidar_frame, "lidar_frame");
	lidar->SetLidarFrame(lidar_frame);

	nh_private.param<std::string>("serial_port", port_name, std::string());

	if (port_name.empty())
	{
		ROS_INFO("Autodetecting serial port");
		std::vector<std::pair<std::string, std::string>> device_list;
		cmd_port.GetCmdDevices(device_list);
		auto found = std::find_if(
			device_list.begin(),
			device_list.end(),
			[](std::pair<std::string, std::string> n)
			{ return strstr(n.second.c_str(), "CP2102"); });

		if (found != device_list.end())
		{
			ROS_INFO_STREAM(found->first << "    " << found->second);
			port_name = found->first;
		}
		else
		{
			ROS_ERROR("Can't find LiDAR LD06");
			return -1;
		}
	}

	ROS_INFO_STREAM("Using port " << port_name);

	cmd_port.SetReadCallback([&lidar](const char *byte, size_t len) {
		if(lidar->Parse((uint8_t*)byte, len))
		{
			lidar->AssemblePacket();
		}
	});

	if(cmd_port.Open(port_name)){
		ROS_INFO("LiDAR_LD06 started successfully");
	} else {
		ROS_ERROR("Can't open the serial port");
		return -1;
	}
	ros::Publisher lidar_pub = nh.advertise<sensor_msgs::LaserScan>("LiDAR/LD06", 1); /*create a ROS topic */
	
	ros::Rate rate(30);
	while (ros::ok())
	{
		if (lidar->IsFrameReady())
		{
			lidar_pub.publish(lidar->GetLaserScan());  // Fixed Frame:  lidar_frame
			lidar->ResetFrameReady();
		}
		rate.sleep();
	}
    return 0;
}

