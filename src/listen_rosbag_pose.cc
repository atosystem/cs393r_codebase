#include <signal.h>
#include <stdio.h>
#include <ros/ros.h>
#include <iostream>
#include <fstream>
#include "amrl_msgs/Localization2DMsg.h"

std::ofstream output_file("reference_pose.csv");
bool run_ = true;

void PoseCallback(const amrl_msgs::Localization2DMsg& msg) {
  output_file << msg.pose.x << ',' << msg.pose.y << ',' << msg.pose.theta << '\n';
}

void SignalHandler(int) {
  if (!run_) {
    printf("Force Exit.\n");
    exit(0);
  }
  printf("Exiting.\n");
  run_ = false;
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "listen_rosbag_pose");
  ros::NodeHandle n;
  ros::Subscriber pose_sub = n.subscribe(
    "/reference_localization",
    1,
    PoseCallback);

  signal(SIGINT, SignalHandler);
  output_file << "x,y,theta\n";

  while (ros::ok() && run_) {
    ros::spinOnce();
    ros::Rate(20).sleep();
  }
  output_file.close();
  return 0;
}
