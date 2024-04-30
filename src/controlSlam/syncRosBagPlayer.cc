
#include <ros/ros.h>
#include "gflags/gflags.h"
#include <std_msgs/Empty.h>

#include <mutex>
#include <condition_variable>
#include <signal.h>

#include "ros/node_handle.h"
#include "rosbag/bag.h"
#include "rosbag/view.h"

#include <algorithm>
#include <csignal>
#include <fstream>
#include <iostream>
#include <iterator>
#include <vector>

using std::string;
using std::vector;

DEFINE_string(stopSlam_topic, "/stop_slam", "Name of ROS topic for stop slam");
DEFINE_string(rosbag_fp, "example.bag", "Filename of the ROS bag");

DEFINE_string(laser_topic, "/scan", "Name of ROS topic for LIDAR data");
DEFINE_string(odom_topic, "/odom", "Name of ROS topic for odometry data");

std::mutex slam_done_mutex_;
std::condition_variable slam_done_cv_;
bool slam_ready_ = false;

bool run_ = true;

ros::Publisher stopSlam_pub_;
ros::Publisher stopSlam_pub_;

void SlamDoneCallback(const std_msgs::Empty &empty_msg) {
    {
        std::lock_guard<std::mutex> lk(slam_done_mutex_);
        slam_ready_ = true;
    }
    slam_done_cv_.notify_one();
}



void sendStopSlam() {
    ROS_INFO_STREAM("Sending StopSlam");
    stopSlam_pub_.publish(std_msgs::Empty());
}


void SignalHandler(int) {
    if (!run_) {
        printf("Force Exit.\n");
        exit(0);
    }
    {
        std::lock_guard<std::mutex> lk(slam_done_mutex_);
        slam_ready_ = true;
    }
    slam_done_cv_.notify_one();
    printf("Exiting.\n");
    run_ = false;
}

void playBag() {
    ROS_INFO_STREAM("Play RosBag" << FLAGS_rosbag_fp);

    rosbag::Bag bag;
    try {
        bag.open(FLAGS_rosbag_fp, rosbag::bagmode::Read);
    } catch (rosbag::BagException& exception) {
        printf("Unable to read %s, reason %s:", bag_path.c_str(), exception.what());
        return;
    }

    // Get the topics we want
    vector<string> topics;
    topics.emplace_back(FLAGS_laser_topic);
    topics.emplace_back(FLAGS_odom_topic);
    rosbag::View view(bag, rosbag::TopicQuery(topics));
    printf("Bag file has %d scans\n", view.size());

    bag.close();
    printf("Done.\n");
    fflush(stdout);


}

int main(int argc, char** argv) {
    google::ParseCommandLineFlags(&argc, &argv, false);
    signal(SIGINT, SignalHandler);

    // Initialize ROS.
    ros::init(argc, argv, "stopSlam_controller");
    ros::NodeHandle n;


    // stopSlam_pub_ = n.advertise<std_msgs::Empty>(FLAGS_stopSlam_topic.c_str(), 1);

    // while (stopSlam_pub_.getNumSubscribers() == 0) {
    //     ros::Duration(0.1).sleep();
    // }

    // ros::Subscriber optimization_done_sub = n.subscribe(
    //         "reoptimization_complete",
    //         1,
    //         ReoptimizationCompleteCallback);

    ros::AsyncSpinner spinner(2); // TODO do we need this to be this high?
    spinner.start();
    playBag();

    return 0;
}
