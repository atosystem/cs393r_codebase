
#include <ros/ros.h>
#include "gflags/gflags.h"
#include <std_msgs/Empty.h>

#include <mutex>
#include <condition_variable>
#include <signal.h>

#include "ros/node_handle.h"
#include "rosbag/bag.h"
#include "rosbag/view.h"
#include "sensor_msgs/LaserScan.h"
#include "nav_msgs/Odometry.h"

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

ros::Publisher odometry_publisher_;
ros::Publisher laser_publisher_;

void SlamDoneCallback(const std_msgs::Empty &empty_msg) {
    {
        std::lock_guard<std::mutex> lk(slam_done_mutex_);
        slam_ready_ = true;
    }
    slam_done_cv_.notify_one();
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
        printf("Unable to read %s, reason %s:",FLAGS_rosbag_fp.c_str(), exception.what());
        printf("\n");
        return;
    }

    // Get the topics we want
    vector<string> topics;
    topics.emplace_back(FLAGS_laser_topic);
    topics.emplace_back(FLAGS_odom_topic);
    rosbag::View view(bag, rosbag::TopicQuery(topics));
    printf("Bag file has %d scans\n", view.size());

    for (rosbag::View::iterator it = view.begin(); it != view.end(); ++it) {
    const rosbag::MessageInstance& message = *it;
    {   
        if (message.getTime() < ros::Time(1714267872.593701780)) {
            continue;
        }
        ROS_INFO_STREAM(message.getTime());
         if (message.getTopic() == "/odom") {
            ROS_INFO_STREAM("Odom");
            // nav_msgs::OdometryPtr odom_msg = message.instantiate<nav_msgs::Odometry>();
            // odometry_publisher_.publish(*odom_msg);
            // OdometryCallback(*odom_msg);

        } else if (message.getTopic() == "/scan") {
            ROS_INFO_STREAM("Scan");
            // sensor_msgs::LaserScanPtr scan_msg = message.instantiate<sensor_msgs::LaserScan>();
            // laser_publisher_.publish(*scan_msg);
            // LaserCallback(*scan_msg);
        }
        
      // Load all the point clouds into memory.
    //   sensor_msgs::LaserScanPtr laser_scan =
    //       message.instantiate<sensor_msgs::LaserScan>();
    //   if (laser_scan != nullptr) {
    //     // Process the laser scan
    //     // check if the timestamp lines up
    //     double scan_time =
    //         (laser_scan->header.stamp - view.getBeginTime()).toSec();
    //     if (abs(scan_time - base_timestamp) <= 1e-1) {
    //       printf("Found Base Scan %f\n", scan_time);
    //       baseCloud = pointcloud_helpers::LaserScanToPointCloud(
    //           *laser_scan, laser_scan->range_max, truncate_scan_angles);
    //     }
    //     if (abs(scan_time - match_timestamp) <= 1e-1) {
    //       printf("Found Match Scan %f\n", scan_time);
    //       matchCloud = pointcloud_helpers::LaserScanToPointCloud(
    //           *laser_scan, laser_scan->range_max, truncate_scan_angles);
    //     }
    //   }
    }
  }


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

    odometry_publisher_ = n.advertise<nav_msgs::Odometry>("/odom",1);
    laser_publisher_ = n.advertise<sensor_msgs::LaserScan>("/scan", 1);
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
