
#include <ros/ros.h>
#include "gflags/gflags.h"
#include <std_msgs/Empty.h>
#include <mutex>
#include <condition_variable>
#include <signal.h>

DEFINE_string(stopSlam_topic, "/stop_slam", "Name of ROS topic for stop slam");


// std::mutex reoptimization_done_mutex_;
// std::condition_variable reoptimization_done_cv_;
// bool dpg_ready_ = false;
bool run_ = true;

ros::Publisher stopSlam_pub_;

// void ReoptimizationCompleteCallback(const std_msgs::Empty &empty_msg) {
//     {
//         std::lock_guard<std::mutex> lk(reoptimization_done_mutex_);
//         dpg_ready_ = true;
//     }
//     reoptimization_done_cv_.notify_one();
// }



void sendStopSlam() {
    ROS_INFO_STREAM("Sending StopSlam");
    stopSlam_pub_.publish(std_msgs::Empty());
}



// void SignalHandler(int) {
//     if (!run_) {
//         printf("Force Exit.\n");
//         exit(0);
//     }
//     {
//         std::lock_guard<std::mutex> lk(reoptimization_done_mutex_);
//         dpg_ready_ = true;
//     }
//     reoptimization_done_cv_.notify_one();
//     printf("Exiting.\n");
//     run_ = false;
// }

int main(int argc, char** argv) {
    google::ParseCommandLineFlags(&argc, &argv, false);
    // signal(SIGINT, SignalHandler);

    // Initialize ROS.
    ros::init(argc, argv, "stopSlam_controller");
    ros::NodeHandle n;


    stopSlam_pub_ = n.advertise<std_msgs::Empty>(FLAGS_stopSlam_topic.c_str(), 1);

    while (stopSlam_pub_.getNumSubscribers() == 0) {
        ros::Duration(0.1).sleep();
    }

    // ros::Subscriber optimization_done_sub = n.subscribe(
    //         "reoptimization_complete",
    //         1,
    //         ReoptimizationCompleteCallback);

    ros::AsyncSpinner spinner(2); // TODO do we need this to be this high?
    spinner.start();
    sendStopSlam();
    // if (run_mit) {
    //     ROS_INFO_STREAM("Run on MIT dataset");
    //     runOnMitRosBags();
    // } else {
    //     ROS_INFO_STREAM("Run on GDC dataset");
    //     runOnGdcRosBags();
    // }

    return 0;
}
