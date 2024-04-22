//
// Created by amanda on 11/27/20.
//

#pragma once

#include <eigen3/Eigen/Geometry>

namespace math_utils {

    pose_2d::Pose2Df transformPoseFromSrc2Map(const pose_2d::Pose2Df & pose_rel_src_frame, 
                                            const pose_2d::Pose2Df & src_frame_pose_rel_map_frame);

    pose_2d::Pose2Df transformPoseFromMap2Target(const pose_2d::Pose2Df & pose_rel_map_frame, 
                                            const pose_2d::Pose2Df & target_frame_pose_rel_map_frame);


}