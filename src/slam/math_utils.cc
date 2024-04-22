
#include "math_utils.h"

namespace math_utils {

    // trasfrom a 2D pose from src frame to map frame
    pose_2d::Pose2Df transformPoseFromSrc2Map(const pose_2d::Pose2Df & pose_rel_src_frame, const pose_2d::Pose2Df & src_frame_pose_rel_map_frame) {
        // Rotate the point first
        Eigen::Rotation2Df rotation_mat(src_frame_pose_rel_map_frame.angle);
        Eigen::Vector2f rotated_still_src_transl = rotation_mat * pose_rel_src_frame.translation;

        // Then translate
        Eigen::Vector2f rotated_and_translated = src_frame_pose_rel_map_frame.translation + rotated_still_src_transl;
        float target_angle = AngleMod(src_frame_pose_rel_map_frame.angle + pose_rel_src_frame.angle);

        return std::make_pair(rotated_and_translated, target_angle);

    }

    // trasfrom a 2D pose from map frame to target frame
    pose_2d::Pose2Df transformPoseFromMap2Target(const pose_2d::Pose2Df & pose_rel_map_frame, const pose_2d::Pose2Df & target_frame_pose_rel_map_frame) {
        // Translate the point
        Eigen::Vector2f trans = pose_rel_map_frame.translation - target_frame_pose_rel_map_frame.translation;

        // Then rotate
        Eigen::Rotation2Df rot_mat(-target_frame_pose_rel_map_frame.angle);
        Eigen::Vector2f final_trans = rot_mat * trans;

        float final_angle = AngleMod(pose_rel_map_frame.angle - target_frame_pose_rel_map_frame.angle);

        return pose_2d::Pose2Df(final_angle,final_trans);
    }



    std::pair<Eigen::Vector2f, float> inverseTransformPoint(const Eigen::Vector2f &src_frame_point,
                                                     const float &src_frame_angle,
                                                     const Eigen::Vector2f &target_frame_pos_rel_src_frame,
                                                     const float &target_frame_angle_rel_src_frame) {

        // Translate the point
        Eigen::Vector2f translated = src_frame_point - target_frame_pos_rel_src_frame;

        // Then rotate
        Eigen::Rotation2Df rotation_mat(-target_frame_angle_rel_src_frame);
        Eigen::Vector2f rotated_and_translated = rotation_mat * translated;

        float target_angle = AngleMod(src_frame_angle - target_frame_angle_rel_src_frame);
        return std::make_pair(rotated_and_translated, target_angle);
    }
}
