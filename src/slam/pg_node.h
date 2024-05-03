#pragma once

#include "eigen3/Eigen/Dense"
#include "eigen3/Eigen/Geometry"

#include <vector>
#include "shared/math/math_util.h"
#include "shared/math/poses_2d.h"

namespace slam
{

    class PgNode
    {
    public:
        /**
         * Create the Pg node.
         *
         * @param init_pos                  Initial pose estimate for the node.
         * @param node_number               Node number (to be used in factor graph).
         * @param point_cloud               Point cloud for the node.
         */

        PgNode(const pose_2d::Pose2Df &node_pose_, const uint32_t &node_number, std::vector<Eigen::Vector2f> point_cloud)
            : node_pose_(node_pose_), node_number_(node_number), point_cloud(point_cloud)
        {
        }

        /**
         * Set the pose of the node.
         *
         * This should be called after a pose-graph optimization is called.
         *
         * @param new_loc           New location for the node.
         * @param new_orientation   New orientation for the node.
         */
        void setPose(const Eigen::Vector2f &new_loc, const float &new_orientation)
        {
            node_pose_.Set(new_orientation, new_loc);
        }

        /**
         * Set point cloud for the node.
         *
         */
        void setPointCloud(const std::vector<Eigen::Vector2f> &_point_cloud)
        {
            point_cloud = _point_cloud;
        }

        /**
         * Get the point cloud
         * @return
         */
        std::vector<Eigen::Vector2f> getPointCloud() const
        {
            return point_cloud;
        }

        /**
         * Get the estimated position of the node.
         * @return
         */
        pose_2d::Pose2Df getEstimatedPose() const
        {
            return node_pose_;
        }

        /**
         * Number of the node. Provides identifier in factor graph.
         *
         * @return node number.
         */
        uint64_t getNodeNumber() const
        {
            return node_number_;
        }

    private:
        /**
         * Pose of the node, as computed by the pose graph algorithm.
         */

        pose_2d::Pose2Df node_pose_;

        /**
         * Number of the node.
         */
        uint64_t node_number_;

        /**
         * Observation of the node.
         */
        std::vector<Eigen::Vector2f> point_cloud;
    };
} // end dpg_slam
