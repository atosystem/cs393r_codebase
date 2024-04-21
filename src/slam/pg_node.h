#pragma once

#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Geometry>

#include <vector>

namespace slam
{

    class PgNode
    {
    public:
        /**
         * Create the Pg node.
         *
         * @param init_pos                  Initial position estimate for the node.
         * @param init_orientation          Initial orientation estimate for the node.
         * @param node_number               Node number (to be used in factor graph).
         * @param point_cloud               Point cloud for the node.
         */

        PgNode(const Eigen::Vector2f &init_pos, const float &init_orientation, const uint32_t &node_number)
            : node_loc_(init_pos), node_orientation_(init_orientation), node_number_(node_number)
        {
        }

        /**
         * Set the position of the node.
         *
         * This should be called after a pose-graph optimization is called.
         *
         * @param new_loc           New location for the node.
         * @param new_orientation   New orientation for the node.
         */
        void setPosition(const Eigen::Vector2f &new_loc, const float &new_orientation)
        {
            node_loc_ = new_loc;
            node_orientation_ = new_orientation;
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
        std::pair<Eigen::Vector2f, float> getEstimatedPosition() const
        {
            return std::make_pair(node_loc_, node_orientation_);
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
         * Location of the node, as computed by the pose graph algorithm.
         */
        Eigen::Vector2f node_loc_;

        /**
         * Orientation of the node, as computed by the pose graph algorithm.
         */
        float node_orientation_;

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
