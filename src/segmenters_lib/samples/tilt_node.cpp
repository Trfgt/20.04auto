/*
 * Copyright (C) 2019 by AutoSense Organization. All rights reserved.
 * Gary Chan <chenshj35@mail2.sysu.edu.cn>
 */

#include <pcl_conversions/pcl_conversions.h>  // pcl::fromROSMsg
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <std_msgs/Header.h>
#include <visualization_msgs/MarkerArray.h>
#include <memory>

#include <pcl_ros/point_cloud.h>

#include <pcl/common/transforms.h>
#include <pcl/features/normal_3d.h>
#include <pcl/search/impl/search.hpp>
#include <pcl/filters/impl/plane_clipper3D.hpp>
#include <pcl/filters/extract_indices.h>
#include <pcl/sample_consensus/ransac.h>
#include <pcl/sample_consensus/sac_model_plane.h>

#include "common/color.hpp"
#include "common/parameter.hpp"  // common::getSegmenterParams
#include "common/publisher.hpp"  // common::publishCloud
#include "common/time.hpp"       // common::Clock
#include "common/types/type.h"   // PointICloudPtr
#include "object_builders/object_builder_manager.hpp"
#include "roi_filters/roi.hpp"               // roi::applyROIFilter
#include "segmenters/segmenter_manager.hpp"  // segmenter::createGroundSegmenter

using namespace autosense;  // NOLINT

const std::string param_ns_prefix_ = "detect";  // NOLINT
std::string frame_id_ = "";  // NOLINT
bool use_roi_filter_;
ROIParams params_roi_;
ROIParams params_roi_2;
bool use_non_ground_segmenter_;
bool is_object_builder_open_;
// ROS Subscriber
ros::Subscriber pointcloud_sub_;
// ROS Publisher
ros::Publisher tiltPointCloud_pub_;
ros::Publisher nonground_pub_;
ros::Publisher clusters_pub_;
ros::Publisher objects_pub_;
//hyunjin for lane publisher 0627
ros::Publisher lane_pub_;
ros::Publisher Roi_pub_;
/// @note Core components
std::unique_ptr<segmenter::BaseSegmenter> ground_remover_;
std::unique_ptr<segmenter::BaseSegmenter> segmenter_;
std::unique_ptr<object_builder::BaseObjectBuilder> object_builder_;

void tiltPointCloud(const sensor_msgs::PointCloud2ConstPtr& cloud_msg) {
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>());
    pcl::fromROSMsg(*cloud_msg, *cloud);
    //tilting here
    Eigen::Matrix4f tilt_matrix = Eigen::Matrix4f::Identity();
    tilt_matrix.topLeftCorner(3, 3) = Eigen::AngleAxisf(tilt_deg * M_PI / 180.0f, Eigen::Vector3f::UnitY()).toRotationMatrix();

    // filtering before RANSAC (height and normal filtering)
    pcl::PointCloud<PointXYZI>::Ptr tilted(new pcl::PointCloud<PointT>);
    pcl::transformPointCloud(*cloud, *tilted, tilt_matrix);

}

int main(int argc, char **argv) {
    ros::init(argc, argv, "tilt_node");

    // Node handle
    ros::NodeHandle nh = ros::NodeHandle(), private_nh = ros::NodeHandle("~");
    std::string sub_pc_topic, pub_pc_tilt_topic;
    int sub_pc_queue_size;
    /// @brief Load ROS parameters from rosparam server
    private_nh.getParam(param_ns_prefix_ + "/frame_id", frame_id_);

    private_nh.getParam(param_ns_prefix_ + "/pub_pc_tilt_topic",
                        pub_pc_tilt_topic);
    private_nh.getParam(param_ns_prefix_ + "/sub_pc_topic", sub_pc_topic);
    private_nh.getParam(param_ns_prefix_ + "/sub_pc_queue_size",
                        sub_pc_queue_size);
    tilt_deg = private_nh.param<double>("tilt_deg", -10.0);  

    tiltPointCloud_pub_ =
            nh.advertise<sensor_msgs::PointCloud2>(pub_pc_tilt_topic, 1);

    pointcloud_sub_ = nh.subscribe<sensor_msgs::PointCloud2>(
            sub_pc_topic, sub_pc_queue_size, tiltPointCloud);
     
    ROS_INFO("detection_node started...");

    ros::Rate fps(10);
    while (ros::ok()) {
        ros::spinOnce();
        //fps.sleep();
    }

    ROS_INFO("detection_node exited...");

    return 0;
}
