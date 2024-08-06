/*
 * Copyright (C) 2019 by AutoSense Organization. All rights reserved.
 * Gary Chan <chenshj35@mail2.sysu.edu.cn>
 */

#include <pcl_conversions/pcl_conversions.h>  // pcl::fromROSMsg
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <std_msgs/Header.h>
#include "std_msgs/Int16.h"

//for debug msg
#include <sensor_msgs/JointState.h>
#include <visualization_msgs/MarkerArray.h>
#include <memory>
#include "tracking_msg/TrackingObjectArray.h"


#include "common/msgs/autosense_msgs/PointCloud2Array.h"

#include "common/parameter.hpp"              // common::getSegmenterParams
#include "common/publisher.hpp"              // common::publishCloud
#include "common/time.hpp"                   // common::Clock
#include "common/types/type.h"               // PointICloudPtr
#include "segmenters/segmenter_manager.hpp"  // segmenter::createGroundSegmenter

#include "common/color.hpp"
#include "object_builders/object_builder_manager.hpp"
#include "segmenters/DBSCAN.hpp"

#include "roi_filters/roi.hpp"  // roi::applyROIFilter

using namespace autosense;

const std::string param_ns_prefix_ = "detect";  // NOLINT
std::string frame_id_;                          // NOLINT

bool use_voxelGrid_;
float voxel_size;
int min_points_per_voxel;
bool use_roi_filter_;
autosense::ROIParams params_roi_;
bool use_non_ground_segmenter_;
bool is_object_builder_open_;
int sub_pc_angle;
int sub_pc_queue_size;
float car_vel; //Avg car velocity
// ROS Subscriber
ros::Subscriber pointcloud_sub_;
ros::Subscriber vel_sub_;
// ROS Publisher
ros::Publisher pcs_segmented_pub_;
ros::Publisher lane_pub_;
/// @note Core components
boost::shared_ptr<autosense::segmenter::BaseSegmenter> ground_remover_;
boost::shared_ptr<autosense::segmenter::BaseSegmenter> segmenter_;

void OnPointCloud(const sensor_msgs::PointCloud2ConstPtr &ros_pc2) {

    autosense::common::Clock clock;

    autosense::PointICloudPtr cloud(new autosense::PointICloud);
    pcl::fromROSMsg(*ros_pc2, *cloud);

    /*Eigen::Matrix4f tilt_matrix = Eigen::Matrix4f::Identity();
    tilt_matrix.topLeftCorner(3, 3) = Eigen::AngleAxisf(sub_pc_angle * M_PI / 180.0f, Eigen::Vector3f::UnitY()).toRotationMatrix();

    // filtering before RANSAC (height and normal filtering)
    pcl::transformPointCloud(*cloud, *cloud, tilt_matrix);
*/
    std_msgs::Header header = ros_pc2->header;
    header.frame_id = frame_id_;
    header.stamp = ros::Time::now();

    if (use_voxelGrid_){
        autosense::roi::voxelGridFilter<autosense::PointI>(voxel_size, min_points_per_voxel, cloud);
        //ROS_INFO("PointCloud size after VoxelGrid: %zu", cloud->size());
        //디버깅용 코드.
    }

    if (use_roi_filter_) {
        autosense::roi::applyROIFilter<autosense::PointI>(params_roi_, cloud);
    }

	if (cloud->empty()) {
		return;
	}
    std::vector<autosense::PointICloudPtr> cloud_clusters;
    autosense::PointICloudPtr cloud_ground(new autosense::PointICloud);
    autosense::PointICloudPtr cloud_nonground(new autosense::PointICloud);

    ground_remover_->segment(*cloud, cloud_clusters);
    *cloud_ground = *cloud_clusters[0];
    *cloud_nonground = *cloud_clusters[1];

    // reset clusters
    cloud_clusters.clear();
    
    segmenter_->segment(*cloud_nonground, cloud_clusters);
    autosense::common::publishPointCloudArray<autosense::PointICloudPtr>(
        pcs_segmented_pub_, header, cloud_clusters);

    common::publishLaneMarkers(
        lane_pub_, header, common::WHITE.rgbA);
}

void velcallback(const sensor_msgs::JointState msg){
    car_vel = msg.velocity[0];
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "detection_node");

    // Node handle
    ros::NodeHandle nh = ros::NodeHandle();
    ros::NodeHandle private_nh = ros::NodeHandle("~");
    ros::AsyncSpinner spiner(1);

    /// @brief Load ROS parameters from rosparam server
    
    //detection.yaml 파일안에 관련 설정 있음
    private_nh.getParam(param_ns_prefix_ + "/use_voxelGrid_", use_voxelGrid_);
    private_nh.getParam(param_ns_prefix_ + "/voxel_size", voxel_size);
    private_nh.getParam(param_ns_prefix_ + "/min_points_per_voxel", min_points_per_voxel);
    ROS_INFO("VoxelGrid parameters: voxel_size=%f, min_points_per_voxel=%d", 
                                            voxel_size, min_points_per_voxel);


    private_nh.getParam(param_ns_prefix_ + "/frame_id", frame_id_);

    std::string sub_pc_topic, pub_pcs_segmented_topic;
    
    private_nh.getParam(param_ns_prefix_ + "/sub_pc_topic", sub_pc_topic);

    private_nh.getParam(param_ns_prefix_ + "/sub_pc_queue_size",
                        sub_pc_queue_size);
    private_nh.getParam(param_ns_prefix_ + "/pub_pcs_segmented_topic",
                        pub_pcs_segmented_topic);
    private_nh.getParam(param_ns_prefix_ + "/sub_pc_angle", sub_pc_angle);

    /// @note Important to use roi filter for "Ground remover"
    private_nh.param<bool>(param_ns_prefix_ + "/use_roi_filter",
                           use_roi_filter_, false);
    params_roi_ = autosense::common::getRoiParams(private_nh, param_ns_prefix_);

    // Ground remover & non-ground segmenter
    std::string ground_remover_type, non_ground_segmenter_type;
    private_nh.param<std::string>(param_ns_prefix_ + "/ground_remover_type",
                                  ground_remover_type,
                                  "GroundPlaneFittingSegmenter");
    private_nh.param<std::string>(
        param_ns_prefix_ + "/non_ground_segmenter_type",
        non_ground_segmenter_type, "RegionEuclideanSegmenter");
    autosense::SegmenterParams param =
        autosense::common::getSegmenterParams(private_nh, param_ns_prefix_);

    param.segmenter_type = ground_remover_type;
    ground_remover_ = autosense::segmenter::createGroundSegmenter(param);

    param.segmenter_type = non_ground_segmenter_type;
    segmenter_ = autosense::segmenter::createNonGroundSegmenter(param);

    vel_sub_  = nh.subscribe<sensor_msgs::JointState>("/Joint_state", 1, velcallback);
    pointcloud_sub_ = nh.subscribe<sensor_msgs::PointCloud2>(
        sub_pc_topic, 1, OnPointCloud);

    std::string pub_lane_marker;
    pub_lane_marker = "/lane_marker";
    lane_pub_ = nh.advertise<visualization_msgs::Marker>(
            pub_lane_marker, 1);    

    pcs_segmented_pub_ = nh.advertise<autosense_msgs::PointCloud2Array>(
        pub_pcs_segmented_topic, 1);


    spiner.start();
    ROS_INFO("detection_node started...");

    ros::waitForShutdown();
    ROS_INFO("detection_node exited...");

    return 0;
}
