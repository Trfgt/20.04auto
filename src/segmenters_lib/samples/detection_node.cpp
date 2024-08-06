/*
 * Copyright (C) 2019 by AutoSense Organization. All rights reserved.
 * Gary Chan <chenshj35@mail2.sysu.edu.cn>
 */

#include <pcl_conversions/pcl_conversions.h>  // pcl::fromROSMsg
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <std_msgs/Header.h>
#include "std_msgs/Int16.h"
#include "std_msgs/Float32MultiArray.h"
#include <sensor_msgs/JointState.h>
#include <visualization_msgs/MarkerArray.h>
#include <memory>

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
int sub_pc_angle;
float car_vel; //Avg car velocity
// ROS Subscriber
ros::Subscriber pointcloud_sub_;
ros::Subscriber vel_sub_;
// ROS Publisher
ros::Publisher ground_pub_;
ros::Publisher nonground_pub_;
ros::Publisher clusters_pub_;
ros::Publisher objects_pub_;


//hyunjin for lane publisher 0627
ros::Publisher lane_pub_;
ros::Publisher warning_pub_;
ros::Publisher objects_array_pub_;
ros::Publisher Roi_pub_;
/// @note Core components
std::unique_ptr<segmenter::BaseSegmenter> ground_remover_;
std::unique_ptr<segmenter::BaseSegmenter> segmenter_;
std::unique_ptr<object_builder::BaseObjectBuilder> object_builder_;

void OnPointCloud(const sensor_msgs::PointCloud2ConstPtr& ros_pc2) {
    common::Clock clock;

    PointICloudPtr cloud(new PointICloud);

    pcl::fromROSMsg(*ros_pc2, *cloud);

    std_msgs::Header header = ros_pc2->header;
    header.frame_id = frame_id_;
    header.stamp = ros::Time::now();

    /*Eigen::Matrix4f tilt_matrix = Eigen::Matrix4f::Identity();
    tilt_matrix.topLeftCorner(3, 3) = Eigen::AngleAxisf(sub_pc_angle * M_PI / 180.0f, Eigen::Vector3f::UnitY()).toRotationMatrix();

    // filtering before RANSAC (height and normal filtering)
    pcl::transformPointCloud(*cloud, *cloud, tilt_matrix);
*/
    //ROS_INFO_STREAM(" Cloud inputs: #" << cloud->size() << " Points");

    if (use_roi_filter_) {
        roi::applyROIFilter<PointI>(params_roi_, cloud);
    }

    std::vector<PointICloudPtr> cloud_clusters;
    PointICloudPtr cloud_ground(new PointICloud);
    PointICloudPtr cloud_nonground(new PointICloud);

    ground_remover_->segment(*cloud, cloud_clusters);
    *cloud_ground = *cloud_clusters[0];
    *cloud_nonground = *cloud_clusters[1];

    // reset clusters
    cloud_clusters.clear();
    if (use_non_ground_segmenter_) {
        segmenter_->segment(*cloud_nonground, cloud_clusters);
        common::publishClustersCloud<PointI>(clusters_pub_, header,
                                             cloud_clusters);
    }

    common::publishCloud<PointI>(ground_pub_, header, *cloud_ground);
    common::publishCloud<PointI>(nonground_pub_, header, *cloud_nonground);

    if (is_object_builder_open_) {
        std::vector<autosense::ObjectPtr> objects;
        object_builder_->build(cloud_clusters, &objects);
        //common::publishObjectsMarkers(
        //    objects_pub_, header, common::MAGENTA.rgbA, objects);
        //hyunjin
        common::publishLaneMarkers(
            lane_pub_, header, common::WHITE.rgbA);
        //jaeseung
        common::publish_Minzong(
            objects_array_pub_, objects);
        common::publishRoi(
            objects_pub_, header, common::MAGENTA.rgbA, objects, car_vel);
		common::publishWarningMarkers(
            warning_pub_, header, car_vel);
    }
    //jaeseung
    /*if (use_roi_filter_) {
        roi::applyROIFilter<PointI>(params_roi_2, );
    }*/

    //ROS_INFO_STREAM("Cloud processed. Took " << clock.takeRealTime()
    //                                         << "ms.\n");
}

void velcallback(const sensor_msgs::JointState msg){
    car_vel = msg.velocity[0];
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "detection_node");

    // Node handle
    ros::NodeHandle nh = ros::NodeHandle(), private_nh = ros::NodeHandle("~");

    /// @brief Load ROS parameters from rosparam server
    private_nh.getParam(param_ns_prefix_ + "/frame_id", frame_id_);
    std::cout << frame_id_ << std::endl;
    std::string sub_pc_topic, pub_pc_ground_topic, pub_pc_nonground_topic,
            pub_pc_clusters_topic;
    int sub_pc_queue_size;
    private_nh.getParam(param_ns_prefix_ + "/sub_pc_topic", sub_pc_topic);
    private_nh.getParam(param_ns_prefix_ + "/sub_pc_queue_size",
                        sub_pc_queue_size);
    private_nh.getParam(param_ns_prefix_ + "/pub_pc_ground_topic",
                        pub_pc_ground_topic);
    private_nh.getParam(param_ns_prefix_ + "/pub_pc_nonground_topic",
                        pub_pc_nonground_topic);
    private_nh.getParam(param_ns_prefix_ + "/pub_pc_clusters_topic",
                        pub_pc_clusters_topic);
	private_nh.getParam(param_ns_prefix_ + "/sub_pc_angle", sub_pc_angle);

    /// @note Important to use roi filter for "Ground remover"
    private_nh.param<bool>(param_ns_prefix_ + "/use_roi_filter",
                           use_roi_filter_, false);
    params_roi_ = common::getRoiParams(private_nh, param_ns_prefix_);

    // warning ROI (jaeseung)
    //params_roi_2 = common::getRoiParams2(private_nh, param_ns_prefix_);

    // Ground remover & non-ground segmenter
    std::string ground_remover_type, non_ground_segmenter_type;
    private_nh.param<std::string>(param_ns_prefix_ + "/ground_remover_type",
                                  ground_remover_type,
                                  "GroundPlaneFittingSegmenter");
    private_nh.param<bool>(param_ns_prefix_ + "/use_non_ground_segmenter",
                           use_non_ground_segmenter_, false);
    private_nh.param<std::string>(
            param_ns_prefix_ + "/non_ground_segmenter_type",
            non_ground_segmenter_type, "RegionEuclideanSegmenter");
    SegmenterParams param =
            common::getSegmenterParams(private_nh, param_ns_prefix_);

    param.segmenter_type = ground_remover_type;
    ground_remover_ = segmenter::createGroundSegmenter(param);

    if (use_non_ground_segmenter_) {
        param.segmenter_type = non_ground_segmenter_type;
        segmenter_ = segmenter::createNonGroundSegmenter(param);
    }

    private_nh.param<bool>(param_ns_prefix_ + "/is_object_builder_open",
                           is_object_builder_open_, false);

    vel_sub_  = nh.subscribe<sensor_msgs::JointState>("/Joint_state", 1, velcallback);
    pointcloud_sub_ = nh.subscribe<sensor_msgs::PointCloud2>(
            sub_pc_topic, sub_pc_queue_size, OnPointCloud);    

    if (is_object_builder_open_) {
        object_builder_ = object_builder::createObjectBuilder();
        if (nullptr == object_builder_) {
            ROS_FATAL("Failed to create object_builder_.");
            return -1;
        }

        std::string pub_objects_segmented_topic , pub_lane_marker, pub_warning_marker;
        pub_lane_marker = "/lane_marker";
		pub_warning_marker = "/warning_marker";
        //std::string pub_Roi;
        private_nh.getParam(param_ns_prefix_ + "/pub_objects_segmented_topic",
                            pub_objects_segmented_topic);
        objects_pub_ = nh.advertise<visualization_msgs::MarkerArray>(
                pub_objects_segmented_topic, 1);
        //Hyunjin
        lane_pub_ = nh.advertise<visualization_msgs::Marker>(
                pub_lane_marker, 1);
        //jaeseung
		warning_pub_ = nh.advertise<visualization_msgs::Marker>(
                pub_warning_marker, 1);
        objects_array_pub_ = nh.advertise<std_msgs::Float32MultiArray>("objects_array", 1);
    }

    ground_pub_ =
            nh.advertise<sensor_msgs::PointCloud2>(pub_pc_ground_topic, 1);
    nonground_pub_ =
            nh.advertise<sensor_msgs::PointCloud2>(pub_pc_nonground_topic, 1);
    clusters_pub_ =
            nh.advertise<sensor_msgs::PointCloud2>(pub_pc_clusters_topic, 1);

     
    ROS_INFO("detection_node started...");

    ros::Rate fps(10);
    while (ros::ok()) {
        ros::spinOnce();
        //fps.sleep();
    }

    ROS_INFO("detection_node exited...");

    return 0;
}
