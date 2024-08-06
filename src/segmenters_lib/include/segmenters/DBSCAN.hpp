#ifndef SEGMENTERS_INCLUDE_SEGMENTERS_DBSCAN_SEGMENTER_HPP_
#define SEGMENTERS_INCLUDE_SEGMENTERS_DBSCAN_SEGMENTER_HPP_

#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/octree/octree.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <vector>
#include <string>

#include "segmenters/base_segmenter.hpp"
#include "common/types/type.h"

namespace autosense {
namespace segmenter {

struct cluster {
    int index;
    int dense_index;
    double dense;
    int height_index;
    double height;
    int pose_index;
    double pose;
    double score;
};

class DBSCAN : public BaseSegmenter {
 public:
    DBSCAN();
    DBSCAN(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, double eps, int minPts);
    explicit DBSCAN(const SegmenterParams& params);
    ~DBSCAN();

    void segment(const PointICloud &cloud_in, std::vector<PointICloudPtr> &cloud_clusters) override;

    std::string name() const override;

    void start_scan();

 private:
    SegmenterParams params_;

    double MinPts;
    double eps;
    double MinbPts = 22;
    clock_t start, end;

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_;
    pcl::PointCloud<pcl::PointXYZ>::Ptr origin_cloud_;
    std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> result_cloud_;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cluster_center;
    std::vector<int> cluster_type;
    std::vector<int> core_points;
    std::vector<int> bound_points;
    std::vector<int> result_points;
    bool view_on = 1;
    std::vector<Eigen::Vector4f> cluster_centroid;
    int method_ = 0;
    int use_edge = 1;
    int sum_points = 0;
    std::vector<int> points_num;
    const int CORE_POINT = 0;
    const int BOUND_POINT = 1;
    const int NOISE_POINT = 2;
    const int KD_TREE = 0;
    const int OCT_TREE = 1;
    std::vector<cluster> cluster_score;
    std::vector<std::vector<int>> neighbourPoints;
    std::vector<std::vector<float>> neighbourDistance;

    void select_kernel();
    void find_independent();
    std::vector<int> vectors_set_diff(std::vector<int> v1, std::vector<int> v2);
    std::vector<int> vectors_intersection(std::vector<int> v1, std::vector<int> v2);
    std::vector<int> vectors_set_union(std::vector<int> v1, std::vector<int> v2);
    void normal(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud);
    void curve(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud);
    void if_continue(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud1, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud2);
};

}  // namespace segmenter
}  // namespace autosense

#endif  // SEGMENTERS_INCLUDE_SEGMENTERS_DBSCAN_SEGMENTER_HPP_
