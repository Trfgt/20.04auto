/* Git참고하여 작성. with GPT.
 *https://github.com/wrld/PCL_DBSCAN/commits/master/ 
*/

#include "segmenters/DBSCAN.hpp"
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/octree/octree_search.h>
#include <pcl/features/normal_3d.h>
#include <pcl/features/principal_curvatures.h>
#include <vector>
#include <algorithm>
#include <ctime>
#include <cstdlib>
#include <iostream>

using std::vector;
using std::cout;
using std::endl;

namespace autosense {
namespace segmenter {

DBSCAN::DBSCAN() : MinPts(0), eps(0.0) {}

DBSCAN::DBSCAN(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, double eps, int minPts) 
    : cloud_(cloud), eps(eps), MinPts(minPts) {}

DBSCAN::DBSCAN(const SegmenterParams &params) : params_(params) {}

DBSCAN::~DBSCAN() {}

void DBSCAN::segment(const PointICloud &cloud_in, std::vector<PointICloudPtr> &cloud_clusters) {
    // Segment function implementation
}

std::string DBSCAN::name() const {
    return "DBSCAN";
}

void DBSCAN::normal(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud) {
    pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
    ne.setInputCloud(cloud);

    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>());
    ne.setSearchMethod(tree);

    pcl::PointCloud<pcl::Normal>::Ptr cloud_normals(new pcl::PointCloud<pcl::Normal>);
    ne.setKSearch(5);
    ne.compute(*cloud_normals);

    pcl::PrincipalCurvaturesEstimation<pcl::PointXYZ, pcl::Normal, pcl::PrincipalCurvatures> pc;
    pcl::PointCloud<pcl::PrincipalCurvatures>::Ptr cloud_curvatures(new pcl::PointCloud<pcl::PrincipalCurvatures>);
    pc.setInputCloud(cloud);
    pc.setInputNormals(cloud_normals);
    pc.setSearchMethod(tree);
    pc.setKSearch(5);
    pc.compute(*cloud_curvatures);
}

void DBSCAN::if_continue(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud1,
                         pcl::PointCloud<pcl::PointXYZ>::Ptr cloud2) {
    pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
    pcl::PrincipalCurvaturesEstimation<pcl::PointXYZ, pcl::Normal, pcl::PrincipalCurvatures> pc;
    pc.setInputCloud(cloud1);
    pc.setSearchSurface(cloud2);
    ne.setInputCloud(cloud1);
    ne.setSearchSurface(cloud2);

    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>());
    ne.setSearchMethod(tree);
    pc.setSearchMethod(tree);

    pcl::PointCloud<pcl::Normal>::Ptr cloud_normals(new pcl::PointCloud<pcl::Normal>);
    pcl::PointCloud<pcl::PrincipalCurvatures>::Ptr cloud_curvatures(new pcl::PointCloud<pcl::PrincipalCurvatures>);
    ne.setKSearch(5);
    pc.setKSearch(5);

    ne.compute(*cloud_normals);
    pc.setInputNormals(cloud_normals);
    pc.compute(*cloud_curvatures);
    cout << "normals" << cloud_normals->size() << endl;
    cout << "cloud_curvatures" << cloud_curvatures->size() << endl;
}

void DBSCAN::start_scan() {
    start = clock();
    select_kernel();
    find_independent();
    end = clock();
    double endtime = (double)(end - start) / CLOCKS_PER_SEC;
    cout << "Total time: " << endtime << "s" << endl;
}

void DBSCAN::select_kernel() {
    pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
    float resolution = 0.0001f;
    pcl::octree::OctreePointCloudSearch<pcl::PointXYZ> octree(resolution);
    if (method_ == KD_TREE) {
        kdtree.setInputCloud(cloud_);
    } else if (method_ == OCT_TREE) {
        octree.setInputCloud(cloud_);
        octree.addPointsFromInputCloud();
    }

    for (auto i = 0; i < cloud_->points.size(); i++) {
        if (method_ == KD_TREE) {
            kdtree.radiusSearch(cloud_->points[i], eps, neighbourPoints[i], neighbourDistance[i]);
        } else if ((method_ == OCT_TREE)) {
            octree.radiusSearch(cloud_->points[i], eps, neighbourPoints[i], neighbourDistance[i]);
        }
        if (neighbourPoints[i].size() >= MinPts) {
            cluster_type.push_back(CORE_POINT);
            core_points.push_back(i);
        } else if (neighbourPoints[i].size() > MinbPts) {
            cluster_type.push_back(BOUND_POINT);
            bound_points.push_back(i);
        } else {
            cluster_type.push_back(NOISE_POINT);
        }
    }
}

std::vector<int> DBSCAN::vectors_intersection(std::vector<int> v1, std::vector<int> v2) {
    std::vector<int> v;
    std::sort(v1.begin(), v1.end());
    std::sort(v2.begin(), v2.end());
    std::set_intersection(v1.begin(), v1.end(), v2.begin(), v2.end(), std::back_inserter(v));
    return v;
}

std::vector<int> DBSCAN::vectors_set_union(std::vector<int> v1, std::vector<int> v2) {
    std::vector<int> v;
    std::sort(v1.begin(), v1.end());
    std::sort(v2.begin(), v2.end());
    std::set_union(v1.begin(), v1.end(), v2.begin(), v2.end(), std::back_inserter(v));
    return v;
}

std::vector<int> DBSCAN::vectors_set_diff(std::vector<int> v1, std::vector<int> v2) {
    std::vector<int> v;
    std::sort(v1.begin(), v1.end());
    std::sort(v2.begin(), v2.end());
    std::set_difference(v1.begin(), v1.end(), v2.begin(), v2.end(), std::back_inserter(v));
    return v;
}

void DBSCAN::find_independent() {
    cout << "start find intersection of cluster and union them" << endl;
    for (int i = 0; i < core_points.size(); i++) {
        if (neighbourPoints[core_points[i]].size() == 0) continue;
        for (int j = 0; j < core_points.size(); j++) {
            if (j == i || neighbourPoints[core_points[j]].size() == 0) continue;
            std::vector<int> result;
            result = vectors_intersection(neighbourPoints[core_points[i]], neighbourPoints[core_points[j]]);
            if (result.size() > 20) {
                if (neighbourPoints[core_points[i]].size() < 500 &&
                    neighbourPoints[core_points[j]].size() < 500 &&
                    (neighbourPoints[core_points[i]].size() + neighbourPoints[core_points[j]].size()) < 600) {
                    neighbourPoints[core_points[i]] = vectors_set_union(neighbourPoints[core_points[i]], neighbourPoints[core_points[j]]);
                    neighbourPoints[core_points[j]].clear();
                }
            }
        }
    }

    for (int i = 0; i < bound_points.size() && use_edge; i++) {
        int max_intersect = 0;
        int max_index = -1;
        pcl::PointCloud<pcl::PointXYZ>::Ptr bound(new pcl::PointCloud<pcl::PointXYZ>);
        for (auto j = 0; j < neighbourPoints[bound_points[i]].size(); j++) {
            pcl::PointXYZ point;
            point.x = cloud_->points[neighbourPoints[bound_points[i]][j]].x;
            point.y = cloud_->points[neighbourPoints[bound_points[i]][j]].y;
            point.z = cloud_->points[neighbourPoints[bound_points[i]][j]].z;
            bound->points.push_back(point);
        }
        for (int j = 0; j < core_points.size(); j++) {
            if (neighbourPoints[core_points[j]].size() == 0) continue;
            std::vector<int> result;
            result = vectors_intersection(neighbourPoints[bound_points[i]], neighbourPoints[core_points[j]]);
            if (result.size() > max_intersect) {
                max_intersect = result.size();
                max_index = core_points[j];
            }
        }
        if (max_index != -1) {
            neighbourPoints[max_index] = vectors_set_union(neighbourPoints[max_index], neighbourPoints[bound_points[i]]);
        }
    }

    for (auto i = 0; i < core_points.size(); i++) {
        if (neighbourPoints[core_points[i]].size() == 0 || neighbourPoints[core_points[i]].size() < 200)
            continue;
        for (int j = 0; j < core_points.size(); j++) {
            if (j == i || neighbourPoints[core_points[j]].size() == 0) continue;
            std::vector<int> result;
            result = vectors_intersection(neighbourPoints[core_points[i]], neighbourPoints[core_points[j]]);
            if (result.size() > 200) {
                if (neighbourPoints[core_points[i]].size() > neighbourPoints[core_points[j]].size()) {
                    neighbourPoints[core_points[i]] = vectors_set_diff(neighbourPoints[core_points[i]], neighbourPoints[core_points[j]]);
                }
            }
        }
    }

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr result_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    cluster_center = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>);
    srand((int)time(0));

    for (auto i = 0; i < core_points.size(); i++) {
        if (neighbourPoints[core_points[i]].size() == 0 || neighbourPoints[core_points[i]].size() < 200)
            continue;
        int G = rand() % 255;
        int B = rand() % 255;
        int R = rand() % 255;

        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cluster(new pcl::PointCloud<pcl::PointXYZ>);

        for (auto j = 0; j < neighbourPoints[core_points[i]].size(); j++) {
            pcl::PointXYZRGB point;
            point.r = G;
            point.g = R;
            point.b = B;
            point.x = cloud_->points[neighbourPoints[core_points[i]][j]].x;
            point.y = cloud_->points[neighbourPoints[core_points[i]][j]].y;
            point.z = cloud_->points[neighbourPoints[core_points[i]][j]].z;
            result_cloud->points.push_back(point);
            cloud_cluster->points.push_back(cloud_->points[neighbourPoints[core_points[i]][j]]);
        }
        cloud_cluster->width = cloud_cluster->points.size();
        cloud_cluster->height = 1;
        cloud_cluster->is_dense = true;
        sum_points += cloud_cluster->points.size();
        points_num.push_back(cloud_cluster->points.size());
        Eigen::Vector4f centroid;
        pcl::compute3DCentroid(*cloud_cluster, centroid);
        cluster_center->points.push_back(pcl::PointXYZ(centroid[0], centroid[1], centroid[2]));
        cluster_centroid.push_back(centroid);
        result_cloud_.push_back(cloud_cluster);
    }
    cluster_center->width = cluster_center->points.size();
    cluster_center->height = 1;
    cluster_center->is_dense = true;
    cout << "the sum of clusters: " << result_cloud_.size() << endl;
    result_cloud->width = result_cloud->points.size();
    result_cloud->height = 1;
    result_cloud->is_dense = true;

    end = clock();
    double endtime = (double)(end - start) / CLOCKS_PER_SEC;
    cout << "Total time: " << endtime << "s" << endl;
}

}  // namespace segmenter
}  // namespace autosense
