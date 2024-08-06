#include <ros/ros.h>
#include <tf/transform_datatypes.h>
#include <std_msgs/Float32MultiArray.h>
#include <sensor_msgs/Imu.h>
#include <morai_msgs/GPSMessage.h>
#include <nav_msgs/Odometry.h>
#include <tracking_msg/TrackingObjectArray.h>
#include <geometry_msgs/Quaternion.h>
#include <GeographicLib/UTMUPS.hpp>
#include <Eigen/Dense>
#include <math.h>

class GPS2UTM {
public:
    GPS2UTM() : temp1(0.0), temp2(0.0), tempa(0.0), tempb(0.0), start_latitude(0.0), start_longitude(0.0),
                heading(0.0), delta_x(0.0), delta_y(0.0), Orientation()
    {
        ros::NodeHandle node;
        gps_sub = node.subscribe("/gps", 10, &GPS2UTM::gps_callback, this);
        imu_sub = node.subscribe("/imu", 10, &GPS2UTM::imu_callback, this);
        lidar_sub = node.subscribe("/lidar/tracking_objects", 10, &GPS2UTM::lidar_callback, this);
    }
   
    void gps_callback(const morai_msgs::GPSMessage::ConstPtr& _data) {
        start_longitude = _data->longitude;
        start_latitude = _data->latitude;
    }

    void imu_callback(const sensor_msgs::Imu::ConstPtr& _data) {
        Orientation = _data->orientation;
        heading = calculate_heading_from_imu();
    }

    void lidar_callback(const tracking_msg::TrackingObjectArray::ConstPtr& _data) {
        int total_obj_cnt = _data->array.size();

        // 상대적인 위치를 UTM 좌표계로 변환하고
        // 이를 빈 Eigen 행렬에 추가합니다
        Eigen::MatrixX2d pointcloud(total_obj_cnt, 2);

        for (int i = 0; i < total_obj_cnt; i++) {
            delta_x = _data->array[i].point.x;
            delta_y = _data->array[i].point.y;

            double end_utm_easting, end_utm_northing;
            calculate_longitude_latitude(end_utm_easting, end_utm_northing);

            pointcloud(i, 0) = end_utm_easting;
            pointcloud(i, 1) = end_utm_northing;
        }
    }
   
    double calculate_heading_from_imu() {
        // TODO: Update this method to match your Python implementation
        return 0.0; // Placeholder
    }
   
    void calculate_longitude_latitude(double& end_utm_easting, double& end_utm_northing) {
        int zone;
        bool northp;
        double gamma, k;

        // 시작 위치의 UTM 좌표 구하기
        GeographicLib::UTMUPS::Forward(start_latitude, start_longitude, zone, northp, end_utm_easting, end_utm_northing, gamma, k);

        // 헤딩값을 라디안 단위로 변환
        double heading_rad = heading * M_PI / 180.0;

        // 상대적인 x와 y 위치를 UTM 좌표계로 변환
        double delta_utm_easting = delta_x * cos(heading_rad) - delta_y * sin(heading_rad);
        double delta_utm_northing = delta_x * sin(heading_rad) + delta_y * cos(heading_rad);

        // 시작 위치에 UTM 좌표 변화량을 더하여 최종 UTM 좌표 구하기
        end_utm_easting += delta_utm_easting;
        end_utm_northing += delta_utm_northing;
    }

private:
    ros::Subscriber gps_sub;
    ros::Subscriber imu_sub;
    ros::Subscriber lidar_sub;

    double temp1;
    double temp2;
    double tempa;
    double tempb;
    double start_latitude;
    double start_longitude;
    double heading;
    double delta_x;
    double delta_y;
    geometry_msgs::Quaternion Orientation;
};


int main(int argc, char **argv) {
    ros::init(argc, argv, "gps2utm");
    GPS2UTM gps2utm;
    ros::spin();
    return 0;
}
