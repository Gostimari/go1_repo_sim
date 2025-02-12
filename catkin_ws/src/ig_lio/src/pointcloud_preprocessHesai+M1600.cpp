#include "ig_lio/pointcloud_preprocess.h"
#include "ig_lio/timer.h"

extern Timer timer;

void PointCloudPreprocess::Process(
    const livox_ros_driver::CustomMsg::ConstPtr& msg,
    pcl::PointCloud<PointType>::Ptr& cloud_out,
    const double last_start_time) {
  double time_offset =
      (msg->header.stamp.toSec() - last_start_time) * 1000.0;  // ms

  for (size_t i = 1; i < msg->point_num; ++i) {
    if ((msg->points[i].line < num_scans_) &&
        ((msg->points[i].tag & 0x30) == 0x10 ||
         (msg->points[i].tag & 0x30) == 0x00) &&
        !HasInf(msg->points[i]) && !HasNan(msg->points[i]) &&
        !IsNear(msg->points[i], msg->points[i - 1]) &&
        (i % config_.point_filter_num == 0)) {
      PointType point;
      point.normal_x = 0;
      point.normal_y = 0;
      point.normal_z = 0;
      point.x = msg->points[i].x;
      point.y = msg->points[i].y;
      point.z = msg->points[i].z;
      point.intensity = msg->points[i].reflectivity;
      point.curvature = time_offset + msg->points[i].offset_time * 1e-6;  // ms
      cloud_out->push_back(point);
    }
  }
}

void PointCloudPreprocess::Process(
    const sensor_msgs::PointCloud2::ConstPtr& msg,
    pcl::PointCloud<PointType>::Ptr& cloud_out) {
  switch (config_.lidar_type) {
  case LidarType::VELODYNE:
    ProcessVelodyne(msg, cloud_out);
    break;
  case LidarType::OUSTER:
    ProcessOuster(msg, cloud_out);
    break;
  case LidarType::HESAI:
    ProcessHesai(msg, cloud_out);
    break;
  default:
    LOG(INFO) << "Error LiDAR Type!!!" << std::endl;
    exit(0);
  }
}

void PointCloudPreprocess::ProcessVelodyne(
    const sensor_msgs::PointCloud2::ConstPtr& msg,
    pcl::PointCloud<PointType>::Ptr& cloud_out) {
  pcl::PointCloud<VelodynePointXYZIRT> cloud_origin;
  pcl::fromROSMsg(*msg, cloud_origin);

  for (size_t i = 0; i < cloud_origin.size(); ++i) {
    if ((i % config_.point_filter_num == 0) && !HasInf(cloud_origin.at(i)) &&
        !HasNan(cloud_origin.at(i))) {
      PointType point;
      point.normal_x = 0;
      point.normal_y = 0;
      point.normal_z = 0;
      point.x = cloud_origin.at(i).x;
      point.y = cloud_origin.at(i).y;
      point.z = cloud_origin.at(i).z;
      point.intensity = cloud_origin.at(i).intensity;
      // Use the timestampSec field to calculate curvature or other properties
      double timestampSec = cloud_origin.at(i).timestampSec;
      // Example: Set curvature based on timestamp
      // Adjust this calculation as per your requirement
      point.curvature = timestampSec * 1e-6; // Assuming timestamp is in microseconds
      cloud_out->push_back(point);
    }
  }
}
void PointCloudPreprocess::ProcessHesai(
    const sensor_msgs::PointCloud2::ConstPtr& msg,
    pcl::PointCloud<PointType>::Ptr& cloud_out) {
  pcl::PointCloud<HesaiPointXYZIRT> cloud_origin;
  pcl::fromROSMsg(*msg, cloud_origin);

  for (size_t i = 0; i < cloud_origin.size(); ++i) {
    if ((i % config_.point_filter_num == 0) && !HasInf(cloud_origin.at(i)) &&
        !HasNan(cloud_origin.at(i))) {
      PointType point;
      point.normal_x = 0;
      point.normal_y = 0;
      point.normal_z = 0;
      point.x = cloud_origin.at(i).x;
      point.y = cloud_origin.at(i).y;
      point.z = cloud_origin.at(i).z;
      point.intensity = cloud_origin.at(i).intensity;
      // Assuming curvature information is not available in Hesai LiDAR data.
      // You need to adjust this line according to the actual data structure.
      point.curvature = 0.0; // Adjust this according to the actual curvature information in Hesai LiDAR data
      cloud_out->push_back(point);
    }
  }
}


void PointCloudPreprocess::ProcessOuster(
    const sensor_msgs::PointCloud2::ConstPtr& msg,
    pcl::PointCloud<PointType>::Ptr& cloud_out) {
  pcl::PointCloud<OusterPointXYZIRT> cloud_origin;
  pcl::fromROSMsg(*msg, cloud_origin);

  for (size_t i = 0; i < cloud_origin.size(); ++i) {
    if ((i % config_.point_filter_num == 0) && !HasInf(cloud_origin.at(i)) &&
        !HasNan(cloud_origin.at(i))) {
      PointType point;
      point.normal_x = 0;
      point.normal_y = 0;
      point.normal_z = 0;
      point.x = cloud_origin.at(i).x;
      point.y = cloud_origin.at(i).y;
      point.z = cloud_origin.at(i).z;
      point.intensity = cloud_origin.at(i).intensity;
      // ms
      point.curvature = cloud_origin.at(i).t * 1e-6;
      cloud_out->push_back(point);
    }
  }
}

template <typename T>
inline bool PointCloudPreprocess::HasInf(const T& p) {
  return (std::isinf(p.x) || std::isinf(p.y) || std::isinf(p.z));
}

template <typename T>
inline bool PointCloudPreprocess::HasNan(const T& p) {
  return (std::isnan(p.x) || std::isnan(p.y) || std::isnan(p.z));
}

template <typename T>
inline bool PointCloudPreprocess::IsNear(const T& p1, const T& p2) {
  return ((abs(p1.x - p2.x) < 1e-7) || (abs(p1.y - p2.y) < 1e-7) ||
          (abs(p1.z - p2.z) < 1e-7));
}
