#ifndef COMMON_H
#define COMMON_H
#include <ros/ros.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/OccupancyGrid.h>
#include <tf/transform_datatypes.h>
#include <tf/tf.h>
#include <memory>
#include <unordered_map>
#include <queue>

#include <pcl/point_types.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/common/transforms.h>
#include <pcl/common/centroid.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl_conversions/pcl_conversions.h>

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

using namespace std;

typedef pcl::PointXYZI NormalType;
typedef pcl::PointCloud<NormalType> NormalCloud;
typedef pcl::PointCloud<NormalType>::Ptr NormalPtr;

/* we use the feature method like SIFT or ORB to describe the features of laser scan */
struct ScanFeature
{

};

struct PoseInfo
{
  float wx, wy;
  float score;

  bool operator< (const PoseInfo& other) const
  {
    return (score<other.score);
  }
};

static void worldToMap(int& mx, int& my, float& wx, float& wy, double resolution)
{
  mx = wx/resolution;
  my = wy/resolution;
}

static void mapToWorld(int mx, int my, float& wx, float& wy, double resolution)
{
  wx = mx*resolution;
  wy = my*resolution;
}

static void publishCloud(NormalPtr& pIn, ros::Publisher& pub, std::string frame_id)
{
  sensor_msgs::PointCloud2 cloud;
  pcl::toROSMsg(*pIn, cloud);
  cloud.header.frame_id = frame_id;
  pub.publish(cloud);
}
#endif
