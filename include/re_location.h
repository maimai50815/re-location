#ifndef RE_LOCATION_H
#define RE_LOCATION_H

#include "get_feature.h"

using namespace std;

namespace re_location
{
class Relocation
{
public:
  Relocation(ros::NodeHandle&);

  //做假激光对地图进行探索
  void getFakeLaser(float& wx, float& wy);

  void registFeatures();
  void searchLocation();
private:
  /* the real pose of a robot */
  void initialPoseCb(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg);

  /* the current wrong and initial pose in world frame */
  void poseCb(const geometry_msgs::PoseStamped::ConstPtr& msg);

  void readMapFile();

  std::string map_file_name_;
  cv::Mat map_image_;

  ros::Subscriber initial_pose_sub_, pose_sub_, map_sub_;
  ros::Publisher map_cloud_pub_, laser_pub_;

  float current_x_, current_y_;
  float current_yaw_;

  /* the initial yaw(not the real pose) */
  float init_yaw_;

  /* the laser scan beams params */
  int beams_num_;
  float beams_angle_;

  /* register feature */
  /* we have to devide the map into some grids, in each grid there are several orients for making laser features */
  float grid_size_;
  int grid_orients_;
  /* make fake laser */
  float retrace_hit_thresh_;

  /* map info */
  float map_resolution_;
  int map_sizex_;
  int map_sizey_;
  float map_origin_x_, map_origin_y_;
  std::vector<int> map_info_;

  NormalPtr map_cloud_, laser_cloud_;
  pcl::KdTreeFLANN<NormalType> kdtree_;

  float gather_step_;

  std::shared_ptr<GetFeature> feature_maker_;
};

}/* end of namespace */
#endif
