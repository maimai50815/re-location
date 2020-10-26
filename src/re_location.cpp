#include "re_location.h"

using namespace std;
using namespace cv;

namespace re_location
{
Relocation::Relocation(ros::NodeHandle& nh)
{
  map_file_name_ = nh.param<std::string>("map_file_name", "~");
  beams_num_ = nh.param<int>("beams_num", 1440);
  beams_angle_ = nh.param<float>("beams_angle", 360.0);
  grid_size_ = nh.param<float>("grid_size", 0.5);
  grid_orients_ = nh.param<int>("grid_orients", 16);

  map_resolution_ = nh.param<float>("map_resolution", 0.05);
  gather_step_ = nh.param<float>("gather_step", 0.2);

  initial_pose_sub_ = nh.subscribe<geometry_msgs::PoseWithCovarianceStamped>("/initialpose", 1, &Relocation::initialPoseCb, this);
  pose_sub_ = nh.subscribe<geometry_msgs::PoseStamped>("/move_base_simple/goal", 1, &Relocation::poseCb, this);

  /* DEBUG */
  map_cloud_pub_ = nh.advertise<sensor_msgs::PointCloud2>("/map_cloud", 1, true);
  laser_pub_ = nh.advertise<sensor_msgs::PointCloud2>("/fake_laser", 1, true);

  map_cloud_.reset(new NormalCloud());
  laser_cloud_.reset(new NormalCloud());

  readMapFile();
}

void Relocation::readMapFile()
{
  cv::Mat thisMat = cv::imread(map_file_name_);
  cv::cvtColor(thisMat, map_image_, CV_BGR2GRAY);

  cout<<"map image type:"<<map_image_.type()<<endl;

  map_sizex_ = map_image_.rows;
  map_sizey_ = map_image_.cols;

  map_cloud_->clear();

  NormalType p;

  for(int u = 0; u < map_sizex_; ++u)
  {
    for(int v = 0; v < map_sizey_; ++v)
    {
      if((int)map_image_.at<uchar>(v,u) < 10)
      {
        mapToWorld(u, v, p.x, p.y, map_resolution_);

        map_cloud_->points.push_back(p);
      }
    }
  }

  publishCloud(map_cloud_, map_cloud_pub_, "rslidar");

  //对整个地图进行栅格化，并对每个栅格做假激光
  registFeatures();
}

void Relocation::searchLocation()
{

}

void Relocation::registFeatures()
{
  cout<<"begin registering features"<<endl;
  //在获得一幅地图后，我们对每一个栅格中心打出的点云，获取特征点

  float mapWidth = map_sizex_*map_resolution_;
  float mapHeight = map_sizey_*map_resolution_;

  int xNum = (int)(mapWidth/gather_step_);
  int yNum = (int)(mapHeight/gather_step_);

  for(int i = 0; i < xNum; ++i)
  {
    for(int j = 0; j < yNum; ++j)
    {
      float wx = float(i)*gather_step_;
      float wy = float(j)*gather_step_;
      getFakeLaser(wx, wy);

      feature_maker_->setFeatureFromCloud(wx, wy, laser_cloud_);

    }
  }
}

//用假激光来初始化整个地图，也用来模拟测试
void Relocation::getFakeLaser(float& wx, float& wy)
{
  int mx, my;
  worldToMap(mx, my, wx, wy, map_resolution_);

  cout<<"get fake laser, world:"<<wx<<","<<wy<<"; map:"<<mx<<","<<my<<endl;

  //在图像中一步的探索长度为2个像素
  float retraceStep = 2.0;
  //相邻波束之间的角度间距
  float beamStep = (beams_angle_*M_PI/180.0)/float(beams_num_);

  //float firstBeamYaw = current_yaw_-0.5*beams_num_*beamStep;
  float firstBeamYaw = 0.0;

  laser_cloud_->clear();

  //对所有波束进行依次回溯，得到模拟的一圈激光点
  for(int i = 0; i < beams_num_; ++i)
  {
    NormalType tp;

    float thisYaw = firstBeamYaw + float(i)*beamStep;
    float crossX = 1.0*cos(thisYaw);
    float crossY = 1.0*sin(thisYaw);

    bool hitBoundary = false;
    int stepCount = 0;

    float fu = (float)mx, fv = (float)my;

    float stepX = (retraceStep * crossX);
    float stepY = (retraceStep * crossY);
    //cout<<"step:"<<stepX<<","<<stepY<<endl;

    while(!hitBoundary)
    {
      //如果该波束未碰到边缘
      int u = (int)fu;
      int v = (int)fv;
      if((int)map_image_.at<uchar>(v, u) > 10)
      {
        fu += stepX;
        fv += stepY;
      }
      else
      {
        hitBoundary = true;
      }

      if(u<0 || u>map_sizex_ || v<0 || v>map_sizey_)
        break;

      if(stepCount > 3000)
        break;

      ++stepCount;
    }

    mapToWorld((int)fu, (int)fv, tp.x, tp.y, map_resolution_);

    if(hitBoundary)
    {
      //cout<<"tp:"<<tp.x<<","<<tp.y<<endl;
      laser_cloud_->points.push_back(tp);
    }
  }

  //cout<<"laser_cloud_:"<<laser_cloud_->size()<<endl;

  publishCloud(laser_cloud_, laser_pub_, "rslidar");
}

void Relocation::initialPoseCb(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg)
{
  current_x_ = msg->pose.pose.position.x;
  current_y_ = msg->pose.pose.position.y;

  // float roll, pitch, yaw;
  // tf::Quaternion orientation;
  // tf::quaternionMsgToTF(msg->pose.pose.orientation, orientation);
  // tf::Matrix3x3(orientation).getRPY(roll, pitch, yaw);
  // current_yaw_ = yaw;

  getFakeLaser(current_x_, current_y_);
}

void Relocation::poseCb(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
  cout<<"begin making relocation"<<endl;

  searchLocation();
}


}/* end of namespace */

int main(int argc, char** argv)
{
  ros::init(argc, argv, "re_location");
  ros::NodeHandle nh("~");
  re_location::Relocation loc(nh);
  ros::spin();
  return 0;
}
