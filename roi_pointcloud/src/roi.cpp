#include <Eigen/Core>
#include <Eigen/Dense>
#include <algorithm>
#include <functional>
#include <pcl/filters/voxel_grid.h>
#include <pcl_conversions/pcl_conversions.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>

using namespace std;
typedef pcl::PointXYZ PointType;

bool cmp(int lhs, int rhs) //降序
{
  return lhs > rhs;
}
class Grid {
public:
  pcl::PointCloud<pcl::PointXYZ>::Ptr grid_cloud;
  pcl::PointIndices::Ptr grid_inliners;
  Grid() {
    grid_cloud.reset(new pcl::PointCloud<PointType>());
    grid_inliners.reset(new pcl::PointIndices());
  }
};

class ROIFilter {
private:
  ros::NodeHandle nh;
  ros::Subscriber subLidarCloud;
  ros::Publisher pubFliterCloud;
  Grid ***grid;
  int width;
  int length;
  int height;
  float gridHeight;
  float gridWidth;
  float gridLength;
  string cloudTopic;
  sensor_msgs::PointCloud2 currentCloudMsg;
  pcl::PointCloud<PointType>::Ptr laserCloudIn;

public:
  ROIFilter() {
    nh.param<string>("roiFilter/cloudTopic", cloudTopic, "lidar_points");
    nh.param<int>("roiFilter/length", length, 100);
    nh.param<int>("roiFilter/width", width, 100);
    nh.param<int>("roiFilter/height", height, 50);
    nh.param<float>("roiFilter/gridLength", gridLength, 0.5);
    nh.param<float>("roiFilter/gridWidth", gridWidth, 0.5);
    nh.param<float>("roiFilter/gridHeight", gridHeight, 0.5);
    subLidarCloud = nh.subscribe<sensor_msgs::PointCloud2>(
        cloudTopic, 5, &ROIFilter::cloudHandler, this);
    pubFliterCloud =
        nh.advertise<sensor_msgs::PointCloud2>("filtered_cloud", 5);
    laserCloudIn.reset(new pcl::PointCloud<PointType>());

    // for (size_t i = 0; i < length * width * height; i++) {
    //   Grid *tmpGrid = new Grid();
    //   grid.push_back(*tmpGrid);
    // }
    //想要在这里初始化一个多位数字grid[length][width][height],但是不知道怎么做？
  };

  ~ROIFilter(){};
  void cloudHandler(const sensor_msgs::PointCloud2ConstPtr &laserCloudMsg) {
    resetParam();
    currentCloudMsg = *laserCloudMsg;
    pcl::moveFromROSMsg(currentCloudMsg, *laserCloudIn);
    voxelizeCloud();
  }

  void resetParam() {
    // for (size_t i = 0; i < length * width * height; i++) {
    //   grid[i].grid_cloud.reset();
    //   grid[i].grid_inliners.reset();
    // }
  }

  void voxelizeCloud() {
    if (laserCloudIn->empty())
      return;
    for (int i = 0; i < laserCloudIn->size(); i++) {
      PointType tmpPoint;
      tmpPoint = laserCloudIn->points[i];
      int l_idx = round((tmpPoint.x + (length / 2.0)) / gridLength);
      int w_idx = round((tmpPoint.y + (width / 2.0)) / gridWidth);
      int h_idx = round((tmpPoint.z + 1.2) / gridHeight);
      if (l_idx < 0 || l_idx >= length || w_idx < 0 || w_idx >= width ||
          h_idx < 0 || h_idx >= height) {
        continue;
      } else {
        grid[l_idx][w_idx][h_idx].grid_cloud->push_back(tmpPoint);
        grid[l_idx][w_idx][h_idx].grid_inliners->indices.push_back(i);
        grid[l_idx][w_idx][h_idx].grid_inliners->header = laserCloudIn->header;
      }
    }
  }
};

int main(int argc, char **argv) {
  ros::init(argc, argv, "roiFilter");
  ROIFilter RF;
  ros::spin();
  return 0;
}
