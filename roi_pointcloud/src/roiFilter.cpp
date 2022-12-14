#include <Eigen/Core>
#include <Eigen/Dense>
#include <algorithm>
#include <pcl/filters/voxel_grid.h>
#include <pcl_conversions/pcl_conversions.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>

struct VelodynePointXYZIR {
  PCL_ADD_POINT4D
  PCL_ADD_INTENSITY;
  uint16_t ring;
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
} EIGEN_ALIGN16;
POINT_CLOUD_REGISTER_POINT_STRUCT(
    VelodynePointXYZIR,
    (float, x, x)(float, y, y)(float, z, z)(float, intensity,
                                            intensity)(uint16_t, ring, ring))

using PointXYZIR = VelodynePointXYZIR;
using namespace std;
typedef pcl::PointXYZ PointType;

bool cmp(int lhs, int rhs) //降序
{
  return lhs > rhs;
}
class Grid {
public:
  pcl::PointCloud<pcl::PointXYZ>::Ptr grid_cloud;
  vector<int> point_id;
  Grid() { grid_cloud.reset(new pcl::PointCloud<PointType>()); }
};

class ROIFilter {
private:
  ros::NodeHandle nh;
  ros::Subscriber subLidarCloud;
  ros::Publisher pubFliterCloud;
  vector<Grid> grid;
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
    grid.reserve(length * width * height);
    for (size_t i = 0; i < length * width * height; i++) {
      Grid tmpGrid = Grid{};
      grid.push_back(tmpGrid);
    }
  };

  ~ROIFilter(){};
  void cloudHandler(const sensor_msgs::PointCloud2ConstPtr &laserCloudMsg) {
    resetParam();
    currentCloudMsg = *laserCloudMsg;
    pcl::moveFromROSMsg(currentCloudMsg, *laserCloudIn);
    voxelizeCloud();
  }

  void resetParam() {
    for (size_t i = 0; i < length * width * height; i++) {
      grid[i].grid_cloud->clear();
      grid[i].point_id.clear();
    }
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
        int grid_idx = h_idx * (length * width) + l_idx * width + w_idx;
        // cout << grid_idx << endl;
        grid[grid_idx].grid_cloud->points.push_back(tmpPoint);
        grid[grid_idx].point_id.push_back(i);
      }
    }
  }

  void featureExtraction(pcl::PointCloud<PointType>::Ptr grid_cloud) {
    int num = grid_cloud->points.size();
    if (num < 5)
      return;
    PointType CenterPoint;
    for (int i = 0; i < num; i++) {
      CenterPoint.x += grid_cloud->points[i].x;
      CenterPoint.y += grid_cloud->points[i].y;
      CenterPoint.z += grid_cloud->points[i].z;
    }
    CenterPoint.x /= num;
    CenterPoint.y /= num;
    CenterPoint.z /= num;

    Eigen::MatrixXd A;
    Eigen::MatrixXd P;
    Eigen::MatrixXd P_t;
    P.resize(3, num);

    for (int i = 0; i < num; i++) {
      PointType thisPoint;
      thisPoint.x -= CenterPoint.x;
      thisPoint.y -= CenterPoint.y;
      thisPoint.z -= CenterPoint.z;
      P(0, i) = thisPoint.x;
      P(1, i) = thisPoint.y;
      P(2, i) = thisPoint.z;
    }
    P_t = P.transpose();
    A = P * P_t;
    A = A / num;
    Eigen::EigenSolver<Eigen::MatrixXd> es(A);
    Eigen::Vector3d ev = es.eigenvalues().real();
    double lambda[] = {ev(0), ev(1), ev(3)};
    sort(lambda, lambda + 3, cmp);
  }
};

int main(int argc, char **argv) {
  ros::init(argc, argv, "roiFilter");
  ROIFilter RF;
  ros::spin();
  return 0;
}
