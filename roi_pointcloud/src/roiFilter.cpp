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

class Grid {
  pcl::PointCloud<PointXYZIR>::Ptr grid_cloud{
      new pcl::PointCloud<PointXYZIR>()};
  pcl::PointIndices::Ptr grid_inliners{new pcl::PointIndices()};
};

class ROIFilter {
private:
  ros::NodeHandle nh;
  ros::Subscriber subLidarCloud;
  ros::Publisher pubFliterCloud;

  int width;
  int length;
  int height;
  float gridHeight;
  float gridWidth;
  string cloudTopic;
  sensor_msgs::PointCloud2 currentCloudMsg;

  pcl::PointCloud<PointXYZIR>::Ptr laserCloudIn;

public:
  ROIFilter() {
    nh.param<string>("roiFilter/cloudTopic", cloudTopic, "lidar_points");
    nh.param<int>("roiFilter/width", width, 100);
    nh.param<int>("roiFilter/length", length, 100);
    nh.param<int>("roiFilter/height", height, 20);
    nh.param<float>("roiFilter/gridHeight", gridHeight, 0.3);
    nh.param<float>("roiFilter/gridWidth", gridWidth, 0.5);
    Grid grid[width][length][height];
    subLidarCloud = nh.subscribe<sensor_msgs::PointCloud2>(
        cloudTopic, 5, &ROIFilter::cloudHandler, this);
    pubFliterCloud =
        nh.advertise<sensor_msgs::PointCloud2>("filtered_cloud", 5);
    laserCloudIn.reset(new pcl::PointCloud<PointXYZIR>());
  };

  ~ROIFilter(){};
  void cloudHandler(const sensor_msgs::PointCloud2ConstPtr &laserCloudMsg) {
    currentCloudMsg = *laserCloudMsg;
    pcl::moveFromROSMsg(currentCloudMsg, *laserCloudIn);
  }

  void voxelizeCloud() {
    if (laserCloudIn->empty())
      return;
    for (int i = 0; i < laserCloudIn->size(); i++) {
      PointXYZIR tmpPoint;
      tmpPoint = laserCloudIn->points[i];
    }
  }
};

int main(int argc, char **argv) {
  ros::init(argc, argv, "roiFilter");
  ROIFilter RF;
  ros::spin();
  return 0;
}
