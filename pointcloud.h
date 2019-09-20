#ifndef POINTCLOUD_H
#define POINTCLOUD_H

#include <QObject>
#include <QTime>

#include <vector>
#include <stdlib.h>
#include <math.h>

#include <Eigen/Core>
#include <Eigen/SVD>
#include <Eigen/Dense>
#include <Eigen/Geometry>

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/io/ply_io.h>
#include <pcl/io/pcd_io.h>

//filters
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/approximate_voxel_grid.h>   //滤波类头文件  （使用体素网格过滤器处理的效果比较好）
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/uniform_sampling.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/extract_indices.h>//提取滤波器
#include <pcl/filters/project_inliers.h>//投影滤波类头文件

#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>

#include <pcl/search/kdtree.h>

#include <pcl/common/centroid.h>

typedef pcl::PointXYZ PointT;
typedef pcl::PointCloud<PointT> PointCloud;

class pointCloud : public QObject
{
    Q_OBJECT
public:
    explicit pointCloud(QObject *parent = 0);

signals:

public slots:

public:
    //计算分辨率
    float computeResolution(PointCloud::Ptr cloud);
    //去质心
    void removeCentroidPoint(PointCloud::Ptr pointCloud_, Eigen::Vector4f& mean);
    //体素滤波
    int VoxelGrid_Filter(PointCloud::Ptr input,
        PointCloud::Ptr output,
        float leafsize = 1.0);
    //基于统计学滤波
    int StatisticalOutlierRemoval_Filter(PointCloud::Ptr input,
        PointCloud::Ptr output,
        int K = 30,
        float stddevMulThresh = 1.0);
    //提取平面，计算平面法向量
    void SACSegmentation_plane(PointCloud::Ptr input,
                               pcl::ModelCoefficients coefficients,
                               pcl::PointIndices::Ptr inliers,
                               int maxIteration = 100,
                               float distanceThreshold = 1.0
                               );
    //求两向量旋转矩阵
    Eigen::Matrix3f computeRotation(Eigen::Vector3f &a, Eigen::Vector3f &b);

private:
    PointCloud::Ptr pointCloudPtr;



};

#endif // POINTCLOUD_H
