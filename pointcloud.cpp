#include "pointcloud.h"
#include "math.h"

pointCloud::pointCloud(QObject *parent) : QObject(parent)
{

}

float pointCloud::computeResolution(PointCloud::Ptr cloud)
{

    float resolution = 0.0;
    int n_points = 0;
    int nres;
    std::vector<int> indices(2);
    std::vector<float> sqr_distances(2);

    pcl::search::KdTree<PointT> tree;
    tree.setInputCloud(cloud);

    for (size_t i = 0; i < cloud->size(); ++i)
    {
        if (!pcl_isfinite((*cloud)[i].x))
        {
            continue;
        }
        nres = tree.nearestKSearch(i, 2, indices, sqr_distances);
        if (nres == 2)
        {
            resolution += sqrt(sqr_distances[1]);
            ++n_points;
        }
    }
    if (n_points != 0)
    {
        resolution /= n_points;
    }

    return resolution;
}

void pointCloud::removeCentroidPoint(PointCloud::Ptr pointCloud_, Eigen::Vector4f &mean)
{
    if(pointCloud_ == NULL)
        return;

    if(pointCloud_->points.empty())
        return;

    int npti = pointCloud_->size();
    for(size_t i = 0; i < npti; ++i)
    {
        (*pointCloud_)[i].x -= mean(0);
        (*pointCloud_)[i].y -= mean(1);
        (*pointCloud_)[i].z -= mean(2);
    }

}

int pointCloud::VoxelGrid_Filter(PointCloud::Ptr input, PointCloud::Ptr output, float leafsize)
{
    int num = input->size();

    pcl::VoxelGrid<PointT> voxelgrid_filter;
    voxelgrid_filter.setLeafSize(leafsize, leafsize, leafsize);
    voxelgrid_filter.setInputCloud(input);
    voxelgrid_filter.filter(*output);

    std::cout << "VoxelGrid_Filter, Input points: " << num
              << "; Output points: " << output->size() << std::endl;

    return output->size();
}

int pointCloud::StatisticalOutlierRemoval_Filter(PointCloud::Ptr input, PointCloud::Ptr output, int K, float stddevMulThresh)
{
    int num = input->size();

    pcl::StatisticalOutlierRemoval<PointT> statistical_filter;
    statistical_filter.setMeanK(K);
    statistical_filter.setStddevMulThresh(stddevMulThresh);
    statistical_filter.setInputCloud(input);
    statistical_filter.filter(*output);

    std::cout << "StatisticalOutlierRemoval_Filter, Input points: " << num
        << "; Output points: " << output->size() << std::endl;

    return output->size();
}

void pointCloud::SACSegmentation_plane(PointCloud::Ptr input, pcl::ModelCoefficients coefficients, pcl::PointIndices::Ptr inliers, int maxIteration, float distanceThreshold)
{
    pcl::SACSegmentation<PointT> seg;
    seg.setOptimizeCoefficients(true);
    seg.setModelType(pcl::SACMODEL_PLANE);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setMaxIterations(maxIteration);
    seg.setDistanceThreshold(distanceThreshold);

    seg.setInputCloud(input);
    seg.segment(*inliers, coefficients);
}

Eigen::Matrix3f pointCloud::computeRotation(Eigen::Vector3f &a, Eigen::Vector3f &b)
{
    Eigen::Matrix3f rotation = Eigen::Matrix3f::Identity();

    float theta =  acos(a.dot(b)/(a.norm() * b.norm()));

    Eigen::Vector3f rotationAxis;
    rotationAxis(0) = a(2) * b(3) - a(3) * b(2);
    rotationAxis(1) = a(3) * b(1) - a(1) * b(3);
    rotationAxis(2) = a(1) * b(2) - a(2) * b(1);

    rotation(0,0) = cos(theta) + pow(rotationAxis(0),2) * (1 - cos(theta));
    rotation(0,1) = rotationAxis(0)*rotationAxis(1)*(1-cos(theta))-rotationAxis(2)*sin(theta);
    rotation(0,2) = rotationAxis(1)*sin(theta) + rotationAxis(0)*rotationAxis(2)*(1-cos(theta));
    rotation(1,0) = rotationAxis(0)*rotationAxis(1)*(1-cos(theta))+rotationAxis(2)*sin(theta);
    rotation(1,1) = cos(theta) + pow(rotationAxis(1),2) * (1 - cos(theta));
    rotation(1,2) = rotationAxis(1)*rotationAxis(2)*(1-cos(theta))-rotationAxis(0)*sin(theta);
    rotation(2,0) = -rotationAxis(1)*sin(theta) + rotationAxis(0)*rotationAxis(2)*(1-cos(theta));
    rotation(2,1) = rotationAxis(1)*rotationAxis(2)*(1-cos(theta))+rotationAxis(0)*sin(theta);
    rotation(2,2) = cos(theta) + pow(rotationAxis(2),2) * (1 - cos(theta));

    return rotation;
}



























