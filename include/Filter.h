#ifndef FILTER
#define FILTER
#include <iostream>
#include <vector>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/search/search.h>
#include <pcl/search/kdtree.h>
#include <pcl/surface/mls.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/features/normal_3d.h>
#include <pcl/features/integral_image_normal.h>
#include <pcl/segmentation/organized_multi_plane_segmentation.h>
#include <pcl/segmentation/planar_polygon_fusion.h>
#include <pcl/common/transforms.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/segmentation/plane_coefficient_comparator.h>
#include <pcl/segmentation/euclidean_plane_coefficient_comparator.h>
#include <pcl/segmentation/rgb_plane_coefficient_comparator.h>
#include <pcl/segmentation/edge_aware_plane_comparator.h>
#include <pcl/segmentation/euclidean_cluster_comparator.h>
#include <pcl/segmentation/organized_connected_component_segmentation.h>
#include <pcl/apps/dominant_plane_segmentation.h>
#include <pcl/common/angles.h>
#include <stdio.h>
#include <opencv2/core.hpp>
// this line for opencv2.4
// #include <opencv2/contrib/contrib.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>

//#include <opencv2/nonfree/nonfree.hpp>
#include "vfh.h"
typedef pcl::PointXYZRGBA PointT;
double FOCAL=525.0;
class Filter_Save
{
public:
Filter_Save ();
~Filter_Save ();
    struct Object
{
    std::vector<int> name_lists;
    double distances;
    Eigen::Vector4f  centroid;
    cv::Rect vertex;
    double WIDTH;
    double HEIGHT;
};
    std::string name;
    bool SAVEORNO;
    flann::Matrix<float> data;
    std::vector<std::string> lists;
    pcl::PointCloud<PointT>::Ptr  RemovePlane (const pcl::PointCloud<PointT>::Ptr input_cloud);
    cv::Rect vertex4(const pcl::PointCloud<PointT>::Ptr cloud_cluster);
    std::vector<Filter_Save ::Object>   Clusters(const pcl::PointCloud<PointT>::Ptr input_cloud);
    std::vector<Filter_Save ::Object>   LocatOnly(const pcl::PointCloud<PointT>::Ptr input_cloud);
   std::vector<int>   Scale(std::vector<int> name_lists, double  width[31],double WIDTH, double  height[31],double &HEIGHT);
 //   int Cvrecognize(const cv::Mat input_image, const std::vector<int> input_list, const std::vector<cv::Mat> (& input_des)[31], const std::vector <cv::MatND> (& input_)[31]);
};


#endif // FILTER_H_INCLUDED
