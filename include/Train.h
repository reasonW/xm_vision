#ifndef TRAIN
#define TRAIN
//从文件中获取图片名称，载入，分批进行处理，最后识别
#include <iostream>
#include <string>
#include <fstream>
#include <vector>
#include <cstdio>
#include <pcl/point_cloud.h>
#include <pcl/io/pcd_io.h>
#include <pcl/console/print.h>
#include <pcl/features/normal_3d.h>
#include <pcl/features/our_cvfh.h>
#include <pcl/kdtree/kdtree.h>
#include <flann/flann.h>
#include <flann/io/hdf5.h>
#include <boost/filesystem.hpp>
typedef pcl::PointXYZRGB PointT2;
typedef pcl::VFHSignature308 VFH308;
typedef  std::pair<std::string, std::vector<float> > vfh_model;
std::string mainPath="/home/reasonw/";
//using namespace std;
//using namespace cv;
class Train
{
public:
    Train();
    ~Train();
    void EstimateNormals (const pcl::PointCloud<PointT2>::Ptr input_cloud,
                 pcl::PointCloud<pcl::Normal>::Ptr cloud_normals);
    void EstimateOUR_CVFH2 (const pcl::PointCloud<PointT2>::Ptr input_cloud,
             pcl::PointCloud<VFH308>::Ptr cvfhs);
    void SaveVFH (const pcl::PointCloud<VFH308>::Ptr vfhs,
         std::string name);
    void LoadVFH2 (std::string name,
         vfh_model &vfh);
    void LoadFeatureModels (const boost::filesystem::path &base_dir,
                   const std::string &extension,
                   std::vector<vfh_model> &models);
    void SaveModel (std::vector<vfh_model> models,
           std::string training_data_h5_file_name,
           std::string training_data_list_file_name,
           std::string kdtree_idx_file_name) ;
    void ExtractCVFH (const boost::filesystem::path &base_dir,
            const std::string &extension);
};

#endif // TRAIN_H_INCLUDED
