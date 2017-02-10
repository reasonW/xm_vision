#include <pcl/io/pcd_io.h>
#include <pcl/console/print.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/features/normal_3d.h>
#include <pcl/features/our_cvfh.h>
#include <iostream>
#include <flann/flann.h>
#include <flann/io/hdf5.h>
#include <boost/filesystem.hpp>
int obj_sum=30;
int arguments=6;
#define training_h5_name   "/home/reasonw/Recognition/train_model/training.h5"
#define training_list_name "/home/reasonw/Recognition/train_model/training.list"
#define kdtree_idx_name    "/home/reasonw/Recognition/train_model/kdtree.idx"
typedef pcl::PointXYZRGBA PointT;
typedef pcl::PointXYZRGB PointT2;
typedef pcl::VFHSignature308 VFH308;
typedef  std::pair<std::string, std::vector<float> > vfh_model;
void
nearestKSearch (flann::Index<flann::ChiSquareDistance<float> > &index,
                const vfh_model &model,
                int k,
                flann::Matrix<int> &indices,
                flann::Matrix<float> &distances)
{
    flann::Matrix<float> p = flann::Matrix<float>(new float[model.second.size ()], 1, model.second.size());
    memcpy (&p.ptr ()[0], &model.second[0], p.cols * p.rows * sizeof (float));
    indices = flann::Matrix<int>(new int[k], 1, k);
    distances = flann::Matrix<float>(new float[k], 1, k);
    index.knnSearch (p, indices, distances, k, flann::SearchParams (512));
    delete[] p.ptr();
}

void
EstimateNormals (const pcl::PointCloud<PointT2>::Ptr input_cloud,
                 pcl::PointCloud<pcl::Normal>::Ptr cloud_normals)
{
  pcl::NormalEstimation<PointT2, pcl::Normal> normal;
  normal.setInputCloud (input_cloud);
  pcl::search::KdTree<PointT2>::Ptr tree (new pcl::search::KdTree<PointT2> ());
  normal.setSearchMethod (tree);
  normal.setRadiusSearch (0.03);
  normal.compute (*cloud_normals);
}

void
EstimateOUR_CVFH (const pcl::PointCloud<PointT>::Ptr input_cloud,
             pcl::PointCloud<VFH308>::Ptr our_cvfhs)

{
    pcl::PointCloud<PointT2>::Ptr input_cloud2(new pcl::PointCloud<PointT2>);
    pcl::copyPointCloud (*input_cloud,*input_cloud2);
    pcl::OURCVFHEstimation<PointT2, pcl::Normal, VFH308> our_cvfh;
    pcl::PointCloud<pcl::Normal>::Ptr normals (new pcl::PointCloud<pcl::Normal>);
    EstimateNormals(input_cloud2, normals);
    our_cvfh.setInputCloud (input_cloud2);
    our_cvfh.setInputNormals (normals);
    //if cloud is of type PointNormal, do vfh.setInputNormals (cloud);
    pcl::search::KdTree<PointT2>::Ptr tree (new pcl::search::KdTree<PointT2> ());
    our_cvfh.setSearchMethod (tree);
    our_cvfh.setEPSAngleThreshold(5.0 / 180.0 * M_PI);  // 5 degrees.
    our_cvfh.setCurvatureThreshold(1.0);
    our_cvfh.setNormalizeBins(false);
    our_cvfh.setAxisRatio(1.0);
//	our_cvfh.setMinPoints (50);

//	our_cvfh.setCurvatureThreshold (0.025f);

//	our_cvfh.setClusterTolerance (0.015f); //1.5cm, three times the leaf size

//	our_cvfh.setEPSAngleThreshold (0.13f);
    our_cvfh.compute (*our_cvfhs);
}

void
LoadVFH (const pcl::PointCloud<PointT>::Ptr input_cloud,
         vfh_model &vfh_m)
{
    pcl::PointCloud<VFH308>::Ptr vfhs (new pcl::PointCloud<VFH308>);
    EstimateOUR_CVFH (input_cloud, vfhs);
    for(size_t i = 0; i < 308; i ++)
    {
        float tmp;
        tmp  = vfhs->points[0].histogram[i];
        vfh_m.second.push_back(tmp);
    }
    vfh_m.first = "unknown";
    vfh_m.second.resize(309);
}

std::string
GetName (std::string full_path)
{
    std::string name;
    std::string::size_type pos;
    pos = full_path.rfind ('_');
    if (pos == std::string::npos)
        std::cout << "I can't find a '_' in " << full_path << std::endl;
    name = full_path.erase (pos);
    pos = name.rfind ('/');
    if (pos == std::string::npos)
        std::cout << "I can't find a '/' in " << name << std::endl;
    name = name.substr (pos + 1);
    return name;
}

void
LoadList (std::string name,
          std::vector<std::string> &lists)
{
    std::fstream in;
    in.open (name.c_str ());
    if (! in.is_open())
    {
        std::cout << "can't open list file" << std::endl;
        exit (1);
    }
    while (!in.eof ())
    {
        std::string temp;
        in >> temp;
        if (temp.size() == 0)continue;
        temp = GetName(temp);
        lists.push_back (temp);
    }
    in.close();
}

void
LoadTFile(flann::Matrix<float> &data,
                std::vector<std::string> &lists)
{
    if (!boost::filesystem::exists (training_h5_name) || !boost::filesystem::exists (training_list_name))
    {
        pcl::console::print_error ("Could not find training data models files training.h5 and training.list!\n");
        return;
    }
    else
    {
        LoadList (training_list_name, lists);
        flann::load_from_file (data, training_h5_name, "training_data");
        pcl::console::print_highlight ("Training data found. Loaded %d VFH models from training.h5/training_data.\n",
            (int)data.rows);
    }
}

void
Recognize (std::vector<int> &flags,
           vfh_model histogram,
           flann::Matrix<float> &data,
           std::vector<std::string> &lists,
		double &distances)
{
    flann::Matrix<int> k_indices;
    flann::Matrix<float> k_distances;
    if (!boost::filesystem::exists (kdtree_idx_name))
    {
        pcl::console::print_error ("Could not find kd-tree index in file kdtree.idx!\n");
        return ;
    }
    else
    {
        flann::Index<flann::ChiSquareDistance<float> > index (data, flann::SavedIndexParams (kdtree_idx_name)); // load index
        index.buildIndex ();
        nearestKSearch (index, histogram, arguments, k_indices, k_distances);
    }
    // Output the results on screen
    pcl::console::print_highlight ("The closest %d neighbors ", arguments);
    for (int i = 0; i < arguments; ++i)
    {
        int temp_name = atoi(lists.at(k_indices[0][i]).c_str());
        pcl::console::print_info ("    %d - %s (%d) with a distance of: %f\n",
            i, lists.at (k_indices[0][i]).c_str (), k_indices[0][i], k_distances[0][i]);
        flags.push_back (temp_name);
    }
    distances= k_distances[0][0];
    pcl::console::print_highlight ("\n");
}
