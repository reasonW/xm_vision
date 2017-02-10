#include "Train.h"
typedef pcl::VFHSignature308 VFH308;
typedef  std::pair<std::string, std::vector<float> > vfh_model;

Train::Train()
{


}
Train::~Train()
{

}

void Train::EstimateNormals (const pcl::PointCloud<PointT2>::Ptr input_cloud,
                 pcl::PointCloud<pcl::Normal>::Ptr cloud_normals)
{
  pcl::NormalEstimation<PointT2, pcl::Normal> normal;
  normal.setInputCloud (input_cloud);
  pcl::search::KdTree<PointT2>::Ptr tree (new pcl::search::KdTree<PointT2> ());
  normal.setSearchMethod (tree);
  normal.setRadiusSearch (0.03);
  normal.compute (*cloud_normals);
}

void Train::EstimateOUR_CVFH2 (const pcl::PointCloud<PointT2>::Ptr input_cloud,
             pcl::PointCloud<VFH308>::Ptr our_cvfhs)
{
    pcl::OURCVFHEstimation<PointT2, pcl::Normal, VFH308> our_cvfh;
    pcl::PointCloud<pcl::Normal>::Ptr normals (new pcl::PointCloud<pcl::Normal>);
    EstimateNormals(input_cloud, normals);
    our_cvfh.setInputCloud (input_cloud);
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

void Train::SaveVFH (const pcl::PointCloud<VFH308>::Ptr vfhs,
std::string name)
{
    std::ofstream out;
    float temp;
     out.open (name.c_str());
     if (out.is_open ())
    {
        for (size_t i = 0; i<308; i++)
        {
            temp = vfhs->points[0].histogram[i];
            out << temp << std::endl;
        }
        std::cout << "I have saved " << name << std::endl;
        out.close ();
    }
}

void Train::LoadVFH2 (std::string name,
         vfh_model &vfh)
{
    std::ifstream in;
    in.open (name.c_str());
    if (! in.is_open() )
    {
        std::cout << "can't open this file" << std::endl;
        exit (1);
    }
    while (!in.eof ())
    {
        float temp;
        in >> temp;
        vfh.second.push_back (temp);
    }
    vfh.first = name;
    std::cout << "I have loaded " << name << std::endl;
}


void Train::LoadFeatureModels (const boost::filesystem::path &base_dir,
                   const std::string &extension,
                   std::vector<vfh_model> &models)
{
    if (!boost::filesystem::exists (base_dir) && !boost::filesystem::is_directory (base_dir))
    return;

    for (boost::filesystem::directory_iterator it (base_dir); it != boost::filesystem::directory_iterator (); ++it)
    {
        if (boost::filesystem::is_directory (it->status ()))
        {
            std::stringstream ss;
            ss << it->path ();
            pcl::console::print_highlight ("Loading %s (%lu models loaded so far).\n", ss.str ().c_str (), (unsigned long)models.size ());
            LoadFeatureModels (it->path (), extension, models);
        }
        if (boost::filesystem::is_regular_file (it->status ()) && boost::filesystem::extension (it->path ()) == extension)
        {
            vfh_model m;
            std::stringstream ss;
            ss << base_dir.string () << "/" << it->path ().filename ().string ();
            LoadVFH2 (ss.str(), m);
            models.push_back (m);
        }
  }
}

void Train::SaveModel (std::vector<vfh_model> models,
           std::string training_data_h5_file_name,
           std::string training_data_list_file_name,
           std::string kdtree_idx_file_name)      //定义的时候扩展名分别用.h5  .list  .idx
{
    // Convert data into FLANN format
    flann::Matrix<float> data (new float[models.size () * models[0].second.size ()], models.size (), models[0].second.size ());

    for (size_t i = 0; i < data.rows; ++i)
        for (size_t j = 0; j < data.cols; ++j)
            data[i][j] = models[i].second[j];
    // Save data to disk (list of models)
    flann::save_to_file (data, training_data_h5_file_name, "training_data");
    std::ofstream fs;
    fs.open (training_data_list_file_name.c_str ());
    for (size_t i = 0; i < models.size (); ++i)
        fs << models[i].first << "\n";
    fs.close ();
    // Build the tree index and save it to disk
    pcl::console::print_error ("Building the kdtree index (%s) for %d elements...\n", kdtree_idx_file_name.c_str (), (int)data.rows);
    flann::Index<flann::ChiSquareDistance<float> > index (data, flann::LinearIndexParams ());
    //flann::Index<flann::ChiSquareDistance<float> > index (data, flann::KDTreeIndexParams (4));
    index.buildIndex ();
    index.save (kdtree_idx_file_name);
    delete[] data.ptr ();

}

void Train::ExtractCVFH (const boost::filesystem::path &base_dir,
            const std::string &extension)
{
    if (!boost::filesystem::exists (base_dir) && !boost::filesystem::is_directory (base_dir))
    return;

    for (boost::filesystem::directory_iterator it (base_dir); it != boost::filesystem::directory_iterator (); ++it)
    {
        if (boost::filesystem::is_directory (it->status ()))
        {
            std::stringstream ss;
            ss << it->path ();
            ExtractCVFH (it->path (), extension);
        }
        if (boost::filesystem::is_regular_file (it->status ()) && boost::filesystem::extension (it->path ()) == extension)
        {
            pcl::PointCloud<PointT2>::Ptr cloud (new pcl::PointCloud<PointT2>);
            pcl::PointCloud<VFH308>::Ptr vfhs (new pcl::PointCloud<VFH308>);
            pcl::PCDReader reader;
            std::string name;

            name += base_dir.string ();
            name += "/";
            name += it->path ().filename ().string ();

            reader.read (name, *cloud);
            EstimateOUR_CVFH2(cloud, vfhs);

            name.erase (name.size () - 3, 3);
            name += "vfh";
            SaveVFH (vfhs, name);
        }
  }
}

