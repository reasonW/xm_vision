#include "Filter.h"
using namespace std;
Filter_Save ::Filter_Save()
{

}
Filter_Save ::~Filter_Save()
{

}
pcl::PointCloud<PointT>::Ptr Filter_Save ::RemovePlane (const pcl::PointCloud<PointT>::Ptr input_cloud)
{
    pcl::PointCloud <PointT>::Ptr cloud (new pcl::PointCloud <PointT>);
    copyPointCloud(*input_cloud,*cloud);
    pcl::apps::DominantPlaneSegmentation<PointT> dps;
    dps.setInputCloud (cloud);
    dps.setMaxZBounds (2.00f);
    dps.setObjectMinHeight (0.02);
    dps.setObjectMaxHeight(0.25);
    dps.setMinClusterSize (1000);
    dps.setWSize (9);
    dps.setDistanceBetweenClusters (0.02f);

    std::vector<pcl::PointCloud<PointT>::Ptr> clusters;
    std::vector<pcl::PointIndices> indices;
    dps.setDownsamplingSize (0.02f);
    dps.compute_fast (clusters);
    dps.getIndicesClusters (indices);
    Eigen::Vector4f table_plane_;
    Eigen::Vector3f normal_plane_ = Eigen::Vector3f (table_plane_[0], table_plane_[1], table_plane_[2]);
    dps.getTableCoefficients (table_plane_);
    pcl::PointCloud<PointT>::Ptr cluster (new pcl::PointCloud <PointT>) ;
        for (size_t tmp = 0; tmp < clusters.size (); tmp++)
        {
	  pcl::search::KdTree<PointT>::Ptr tree (new pcl::search::KdTree<PointT>);
   	 pcl::PointCloud<PointT>::Ptr mls_points (new pcl::PointCloud <PointT>) ;
	  // Init object (second point type is for the normals, even if unused)
/*	  pcl::MovingLeastSquares<PointT, PointT> mls;
	  mls.setComputeNormals (true);
	  // Set parameters
	  mls.setInputCloud (clusters[tmp]);
	  mls.setPolynomialFit (true);
	  mls.setSearchMethod (tree);
	  mls.setSearchRadius (0.03);
	  // Reconstruct
	  mls.process (*mls_points);*/
   	 *mls_points=*clusters[tmp];
	float x_min=1000000,y_min=1000000,z_min=1000000;
	float x_max=-1000000,y_max=-1000000,z_max=-1000000;//毫米
	float zm1,zm2,zm3,zm4;
	for(int i=0;i<mls_points->size();i++)
		{
		    if(mls_points->points[i].x<x_min)
		        {x_min=mls_points->points[i].x;zm1=mls_points->points[i].z;}
		    if(mls_points->points[i].y<y_min)
		        {y_min=mls_points->points[i].y;zm2=mls_points->points[i].z;}
		    if(mls_points->points[i].x>x_max)
		       { x_max=mls_points->points[i].x;zm3=mls_points->points[i].z;}
		    if(mls_points->points[i].y>y_max)
		        {y_max=mls_points->points[i].y;zm4=mls_points->points[i].z;}
		}
	std::cout<<"width "<<x_max-x_min<<"  height"<<y_max- y_min<<std::endl;
    	*cluster=( *mls_points)+(*cluster);
        }
  return cluster;
}


cv::Rect  Filter_Save ::vertex4(const pcl::PointCloud<PointT>::Ptr cloud_cluster)
{
	float x_min=1000000,y_min=1000000,z_min=1000000;
	float x_max=-1000000,y_max=-1000000,z_max=-1000000;//毫米
	float zm1,zm2,zm3,zm4;
	for(int i=0;i<cloud_cluster->size();i++)
		{
		    if(cloud_cluster->points[i].x<x_min)
		        {x_min=cloud_cluster->points[i].x;zm1=cloud_cluster->points[i].z;}
		    if(cloud_cluster->points[i].y<y_min)
		        {y_min=cloud_cluster->points[i].y;zm2=cloud_cluster->points[i].z;}
		    if(cloud_cluster->points[i].x>x_max)
		       { x_max=cloud_cluster->points[i].x;zm3=cloud_cluster->points[i].z;}
		    if(cloud_cluster->points[i].y>y_max)
		        {y_max=cloud_cluster->points[i].y;zm4=cloud_cluster->points[i].z;}
		}
	cv::Rect vertex;
	vertex.x= x_min*1.04*FOCAL/zm1+320;
	vertex.y= y_min*FOCAL/zm2+240;
	vertex.width=(x_max/zm3-x_min/zm1)*FOCAL ;
	vertex.height=(y_max/zm4-y_min/zm2)*FOCAL;
	return vertex;
}


 std::vector<Filter_Save ::Object>   Filter_Save ::Clusters(const pcl::PointCloud<PointT>::Ptr input_cloud)
{
    pcl::PointCloud <PointT>::Ptr cloud (new pcl::PointCloud <PointT>);
    copyPointCloud(*input_cloud,*cloud);
    pcl::apps::DominantPlaneSegmentation<PointT> dps;
    dps.setInputCloud (cloud);
    dps.setMaxZBounds (1.60f);
    dps.setObjectMinHeight (0.02);
    dps.setObjectMaxHeight(0.25);
    dps.setMinClusterSize (1000);
    dps.setWSize (9);
    dps.setDistanceBetweenClusters (0.02f);

    std::vector<pcl::PointCloud<PointT>::Ptr> clusters;
    std::vector<pcl::PointIndices> indices;
    dps.setDownsamplingSize (0.02f);
    dps.compute_fast (clusters);
    dps.getIndicesClusters (indices);

    std::vector<Object> my_objects;
        for (size_t i = 0; i < clusters.size (); i++)
    {
			  pcl::search::KdTree<PointT>::Ptr tree (new pcl::search::KdTree<PointT>);
		   	 pcl::PointCloud<PointT>::Ptr mls_points (new pcl::PointCloud <PointT>) ;
			  // Init object (second point type is for the normals, even if unused)
/*			  pcl::MovingLeastSquares<PointT, PointT> mls;
			  mls.setComputeNormals (true);
			  // Set parameters
			  mls.setInputCloud (clusters[i]);
			  mls.setPolynomialFit (true);
			  mls.setSearchMethod (tree);
			  mls.setSearchRadius (0.03);
			  // Reconstruct
			  mls.process (*mls_points);*/
		   	 *mls_points=*clusters[i];
			float x_min=1000000,y_min=1000000,z_min=1000000;
			float x_max=-1000000,y_max=-1000000,z_max=-1000000;
			float zm1,zm2,zm3,zm4;
			for(int tmp=0;tmp<mls_points->size();tmp++)
			 	{
				if(mls_points->points[tmp].x<x_min)
				  	   {x_min=mls_points->points[tmp].x;zm1=mls_points->points[tmp].z;}
				if(mls_points->points[tmp].y<y_min)
					   {y_min=mls_points->points[tmp].y;zm2=mls_points->points[tmp].z;}
				if(mls_points->points[tmp].x>x_max)
						   { x_max=mls_points->points[tmp].x;zm3=mls_points->points[tmp].z;}
				if(mls_points->points[tmp].y>y_max)
						   {y_max=mls_points->points[tmp].y;zm4=mls_points->points[tmp].z;}
				}
			Filter_Save ::Object pp;
			vfh_model test_model;
			LoadVFH(mls_points,test_model);
			Recognize(pp.name_lists,test_model,data,lists, pp.distances);
			cout<<"points size "<<mls_points->points.size()<<"distances  "<<"  "<<pp.distances<<endl;
			pcl::compute3DCentroid(*mls_points,pp.centroid);
 			pp.vertex=vertex4(mls_points);
 			pp.WIDTH=x_max-x_min;
 			pp.HEIGHT=y_max- y_min;
			my_objects.push_back(pp);
   }
	return my_objects;
}
std::vector<Filter_Save ::Object>   Filter_Save ::LocatOnly(const pcl::PointCloud<PointT>::Ptr input_cloud)
{
    pcl::PointCloud <PointT>::Ptr cloud (new pcl::PointCloud <PointT>);
    copyPointCloud(*input_cloud,*cloud);
    pcl::apps::DominantPlaneSegmentation<PointT> dps;
    dps.setInputCloud (cloud);
    dps.setMaxZBounds (1.70f);
    dps.setObjectMinHeight (0.02);
    dps.setObjectMaxHeight(0.25);
    dps.setMinClusterSize (1000);
    dps.setWSize (9);
//    dps.setDistanceBetweenClusters (0.01f);

    std::vector<pcl::PointCloud<PointT>::Ptr> clusters;
    std::vector<pcl::PointIndices> indices;
    dps.setDownsamplingSize (0.02f);
    dps.compute_fast (clusters);
    dps.getIndicesClusters (indices);

    std::vector<Object> my_objects;
        for (size_t i = 0; i < clusters.size (); i++)
    {
			  pcl::search::KdTree<PointT>::Ptr tree (new pcl::search::KdTree<PointT>);
		   	 pcl::PointCloud<PointT>::Ptr mls_points (new pcl::PointCloud <PointT>) ;
		   	 *mls_points=*clusters[i];
			float x_min=1000000,y_min=1000000,z_min=1000000;
			float x_max=-1000000,y_max=-1000000,z_max=-1000000;
			float zm1,zm2,zm3,zm4;
			for(int tmp=0;tmp<mls_points->size();tmp++)
			 	{
				if(mls_points->points[tmp].x<x_min)
				  	   {x_min=mls_points->points[tmp].x;zm1=mls_points->points[tmp].z;}
				if(mls_points->points[tmp].y<y_min)
					   {y_min=mls_points->points[tmp].y;zm2=mls_points->points[tmp].z;}
				if(mls_points->points[tmp].x>x_max)
						   { x_max=mls_points->points[tmp].x;zm3=mls_points->points[tmp].z;}
				if(mls_points->points[tmp].y>y_max)
						   {y_max=mls_points->points[tmp].y;zm4=mls_points->points[tmp].z;}
				}
			Filter_Save ::Object pp;
			pcl::compute3DCentroid(*mls_points,pp.centroid);
 			pp.vertex=vertex4(mls_points);
			my_objects.push_back(pp);
   }
	return my_objects;
}
std::vector<int>   Filter_Save ::Scale(std::vector<int> name_lists, double  width[31],double WIDTH, double  height[31],double &HEIGHT)
{
	std::vector<int> final_lists;
	for(size_t num=0;num<name_lists.size();num++)
	{
		int tmp=name_lists[num];
		if(WIDTH>width[tmp] || HEIGHT>height[tmp] ||(abs(WIDTH/HEIGHT)>4|| abs(WIDTH/HEIGHT)<0.1))
			continue;
		final_lists.push_back(name_lists[num]);
	}
	if(final_lists.size()==0)
	{
		for(int tmp=0;tmp<6;tmp++)
			final_lists.push_back(tmp+1);
	}
	return final_lists;

}
