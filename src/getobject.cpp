    //pcl头文件
    #include <pcl/io/pcd_io.h>
    #include <pcl/io/png_io.h>
    #include <pcl/point_types.h>
    #include <pcl/io/openni2_grabber.h>
    #include <pcl/visualization/pcl_visualizer.h>
    #include <pcl/visualization/image_viewer.h>
    #include <pcl/visualization/keyboard_event.h>
    #include <pcl/common/common_headers.h>
    // 标准库头文件
    #include <iostream>
    #include <string>
    #include <vector>
    // OpenCV头文件
    #include <opencv2/photo/photo.hpp>
    #include <opencv2/highgui/highgui.hpp>
    // 自定义头文件
    #include "Filter.h"
    // namespace
    using namespace std;
    using namespace cv;
    using namespace pcl;

    bool Save=false;
    Filter_Save filter;
    string  name;
    int obj_order=0,photo_order=0;
    Mat cvBGRImg;
    pcl::PointCloud<PointT>::Ptr cloud_ptr(new pcl::PointCloud<PointT>); // A cloud that will store color info.
    pcl::PointCloud<PointT>::Ptr cloud_rgba(new pcl::PointCloud<PointT>);
    boost::shared_ptr<pcl::visualization::PCLVisualizer>  viewer(new pcl::visualization::PCLVisualizer("Get_Object"));

    void keyboardEventOccurred (const pcl::visualization::KeyboardEvent &event,void *viewer_void)
    {
        boost::shared_ptr<pcl::visualization::PCLVisualizer>  viewer =*static_cast<boost::shared_ptr<pcl::visualization::PCLVisualizer> *> (viewer_void);
	  if (event.getKeySym () == "r" && event.keyDown ())
	  {
		Save=true;
  	  }
     }
    void  image_callback (const boost::shared_ptr<pcl::io::openni2::Image>& image)
     {
                cvBGRImg=Mat(image->getHeight (),image->getWidth(),CV_8UC3);
	image->fillRGB(cvBGRImg.cols,cvBGRImg.rows,cvBGRImg.data,cvBGRImg.step);
	cvtColor(cvBGRImg,cvBGRImg,CV_RGB2BGR);
     }
    void cloud_callback (const pcl::PointCloud<PointT>::ConstPtr &cloud)
    {
	copyPointCloud(*cloud,*cloud_ptr);
        cloud_rgba=filter.RemovePlane(cloud_ptr);//把物体从平面上分出来
	viewer->setSize (cloud->width, cloud->height);

	if (!viewer->updatePointCloud (cloud_rgba,  "Get_Object"))
        {
          viewer->addPointCloud (cloud_rgba, "Get_Object");
          viewer->resetCameraViewpoint ("Get_Object");
          viewer->setCameraPosition (
            0,0,0,		// Position,矫正相机位姿
            0,0,1,		// Viewpoint
            0,-1,0);	// Up
        }
  	viewer->registerKeyboardCallback (keyboardEventOccurred, (void*)&viewer);
	viewer->spinOnce (100);
	if(Save)
	{
		std::ostringstream s1;
        	s1<<obj_order<<'_'<<photo_order;
		name=s1.str();
		io::savePCDFile("/home/charle/Recognition/object_point/"+name+".pcd",*cloud_rgba);
		Rect vertex=filter.vertex4(cloud_rgba);
		imwrite("/home/charle/Recognition/object_photo/"+name+".png", cvBGRImg(vertex));
		Save=false;
              		std::ofstream of("/home/charle/Recognition/object_data/photo_name.txt",std::ios::app);
                	of<<name<<endl;
                	of.close();
                	std::cout<<"Got No."<<obj_order<<"_"<<photo_order<<std::endl;
		Save=false;
		photo_order++;
	}
    }

    int main( int argc, char **argv )
    {
	obj_order=atoi(argv[1]); //这一行记得带参数，物体编号
	pcl::Grabber* asus= new io::OpenNI2Grabber();
	 // pcl::Grabber* asus= new OpenNIGrabber();// 如果用kinect请换成这一行
  	viewer->registerKeyboardCallback (keyboardEventOccurred, (void*)&viewer);
	//xtion start
	boost::function<void(const pcl::PointCloud<PointT>::ConstPtr&)> xtion = boost::bind(cloud_callback , _1);
 	boost::function<void (const boost::shared_ptr<pcl::io::openni2::Image>&) > xtion2 = boost::bind (image_callback,  _1);

	asus->registerCallback(xtion);
	asus->registerCallback(xtion2);
	asus->start();
        while(!viewer->wasStopped())
        {
	    boost::this_thread::sleep (boost::posix_time::microseconds (1));
        }
	asus->stop();
	return 0;

    }
