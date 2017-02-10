#include "init_recognize.h"

int main(int argc, char** argv)
{
 //    cv::initModule_nonfree();
     ros::init(argc, argv, "xm_object");
     ros::NodeHandle n;
     ros::ServiceServer service = n.advertiseService("Find_Object", result);
     ros::ServiceServer server  = n.advertiseService("Find_Face",find_face);
     ros::Rate waiting_rate(30);
     LoadTFile(filter.data, filter.lists);
     object_ready(name,width,height);
     pdf_ready(pdf,font);
     ros::Subscriber cloud_client = n.subscribe(topic_cloud_name,  1,cloud_);
     ROS_INFO("cloud_client succeed ");
     ros::Subscriber rgb_client = n.subscribe(topic_rgb_name, 1, rgb_);
     ROS_INFO("rgb_client succeed ");
    cvNamedWindow("CurrentImage",CV_WINDOW_AUTOSIZE);
     ROS_INFO("windows succeed ");
    while(ros::ok())
    {
        while( !isrgb_ || !iscloud_ )
        {
            ros::spinOnce();
        }
	iscloud_=false;
	isrgb_=false;
        std::cout<<".";
       cv::imshow("CurrentImage", image);
        char temp=cvWaitKey(33);
        if(temp=='q')
        {
            HPDF_Free (pdf);
            return 0;
        }
 
    }
    HPDF_Free (pdf);
    return 0;
}
