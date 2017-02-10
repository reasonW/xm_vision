#include "Filter.h"
#include "Cvdetect.h"
//ros specific includes
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/Image.h>
#include <pcl_conversions/pcl_conversions.h>
#include <cv_bridge/cv_bridge.h>
#include "face_alignment.h"
#include "face_detection.h"
#include "face_identification.h"
#include <xm_msgs/xm_Object.h>
#include <xm_msgs/xm_ObjectDetect.h>
#include <xm_msgs/xm_FaceRecognize.h>
#include <geometry_msgs/PointStamped.h>

#include <hpdf.h>
#include <time.h>
#include <setjmp.h>

cv::Rect box;
cv::Mat image;
cv::Mat video;

std::string mainPath="/home/reasonw/";
const std::string topic_cloud_name("/camera/depth_registered/points");
const std::string topic_rgb_name("/camera/rgb/image_raw");
cv_bridge::CvImagePtr img_ptr;
std::string fname=mainPath+"Recognition/XiaoMeng.pdf";
std::string object_name;
std::string name[31];
double width[31];
double height[31];
std::vector<cv::Mat>  descriptor[31];
std::vector <cv::MatND> hist[31];
std::vector<cv::Mat> image_obj[31];

jmp_buf env;
time_t timep;
HPDF_Doc  pdf;
HPDF_Font font;
HPDF_Page page;
HPDF_Destination dst;
bool isrgb_=false;
bool iscloud_=false;
Filter_Save  filter;
pcl::PointCloud<PointT>::Ptr cloudP;
std::vector<Filter_Save::Object> my_objects;

class people{
   public:
    people(std::string str, float * feature) :
        name(str)
    {
        feat = new float[2048];
        memcpy( feat , feature , 2048 * sizeof ( float ) );
    }
    people()
    {
        feat =nullptr;
    }
    ~people()
    {
    }
    std::string name;
    float  *feat;
};
std::vector<people> people_vector;
void cloud_(const sensor_msgs::PointCloud2ConstPtr &ptr);
void rgb_(const sensor_msgs::ImageConstPtr &ptr);
bool sortFun(const double& r1, const double& r2);
bool sortFun2(const cv::Rect& r1, const cv::Rect& r2);
void error_handler  (HPDF_STATUS   error_no,HPDF_STATUS   detail_no,void         *user_data);
void image_str(cv::Rect vertex_pdf,int num_obj);
void object_ready(std::string* name,double* width,double* height);
void pdf_ready(HPDF_Doc& pdf,HPDF_Font& font);
void face_ready();
bool result(xm_msgs::xm_ObjectDetect::Request  &req, xm_msgs::xm_ObjectDetect::Response &res);
bool find_face(xm_msgs::xm_FaceRecognize::Request &req, xm_msgs::xm_FaceRecognize::Response &res);
