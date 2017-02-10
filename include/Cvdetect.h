#include <stdio.h>
#include <iostream>
#include <vector>
#include <opencv2/core.hpp>
// #include <opencv2/contrib.hpp>
#include <opencv2/features2d.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/flann.hpp>
#include <opencv2/imgproc.hpp>
// this line for opencv2..4
// #include <opencv2/nonfree/features2d.hpp>
#define OBJNUM  31

 int Cvrecognize(const cv::Mat input_image , const std::vector<int>  input_list , const std::vector<cv::Mat> (& input_des)[OBJNUM], const std::vector <cv::MatND> (& input_hist)[OBJNUM])
 {
 cv::Mat img_color=input_image;
  //cv::Mat img_gray;
// cv::cvtColor(img_color,img_gray,cv::COLOR_BGR2GRAY);
 cv::Mat img_hsv;
 cv::cvtColor(img_color,img_hsv,cv::COLOR_BGR2HSV);

 cv::Mat obj_descriptor;
 int minHessian = 400;
/* cv::SurfFeatureDetector  featureDetector( minHessian );
 cv::Ptr<cv::DescriptorExtractor>  descriptorExtractor=cv::DescriptorExtractor::create( "SURF" );
 cv::Ptr<cv::DescriptorMatcher>  descriptorMatcher=cv::DescriptorMatcher::create("FlannBased");
 std::vector<cv::KeyPoint> obj_keypoint;
 featureDetector.detect(img_gray,obj_keypoint);//这里输入的是一组照片，所以特征点也要是一组
 descriptorExtractor->compute(img_gray,obj_keypoint,obj_descriptor);*/
    int h_bins = 50; int s_bins = 60;
    int histSize[] = { h_bins, s_bins };
    float h_ranges[] = { 0, 180 };
    float s_ranges[] = { 0, 256 };
    const float * ranges[] = { h_ranges, s_ranges };
    int channels[] = { 0, 1 };
    cv::MatND hist_base;
    cv::calcHist( &img_hsv, 1, channels, cv::Mat(), hist_base, 2, histSize, ranges, true, false );
    cv::normalize( hist_base, hist_base, 0, 1, cv::NORM_MINMAX, -1, cv::Mat() );
    double compare_hist=0;
    double compare_surf=0;
    int result_hist=0;
    int result_surf=0;


    double a[OBJNUM];
    memset(a,0,sizeof(a));
    std::vector<int>  input_list_pro;
 for(size_t tmp_num=0;tmp_num<input_list.size();tmp_num++)
 {
    int temp_name = input_list[tmp_num];
    if(!a[temp_name])
    {
           input_list_pro.push_back (temp_name);
    }
    a[temp_name] = a[temp_name]+1;
 }
      for(size_t tmp_num=0;tmp_num<input_list_pro.size();tmp_num++)
    {
        int tmp=input_list_pro[tmp_num];
        double tmp_hist=0;

        for(size_t num=0;num<input_hist[tmp].size();num++)
        {
              double tmp_hist_= cv::compareHist(hist_base,input_hist[tmp][num],0);
              std::cout<<tmp_hist_<<" ";
              tmp_hist+=tmp_hist_;
        }
        tmp_hist=tmp_hist/(double)input_hist[tmp].size();//according to the former six option number mul weight
        if(tmp_hist>compare_hist)
        {
                compare_hist=tmp_hist;
                result_hist=tmp;
        }
       if(tmp==1||tmp==3||tmp==4)
       {
            if(a[tmp]>=3)
            return tmp;
       }
        std::cout<<" num:"<<tmp<<"  a[tmp]:"<<a[tmp]<<"  average: "<<tmp_hist<<std::endl;
    }

 std::cout<<"result_hist "<<result_hist<<"   ";
return result_hist;
}
