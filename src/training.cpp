#include "Train.h"
#include <fstream>
#include <sstream>
#include <opencv2/opencv.hpp>
#include <opencv2/core.hpp>
#include <opencv2/features2d.hpp>
#include <opencv2/highgui.hpp>
// this line for opencv2.4
// #include <opencv2/nonfree/features2d.hpp>
#include <opencv2/imgproc.hpp>

using namespace std;
typedef pcl::VFHSignature308 VFH308;
typedef  std::pair<std::string, std::vector<float> > vfh_model;


int main()
{
    int sum;
    std::vector<std::string>  photo_str;
    
    double width[31];
    double height[31];
    std::string obj_name[31];
    std::string filename;
    std::cout<<"please input the totle number of objects"<<std::endl;
    std::cin>>sum;

    std::fstream file(mainPath+"Recognition/object_data/photo_name.txt");
    std::fstream file_width(mainPath+"Recognition/object_data/width.txt");
    std::fstream file_height(mainPath+"Recognition/object_data/height.txt");
    std::fstream file_name(mainPath+"Recognition/object_data/object_name.txt");

    if(!file.is_open())
    {

        std::cout<<"can't open photo_name.txt"<<std::endl;
        return -1;
    }
    if(!file_width.is_open())
    {
        std::cout<<"can't open width.txt"<<std::endl;
        return -1;
    }
    if(!file_height.is_open())
    {
        std::cout<<"can't open height.txt"<<std::endl;
        return -1;
    }
    if(!file_name.is_open())
    {
        std::cout<<"can't open object_name.txt"<<std::endl;
        return -1;
     }
    while(!file.eof())
    {
        std::string str;
        getline(file,str);
        if(str.empty())
            break;
        photo_str.push_back(str);
    }
    file.close();
    std::cout<<"Has object_data inputed?  y/n"<<std::endl;
    char got;
    std::cin>>got;
    if(got=='y')
    {
        for(int k=1;(k<=sum)&&(!file_name.eof());k++)
        {
            std::string str;
            getline(file_name,str);
            if(str.empty())
                break;
            obj_name[k]=str;
            std::cout<<"name "<<str<<std::endl;
        }
        for(int k=1;(k<=sum)&&(!file_width.eof());k++)
        {
            std::string str;
            getline(file_width,str);
            if(str.empty())
                break;
            double tmp=atof(str.c_str());
            width[k]=tmp;
            std::cout<<"width "<<tmp<<std::endl;
        }
        for(int k=1;(k<=sum)&&(!file_height.eof());k++)
        {
            std::string str;
            getline(file_height,str);
            if(str.empty())
                break;
            double tmp=atof(str.c_str());
            height[k]=tmp;
            std::cout<<"height "<<tmp<<std::endl;
        }
    }
    else
    {

        for (int k=1;k<=sum;k++)
        {
            std::cout<<"please input the NO."<<k<<" object's name"<<std::endl;
        std::cin>>obj_name[k];
        file_name<<obj_name[k]<<std::endl;
        std::cout<<"please input the NO."<<k<<" object's width"<<std::endl;
        std::cin>>width[k];
        file_width<<width[k]<<std::endl ;
        std::cout<<"please input the NO."<<k<<" object's height"<<std::endl;
        std::cin>>height[k];
        file_height<<height[k]<<std::endl ;
        }
    }
        file_width.close();
        file_height.close();
        file_name.close();

    std::cout<<"train 1 begin"<<std::endl;
     int minHessian = 400;
     // this line for opencv3+
     cv::Ptr<cv::AKAZE> featureDetector = cv::AKAZE::create();
// // this lines for opencv2.4
//     cv::SurfFeatureDetector  featureDetector( minHessian );
//     cv::Ptr<cv::DescriptorExtractor>  descriptorExtractor=cv::DescriptorExtractor::create( "SURF" );
    std::vector<cv::Mat>  obj_discriptor[31]; //最多30个物品,物体编号统一从１开始
 //   std::vector<cv::MatND> obj_hist[31];
    std::vector <cv::Mat> obj_img[31];
    std::vector <cv::Mat> obj_img_bgr[31];
    std::vector <cv::Mat> obj_img_hsv[31];
    std::cout<<"train 1  photo size "<< photo_str.size()<<std::endl;

    for( size_t i = 0; i <  photo_str.size(); i++ )
    {
        filename = photo_str[i];
        cv::Mat img_color = cv::imread( mainPath+"Recognition/object_photo/"+filename+".png" );
        if( img_color.empty() )
        {
            std::cout << "Train image " << filename << " can not be read." << std::endl;
            return 0;
        }
        cv::Mat img_gray;
        cv::cvtColor(img_color,img_gray,cv::COLOR_BGR2GRAY);
       cv::Mat img_hsv;
        cv::cvtColor(img_color,img_hsv,cv::COLOR_BGR2HSV);

            /*            cv::Mat img_hue;
                    cv::cvtColor(img_color,img_hsv,cv::COLOR_BGR2HSV);
                    img_hue.create(img_hsv.size(),img_hsv.depth());
                    int ch[]={0,0};
                    cv::mixChannels(&img_hsv,1,&img_hue,1,ch,1);
                    cv::MatND hist;
                    int h_bins = 50; int s_bins = 60;
                    int histSize[] = { h_bins, s_bins };
                    float h_ranges[] = { 0, 180 };
                    float s_ranges[] = { 0, 256 };
                    const float * ranges[] = { h_ranges, s_ranges };
                    int channels[] = { 0, 1 };
                    cv::calcHist(&img_hue,1,channels,cv::Mat(),hist,2,histSize,ranges,true,false);
                    cv::normalize(hist,hist,0,1,cv::NORM_MINMAX,-1,cv::Mat());*/

        std::string obj_str;
        for(int file_num=0;file_num<filename.size();file_num++)
            {
                if(filename[file_num]!='_')
                    obj_str+=filename[file_num];
            else
                break;
            }
       int tmp=atoi(obj_str.c_str());
        obj_img_bgr[tmp].push_back(img_color);
        obj_img[tmp].push_back(img_gray);
        obj_img_bgr[tmp].push_back(img_color);
        obj_img_hsv[tmp].push_back(img_hsv);

 //       obj_hist[tmp].push_back(hist);
        obj_str.clear();
    }
    std::cout<<"train 1  sum "<<sum<<std::endl;

    for (int tmp=1;tmp<=sum;tmp++)
    {
            std::vector<std::vector<cv::KeyPoint> >    obj_keypoint;
            // // this lines for opencv2.4
            // featureDetector.detect(obj_img[tmp],obj_keypoint);//这里输入的是一组照片，所以特征点也要是一组
            // descriptorExtractor->compute(obj_img[tmp],obj_keypoint,obj_discriptor[tmp]);
            // this line for opencv3+
            for (size_t num_img=0;num_img<obj_img[tmp].size();num_img++)
            {
                
                featureDetector->detectAndCompute(obj_img[tmp][num_img],cv::noArray(),obj_keypoint[num_img],obj_discriptor[tmp][num_img]);
            }


    }
    cv::FileStorage fs1(mainPath+"Recognition/train_model/object.xml", cv::FileStorage::WRITE);
    int obj_num;
    fs1<<"sum"<<sum;
    for(int tmp=1;tmp<=sum;tmp++)
    {
        fs1<<obj_name[tmp];
        fs1<<"{";
        fs1<<"HSVImage";
        {
        fs1<<obj_img_hsv[tmp];
        }
        fs1<<"Descriptor";
        {
        fs1<<obj_discriptor[tmp];
        }
        fs1<<"}";
    std::cout<<"train 1 image finished "<<tmp<<std::endl;
    }
    fs1<<"name"<<"[";
    for(obj_num=1;obj_num<=sum;obj_num++)
    {
        fs1<<obj_name[obj_num];
    }
    fs1<<"]";
    std::cout<<obj_name<<std::endl;
    fs1<<"width"<<"[";
    for(obj_num=1;obj_num<=sum;obj_num++)
    {
        fs1<<width[obj_num];
    }
    fs1<<"]";
    fs1<<"height"<<"[";
    for(obj_num=1;obj_num<=sum;obj_num++)
    {
        fs1<<height[obj_num];
    }
    fs1<<"]";
    std::cout<<"train 1 finished"<<std::endl;
    fs1.release();
    Train train_obj;
    train_obj.ExtractCVFH(mainPath+"Recognition/object_point",".pcd");
    std::vector<vfh_model> models;
    std::cout<<1<<std::endl;
    train_obj.LoadFeatureModels(mainPath+"Recognition/object_point",".vfh",models);
    std::cout<<models.size()<<std::endl;
    std::cout<<2<<std::endl;
    train_obj.SaveModel(models,mainPath+"Recognition/train_model/training.h5",mainPath+"Recognition/train_model/training.list",mainPath+"Recognition/train_model/kdtree.idx");
    std::cout<<3<<std::endl;
    std::cout<<"train down"<<std::endl;
    return 0;
}
