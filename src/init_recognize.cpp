#include "init_recognize.h"
void cloud_(const sensor_msgs::PointCloud2ConstPtr &ptr)
{
      pcl::PointCloud<PointT> cloud;
      pcl::fromROSMsg (*ptr, cloud);
      cloudP=cloud.makeShared();
	iscloud_ = true;
}

void rgb_(const sensor_msgs::ImageConstPtr &ptr)
{
	try
	{
		img_ptr = cv_bridge::toCvCopy(ptr, sensor_msgs::image_encodings::BGR8);
		image = img_ptr->image.clone();
		image.copyTo(video);
	}
	catch (cv_bridge::Exception &e) {
		ROS_ERROR("cv_bridge exception %s", e.what());
	}

	isrgb_ = true;
}
bool sortFun(const double& r1, const double& r2)
{
	return (r1 > r2);
}
bool sortFun2(const cv::Rect& r1, const cv::Rect& r2)
{
	return (r1.area() > r2.area());
}
void error_handler  (HPDF_STATUS  error_no,HPDF_STATUS  detail_no, void  *user_data)
{
    printf ("ERROR: error_no=%04X, detail_no=%u\n", (HPDF_UINT)error_no,
                (HPDF_UINT)detail_no);
    longjmp(env, 1);
}

void image_str(cv::Rect vertex_pdf,int num_obj)
{
     cv::Mat image1;
     image.copyTo(image1);
     cv::rectangle(image1,cv::Point(vertex_pdf.x,vertex_pdf.y),cv::Point(vertex_pdf.x+vertex_pdf.width,vertex_pdf.y+vertex_pdf.height),cv::Scalar(255,0,0));
    std::string my_string;
     if(num_obj!=0)
     {
     cv::putText(image1,"I'm "+name[num_obj]+"",cv::Point(vertex_pdf.x,vertex_pdf.y-8), 4,
             0.5, cv::Scalar(255,0,0), 1, 8);
     my_string=mainPath+"Recognition/"+name[num_obj]+".jpg";
     cv::imwrite(my_string,image1);
     char key2 = cvWaitKey(10);
     }
     page = HPDF_AddPage (pdf);
     HPDF_Page_SetWidth (page, 720);
     HPDF_Page_SetHeight (page,840 );
     dst = HPDF_Page_CreateDestination (page);
     HPDF_Destination_SetXYZ (dst, 0, HPDF_Page_GetHeight (page), 1);
     HPDF_SetOpenAction(pdf, dst);
     HPDF_Page_SetFontAndSize (page, font, 12);
     HPDF_Image pdf_image;
     pdf_image=HPDF_LoadJpegImageFromFile (pdf, my_string.c_str());
     HPDF_Page_DrawImage (page, pdf_image, 40, HPDF_Page_GetHeight (page) - 360,HPDF_Image_GetWidth (pdf_image),HPDF_Image_GetHeight (pdf_image));
     std::stringstream ss;
     time(&timep);
     ss<<ctime(&timep);

     HPDF_Page_BeginText (page);
     HPDF_Page_SetTextLeading (page, 16);
     HPDF_Page_MoveTextPos (page, 70, HPDF_Page_GetHeight (page) - 350);
     if(num_obj!=0)
     {
     HPDF_Page_ShowTextNextLine (page, name[num_obj].c_str());
     }
     HPDF_Page_ShowTextNextLine (page, ss.str().c_str());
     HPDF_Page_EndText (page);
     HPDF_SaveToFile (pdf, fname.c_str());
}
void object_ready(std::string* name,double* width,double* height)
{
     cv::FileStorage fs(mainPath+"Recognition/train_model/object.xml", cv::FileStorage::READ);
     cv::FileNode name_obj = fs["name"];
     cv::FileNodeIterator  it_obj = name_obj.begin(), it_obj_end = name_obj.end();
     for(int i=1;it_obj!=it_obj_end;++it_obj,++i)
            {
                name[i]=(std::string)(*it_obj);
            }
     cv::FileNode width_obj = fs["width"];
     cv::FileNodeIterator  width_it_obj = width_obj.begin(), width_it_obj_end = width_obj.end();
     for(int i=1;width_it_obj!=width_it_obj_end;++width_it_obj,++i)
            {
                width[i]=(double)(*width_it_obj);
            }
     cv::FileNode height_obj = fs["height"];
     cv::FileNodeIterator  height_it_obj = height_obj.begin(), height_it_obj_end = height_obj.end();
     for(int i=1;height_it_obj!=height_it_obj_end;++height_it_obj,++i)
            {
                height[i]=(double)(*height_it_obj);
            }
     int sum=(int)fs["sum"];
     for(int i=1;i<=sum;i++)
            {
                cv::FileNode descriptor_node = fs[name[i]]["Descriptor"];
                cv::FileNodeIterator  des_it_obj = descriptor_node.begin(), des_it_obj_end = descriptor_node.end();
                for(;des_it_obj!=des_it_obj_end;++des_it_obj)
                {
                        des_it_obj>>descriptor[i];
                }
                cv::FileNode img_node = fs[name[i]]["HSVImage"];
                cv::FileNodeIterator  img_it_obj = img_node.begin(), img_it_obj_end = img_node.end();
                for(;img_it_obj!=img_it_obj_end;++img_it_obj)
                {
                        img_it_obj>>image_obj[i];
                }
            }
    int h_bins = 50; int s_bins = 60;
    int histSize[] = { h_bins, s_bins };
    float h_ranges[] = { 0, 180 };
    float s_ranges[] = { 0, 256 };
    const float * ranges[] = { h_ranges, s_ranges };
    int channels[] = { 0, 1 };
     for(int i=1;i<=sum;i++)
            {
                for(int j=0;j<image_obj[i].size();j++)
                {
                    cv::MatND hist_base;
                    cv::calcHist( &image_obj[i][j], 1, channels, cv::Mat(), hist_base, 2, histSize, ranges, true, false );
                    cv::normalize( hist_base, hist_base, 0, 1, cv::NORM_MINMAX, -1, cv::Mat() );
                    hist[i].push_back(hist_base);
                }
            }



     ROS_INFO("object loaded succeed ");
}
void pdf_ready(HPDF_Doc& pdf,HPDF_Font& font)
{

     pdf = HPDF_New (error_handler, NULL);
     if (!pdf) {
	        printf ("error: cannot create PdfDoc object\n");
	        return ;
	       }
     if (setjmp(env)) {
	        HPDF_Free (pdf);
	        return ;
	       }

     HPDF_SetCompressionMode (pdf, HPDF_COMP_ALL);
     font = HPDF_GetFont (pdf, "Helvetica", NULL);
     ROS_INFO("pdf ready succeed ");
}

bool result(xm_msgs::xm_ObjectDetect::Request  &req,
         xm_msgs::xm_ObjectDetect::Response &res)
{
     pcl::PointCloud<PointT>::Ptr cloud_RGBA(new pcl::PointCloud<PointT>);
     *cloud_RGBA=*cloudP;
	     xm_msgs::xm_Object elect_obj;
	     my_objects=filter.Clusters(cloud_RGBA);
	     if(my_objects.size()!=0)
	     {
	        std::cout<<"got "<<my_objects.size()<<" objects"<<std::endl;
	         for(int refk=0;refk<my_objects.size();refk++)
	         {
	        my_objects[refk].name_lists=filter.Scale(my_objects[refk].name_lists,width,my_objects[refk].WIDTH,height,my_objects[refk].HEIGHT);
	    //    int num=my_objects[refk].name_lists[0];
	        int  num=Cvrecognize(image(my_objects[refk].vertex),my_objects[refk].name_lists,descriptor,hist);
                        if(my_objects[refk].WIDTH>=10)
                        	num=10;
	             if(num==0)
	             	 continue;
	             elect_obj.name=name[num];
	             elect_obj.id=num;
	        std::cout<<"name  "<<name[num] <<std::endl;
	             elect_obj.pos.point.y= -my_objects[refk].centroid[0]*1.14;//x
	             elect_obj.pos.point.z= -my_objects[refk].centroid[1]*1.14*1.14;//y
	             elect_obj.pos.point.x=my_objects[refk].centroid[2];//z
		elect_obj.pos.header.frame_id="camera_link";
		elect_obj.pos.header.stamp=ros::Time::now();
	             elect_obj.width=width[num];
	             res.objects.push_back(elect_obj);
	             image_str(my_objects[refk].vertex,num);
	         }
	     }
	     else
	     {
	         cout<<"no points,moving"<<endl;
	         return true;
	     }
  return true;
}

bool find_face(xm_msgs::xm_FaceRecognize::Request &req, xm_msgs::xm_FaceRecognize::Response &res)
{
    std::string MODEL_DIR = mainPath+"Recognition/face_model/";
    seeta::FaceAlignment alignment((MODEL_DIR + "seeta_fa_v1.1.bin").c_str());
    seeta::FaceDetection detector((MODEL_DIR + "seeta_fd_frontal_v1.0.bin").c_str());
    seeta::FaceIdentification face_recognizer((MODEL_DIR + "seeta_fr_v1.0.bin").c_str());

    detector.SetMinFaceSize(40);
    detector.SetScoreThresh(2.f);
    detector.SetImagePyramidScaleFactor(0.8f);
    detector.SetWindowStep(4, 4);
    if(req.command=="detect")
    {
    	bool flag=false;
    	int count=0;
    	while(!flag)
    	 {
               cv::Mat frame=video;
	cv::Mat gray_frame;
	if( frame.empty())
	{
                 res.result = false;
                res.message.push_back("No_face") ;
                 return false;
                }
       cv::cvtColor(frame,  gray_frame, CV_BGR2GRAY);
       seeta::ImageData image_gray(gray_frame.cols ,gray_frame.rows ,gray_frame.channels() );
       image_gray.data =  gray_frame.data;
       std::vector<seeta::FaceInfo> faces = detector.Detect(image_gray);
       int32_t face_num = static_cast<int32_t>(faces.size());
       if( face_num == 0)
       {
		if(count==10)
                                {
                                           res.result = false;
                                           res.message.push_back("No_face") ;
                                           return true;
                                }
	               std::cout<<"No face ,continue call NO."<<(10-count)<<std::endl;
                   	count++;
	                continue;
      }
	for (int32_t i = 0; i < face_num; i++)
	{
		cv::Rect tmp_rect;
		tmp_rect.x = faces[i].bbox.x;
		tmp_rect.y = faces[i].bbox.y;
		tmp_rect.width = faces[i].bbox.width;
		tmp_rect.height = faces[i].bbox.height;
                            if ( (tmp_rect.x<=0)||(tmp_rect.x+tmp_rect.width >=640 )||(tmp_rect.y<=0)||(tmp_rect.y+tmp_rect.height >=480) )
                            {
                                continue;
                            }
	               std::stringstream ss;
	               ss<<mainPath<<"face"<<tmp_rect.x <<"+"<<tmp_rect.y;
	               std::string str1;
	               ss>>str1;
                              cv::imwrite(str1 + ".jpg ",frame(tmp_rect) );
	              float ux= tmp_rect.x + tmp_rect.width/2.0;
	              float uy= tmp_rect.y + tmp_rect.height/2.0;
	              float center_z=(float)((cloudP->at(ux,uy)).z);
                               while(std::isnan(center_z))
                                {
                                    ux=ux+7;
                                    center_z=(float)((cloudP->at(ux,uy)).z);
                                }
	              float center_x=(ux-320)*(center_z)/525.0;
	              float center_y=(uy-240)*(center_z)/525.0;
	              geometry_msgs::PointStamped tmp_pos;
	              tmp_pos.point.y=(-center_x-0.045)*1.14;
	              tmp_pos.point.z=-center_y*1.14*1.14;
	              tmp_pos.point.x=center_z;
	              tmp_pos.header.frame_id="camera_link";
	              tmp_pos.header.stamp=ros::Time::now();
	             res.pos.push_back(tmp_pos);
	             res.result = true;
	             flag =true;

	}
    	}
    }

    else if(req.command == "remember")
    {
                bool flag = false;
                int count=0;
                while(!flag)
                {
                cv::Mat frame=video;
                cv::Mat gray_frame;
                if( frame.empty())
                {
                 res.result = false;
                res.message.push_back("No_face") ;
                  return false;
                }
                cv::cvtColor(frame,  gray_frame, CV_BGR2GRAY);
                seeta::ImageData image_gray(gray_frame.cols ,gray_frame.rows ,gray_frame.channels() );
                image_gray.data =  gray_frame.data;
                std::vector<seeta::FaceInfo> faces = detector.Detect(image_gray);
                int32_t face_num = static_cast<int32_t>(faces.size());
                if( face_num == 0)
                {
                                           if(count==10)
                                           {
                                           res.result = false;
                                           res.message.push_back("No_face") ;
                                           return true;
                                           }
                                           std::cout<<"No face ,continue call NO."<<(10-count)<<std::endl;
                                           count++;
                                           continue;
                }
                std::vector<double> face_area;
                for (int32_t i = 0; i < face_num; i++)
                {
                double tmp_area=faces[i].bbox.width * faces[i].bbox.height;
                face_area.push_back(tmp_area);
                }
                sort(face_area.begin(), face_area.end(), sortFun);
                ROS_INFO("I will remember the nearest person ");
                //crop the nearest face

            seeta::FacialLandmark points[5];
            float features[2048];
            alignment.PointDetectLandmarks(image_gray,faces[0],points);
            seeta::ImageData image_color(frame.cols ,frame.rows ,frame.channels() );
            image_color.data =  frame.data;
            face_recognizer.ExtractFeatureWithCrop(image_color, points, features);
            people   person(req.name,features);
            people_vector.push_back(person);
            cv::Rect face_rect;
            face_rect.x = faces[0].bbox.x;
            face_rect.y = faces[0].bbox.y;
            face_rect.width = faces[0].bbox.width;
            face_rect.height = faces[0].bbox.height;
            if ( (face_rect.x<=0)||(face_rect.x+face_rect.width >=640 )||(face_rect.y<=0)||(face_rect.y+face_rect.height >=480) )
            {
                        ROS_INFO("Sorry ,I can't find a complete facial in my line of sight. I will try it again..... ");
                                           if(count==10)
                                           {
                                           res.result = false;
                                           res.message.push_back("No_face") ;
                                           return true;
                                           }
                                           count++;
                                           continue;
                }
            cv::Mat face_nearest=frame(face_rect);
            cv::imwrite(mainPath+"Recognition/face_data/remember/"+req.name + ".jpg ",face_nearest );
            res.result = true;
            flag = true;
            ROS_INFO("I remember U!!");
            return true;
        }
    }
    else if(req.command == "recognize")
    {
        bool flag = false;
	int count=0;
	std::cout<<people_vector.size()<<std::endl;
/*	if(people_vector.size()==0)
	{
	  std::cout<<"waiting me loading previous faces"<<std::endl;
	  cv::Directory dir;
          std::string imgPath=mainPath+"Recognition/face_data/remember/";
  	  std::string imgExt=".jpg";
          std::vector<std::string> imgNames=dir.GetListFiles(imgPath,imgExt);
	  for (size_t filNum=0;filNum<imgNames.size();filNum++)
	  {
		    cv::Mat img=cv::imread(imgPath+imgNames[filNum]) ;
		    if(img.empty())
		    {
			std::cout<<imgNames[filNum]<<"empty"<<std::endl;
		        continue;
		    }
		    cv::Mat img_gray;
		    cv::cvtColor(img, img_gray, CV_BGR2GRAY);

		    seeta::ImageData tmp_color(img.cols ,img.rows ,img.channels() );
		    tmp_color.data =  img.data;
		    seeta::ImageData image_data(img_gray.cols ,img_gray.rows ,img_gray.channels() );
		    image_data.data =  img_gray.data;
		    std::vector<seeta::FaceInfo> faces = detector.Detect(image_data);
		    int32_t face_num = static_cast<int32_t>(faces.size());
		    if( face_num == 0)
		    {
		        continue;
		    }
		    seeta::FacialLandmark points[5];
		    float features[2048];
		    alignment.PointDetectLandmarks(image_data,faces[0],points);
		    face_recognizer.ExtractFeatureWithCrop(tmp_color, points, features);
                    imgNames[filNum].erase(imgNames[filNum].size()-5,5);
	 std::cout<<imgNames[filNum]<<std::endl;
		    people   person(imgNames[filNum],features);
		    people_vector.push_back(person);
	  }

	}
	std::cout<<people_vector.size()<<std::endl;*/
        while(!flag && (people_vector.size()!=0))
        {
            cv::Mat frame=video;
	    cv::Mat gray_frame;
            if ( frame.empty() )
	    {
		res.result = false;
                               res.message.push_back("No_face") ;
		return false;
	    }
            cv::cvtColor(frame,  gray_frame, CV_BGR2GRAY);
            seeta::ImageData image_gray(gray_frame.cols ,gray_frame.rows ,gray_frame.channels() );
            image_gray.data =  gray_frame.data;

            std::vector<seeta::FaceInfo> faces = detector.Detect(image_gray);
            int32_t face_num = static_cast<int32_t>(faces.size());
            if( face_num == 0)
            {
             if(count==10)
             {
                 res.result = false;
                                           res.message.push_back("No_face") ;
                 return true;
             }
             std::cout<<"No face ,continue call NO."<<(10-count)<<std::endl;

             count++;
             continue;
            }
	    std::vector<cv::Rect> face_rect;
            for (int32_t i = 0; i < face_num; i++)
	    {
		cv::Rect tmp_rect;
		tmp_rect.x = faces[i].bbox.x;
		tmp_rect.y = faces[i].bbox.y;
		tmp_rect.width = faces[i].bbox.width;
		tmp_rect.height = faces[i].bbox.height;
                                if ( (tmp_rect.x<=0)||(tmp_rect.x+tmp_rect.width >=640 )||(tmp_rect.y<=0)||(tmp_rect.y+tmp_rect.height >=480) )
                                {
                                            faces.erase(faces.begin()+i);
                                            continue;
                                }
                                face_rect.push_back(tmp_rect);
	    }
	    bool isRepeat=false;
	    std::vector<std::pair<std::string, double> > history_array;
            for (int32_t face_count = 0; face_count < face_rect.size(); face_count++)//for each gotten face
	    {

                std::vector<double> sim_value_arry(0.0);
		seeta::FacialLandmark points[5];
		float features[2048];
		alignment.PointDetectLandmarks(image_gray,faces[face_count],points);
                seeta::ImageData image_color(frame.cols ,frame.rows ,frame.channels() );
                image_color.data =  frame.data;
                face_recognizer.ExtractFeatureWithCrop(image_color, points, features);
		for(size_t peo_count = 0 ; peo_count < people_vector.size() ;peo_count++)
		    {
		        double temp_sim =face_recognizer.CalcSimilarity(features,people_vector[peo_count].feat );
		        sim_value_arry.push_back(temp_sim);
		        ROS_INFO("DEBUG  %f",temp_sim);
		    }
		int temp;
		double temp_sim=0;
		for(size_t peo_num =0 ; peo_num  < people_vector.size() ; peo_num ++ )
		    {
		        if( sim_value_arry [ peo_num ] >= temp_sim )
		        {
		            temp = peo_num  ;
		            temp_sim = sim_value_arry[peo_num ];
		        }
		    }
	        std::cout<<"check faces"<<std::endl;
		for (size_t check=0;check<history_array.size();check++)
		    {
			if(people_vector [temp].name==history_array[check].first)
			    {
				if (temp_sim>history_array[check].second)
				    {
					history_array[check].first=people_vector [temp].name;
					history_array[check].second=temp_sim;
		        		flag =true;
				        res.result = true;
				        res.message[check]=people_vector [temp].name;
                		        float ux= face_rect[face_count].x + face_rect[face_count].width/2.0;
                		        float uy= face_rect[face_count].y + face_rect[face_count].height/2.0;
                		        float center_z=(float)((cloudP->at(ux,uy)).z);
		                               while(std::isnan(center_z))
		                                {
		                                    ux=ux+7;
		                                    center_z=(float)((cloudP->at(ux,uy)).z);
		                                }
                		        float center_x=(ux-320)*(center_z)/525.0;
                		        float center_y=(uy-240)*(center_z)/525.0;
                		        geometry_msgs::PointStamped tmp_pos;
                                                          tmp_pos.point.y =(-center_x-0.045)*1.14;
                                                          tmp_pos.point.z =-center_y*1.14*1.14;
                                                          tmp_pos.point.x =center_z;
			              tmp_pos.header.frame_id="camera_link";
			              tmp_pos.header.stamp=ros::Time::now();
	        			res.pos[check]=tmp_pos;
	        			cv::imwrite(mainPath+"Recognition/face_data/"+people_vector [temp].name + ".jpg" ,frame(face_rect[face_count]) );
				    }
			 	isRepeat=true;
  			    }

		    }
		if(!isRepeat)
		    {
			std::pair<std::string,double> tmp_array;
			tmp_array.first=people_vector [temp].name;
			tmp_array.second=temp_sim;
			history_array.push_back(tmp_array);
		        flag =true;
		        res.result = true;
		        res.message.push_back(people_vector [temp].name);
                        float ux= face_rect[face_count].x + face_rect[face_count].width/2.0;
                        float uy= face_rect[face_count].y + face_rect[face_count].height/2.0;
                        float center_z=(float)((cloudP->at(ux,uy)).z);
                               while(std::isnan(center_z))
                                {
                                    ux=ux+7;
                                    center_z=(float)((cloudP->at(ux,uy)).z);
                                }
                        float center_x=(ux-320)*(center_z)/525.0;
                        float center_y=(uy-240)*(center_z)/525.0;
                        geometry_msgs::PointStamped tmp_pos;
                        tmp_pos.point.y=(-center_x-0.045)*1.14;
                        tmp_pos.point.z=-center_y*1.14*1.14;
                        tmp_pos.point.x=center_z;
	              tmp_pos.header.frame_id="camera_link";
	              tmp_pos.header.stamp=ros::Time::now();
	        	res.pos.push_back(tmp_pos);
	        	cv::imwrite(mainPath+"Recognition/face_data/"+people_vector [temp].name + ".jpg" ,frame(face_rect[face_count]) );
		    }

	    }
	  }
        return true;
    }//if(recognize)
    return true;
}//serivce
