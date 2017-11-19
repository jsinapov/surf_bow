#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>

#include <opencv2/core.hpp>
#include "opencv2/features2d.hpp"
#include "opencv2/xfeatures2d.hpp"
#include "opencv2/highgui.hpp"
//#include <opencv2/nonfree/nonfree.hpp>
/*#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/opencv.hpp>  

#include <vector>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/nonfree/nonfree.hpp>

#include <string.h>*/

#include "boost/filesystem/operations.hpp"
#include "boost/filesystem/path.hpp"
#include "boost/progress.hpp"


namespace fs = boost::filesystem;

static const std::string OPENCV_WINDOW = "Image window";
static const std::string OUT_WINDOW = "Output window";

void print_float_vector(std::vector<float> x){
	for (unsigned int i = 0; i < x.size(); i ++){
		if (i != x.size()-1){
			printf("%f,",x.at(i));
		}
		else printf("%f\n",x.at(i));
	}
}

std::string type2str(int type) {
  std::string r;

  uchar depth = type & CV_MAT_DEPTH_MASK;
  uchar chans = 1 + (type >> CV_CN_SHIFT);

  switch ( depth ) {
    case CV_8U:  r = "8U"; break;
    case CV_8S:  r = "8S"; break;
    case CV_16U: r = "16U"; break;
    case CV_16S: r = "16S"; break;
    case CV_32S: r = "32S"; break;
    case CV_32F: r = "32F"; break;
    case CV_64F: r = "64F"; break;
    default:     r = "User"; break;
  }

  r += "C";
  r += (chans+'0');

  return r;
}

std::vector<cv::Mat> load_images(std::string vision_data_path){
	std::vector<cv::Mat> images_o_b_t;
	
	//read files using boost
	fs::path full_path( fs::initial_path<fs::path>() );
	full_path = fs::system_complete( fs::path( vision_data_path.c_str() ) );
	
	//if it doesn't exist...
	if ( !fs::exists( vision_data_path ) )
	{
		std::cout << "\nNot found: " << full_path.string() << std::endl;
		return images_o_b_t;
	}
	
	if ( fs::is_directory( full_path ) )
	{
		std::cout << "Processing directory...\n";
		
		fs::directory_iterator end_iter;
		for ( fs::directory_iterator dir_itr( full_path );
			dir_itr != end_iter;
			++dir_itr )
		{	
			std::cout << "processing file...\n";
			
			try
			{
				if ( fs::is_regular_file( dir_itr->status() ) )
				{
					//check if it is jpg
					std::string file_full_path = dir_itr->path().string();
					
					if (file_full_path.find(".jpg") != std::string::npos && file_full_path.find(".pcd") == std::string::npos){
						std::cout << dir_itr->path().string() << "\n";
						
						cv::Mat img_i;
						img_i = cv::imread(file_full_path.c_str(), CV_LOAD_IMAGE_COLOR);   // Read the file

						images_o_b_t.push_back(img_i);
					}
				}
			

			}
			catch ( const std::exception & ex )
			{
				
				std::cout << dir_itr->path().filename() << " " << ex.what() << std::endl;
			}
		}
	}
	
	return images_o_b_t;
}

int main(int argc, char** argv)
{
	std::string path = "";
	
	int num_objects = 32;
	int num_trials = 5;
	
	std::string behaviors[] = {"look","grasp","lift","hold","lower","drop","push","press"};
	
	//create window
	cv::namedWindow(OUT_WINDOW);
	
	std::stringstream dataset_path;
	dataset_path << "/home/jsinapov/research/datasets/ordering_data/";
	
	for (int trial_num = 2; trial_num <= num_trials; trial_num++){
		
		for (int object_id = 1; object_id <= 32; object_id++){
			for (int b = 0; b < 8; b++){
				std::string behavior = behaviors[b];
				
				std::stringstream folder_path;
				folder_path << dataset_path.str() << "t" << trial_num << "/obj_" << object_id << "/trial_1/" << behavior << "/vision_data";
				
				//folder for object, trial and behavior
				std::string folder = folder_path.str();
				//"/home/jsinapov/research/datasets/ordering_data/t1/obj_1/trial_1/grasp/vision_data";
				
				//load all images
				std::vector<cv::Mat> images_o_b_t = load_images(folder);
				
				std::cout << "Loaded " << images_o_b_t.size() << " images.\n";
				
				//save to file
				std::stringstream out_file;
				out_file << "obj" << object_id << "_trial" << trial_num << "_" << behavior << ".surf.txt";
					
				FILE *fp = fopen(out_file.str().c_str(), "w");
				
				for (unsigned int k = 0; k < images_o_b_t.size(); k ++){
					cv::Mat outImg = images_o_b_t.at(k);
					
					//set ROI based on behavior
					if (behavior == "grasp" || behavior == "look"){
						cv::Rect roi = cv::Rect(202,154,120,170);
						outImg = outImg(roi);
					}
					else if (behavior == "lift" || behavior == "hold" || behavior == "lower"){
						cv::Rect roi = cv::Rect(202,0,120,240);
						outImg = outImg(roi);
					}
					else if (behavior == "drop"){
						cv::Rect roi = cv::Rect(70,130,250,310);
						outImg = outImg(roi);
					}
					else if (behavior == "push"){
						cv::Rect roi = cv::Rect(100,140,275,250);
						outImg = outImg(roi);
					}
					else if (behavior == "press"){
						cv::Rect roi = cv::Rect(100,160,275,250);
						outImg = outImg(roi);
					}
					
					
					//-- Step 1: Detect the keypoints using SURF Detector
					int minHessian = 400;
					cv::Ptr<cv::xfeatures2d::SURF> detector = cv::xfeatures2d::SURF::create( minHessian );
					
					std::vector<cv::KeyPoint> keypoints_object;
					detector->detect( outImg, keypoints_object );
					
					
					//-- Step 2: Calculate descriptors (feature vectors)
					cv::Mat descriptors;

					detector->compute(outImg, keypoints_object, descriptors);
					
					/*ROS_INFO("rows = %i",descriptors.rows); 	
					ROS_INFO("cols = %i",descriptors.cols); 	
					ROS_INFO("dims = %i",descriptors.dims); 	

					ROS_INFO("Type = %s",type2str( descriptors.type() ).c_str());
					*/
					int feature_vector_dim = (int)descriptors.cols;
					int num_features = (int)descriptors.rows;

					

					//plot points
					for (unsigned int i = 0; i < keypoints_object.size(); i++){
						int x = keypoints_object.at(i).pt.x;
						int y = keypoints_object.at(i).pt.y;
					
						
						cv::circle(outImg, cv::Point(x, y), 5, CV_RGB(255,0,0));
					} 
					
					
					
					//go through each feature
					for (unsigned int i = 0; i < descriptors.rows; i++){
					
						std::vector<float> f_i;
						for (int j = 0; j < feature_vector_dim; j++){
							f_i.push_back(descriptors.at<float>(i,j));
							
							fprintf(fp,"%f",descriptors.at<float>(i,j));
							
							if (j < feature_vector_dim -1)
								fprintf(fp,",");
							else fprintf(fp,"\n");
						}
					
						/*if (i == 0){
							print_float_vector(f_i);
						}*/
					
					}
					
					
					
					//show output
					cv::imshow(OUT_WINDOW, outImg);
				
					//pause for 3 ms
					cv::waitKey(50);
				}
				
				fclose(fp);
			}	
		}
	}
	
	return 0;
}
