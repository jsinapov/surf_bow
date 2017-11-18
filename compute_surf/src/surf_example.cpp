#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/opencv.hpp>  

#include <vector>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/nonfree/nonfree.hpp>

#include <string.h>

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

class ImageConverter
{
  ros::NodeHandle nh_;
  image_transport::ImageTransport it_;
  image_transport::Subscriber image_sub_;
  image_transport::Publisher image_pub_;
  
public:
  ImageConverter()
    : it_(nh_)
  {
    // Subscrive to input video feed and publish output video feed
    image_sub_ = it_.subscribe("/usb_cam/image_raw", 1, 
      &ImageConverter::imageCb, this);
    image_pub_ = it_.advertise("/image_converter/output_video", 1);

    cv::namedWindow(OPENCV_WINDOW);
  }

  ~ImageConverter()
  {
    cv::destroyWindow(OPENCV_WINDOW);
  }
  


  void imageCb(const sensor_msgs::ImageConstPtr& msg)
  {
    cv_bridge::CvImagePtr cv_ptr;
    try
    {
      cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception& e)
    {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }
    
    
    cv::Mat outImg = cv_ptr->image.clone();
    
    //-- Step 1: Detect the keypoints using SURF Detector
	int minHessian = 400;
	cv::SurfFeatureDetector detector( minHessian );

	std::vector<cv::KeyPoint> keypoints_object;

	detector.detect(outImg, keypoints_object );
   
    //-- Step 2: Calculate descriptors (feature vectors)
	cv::SurfDescriptorExtractor extractor;
	cv::Mat descriptors;

	
	extractor.compute( outImg, keypoints_object, descriptors );


	ROS_INFO("rows = %i",descriptors.rows); 	
	ROS_INFO("cols = %i",descriptors.cols); 	
	ROS_INFO("dims = %i",descriptors.dims); 	

	ROS_INFO("Type = %s",type2str( descriptors.type() ).c_str());

	int feature_vector_dim = (int)descriptors.cols;
	int num_features = (int)descriptors.rows;

	//go through each feature
	for (unsigned int i = 0; i < descriptors.rows; i++){
		
		std::vector<float> f_i;
		for (int j = 0; j < feature_vector_dim; j++){
			f_i.push_back(descriptors.at<float>(i,j));
		}
		
		if (i == 0){
			print_float_vector(f_i);
		}
		
	}

	//plot points
	for (unsigned int i = 0; i < keypoints_object.size(); i++){
		int x = keypoints_object.at(i).pt.x;
		int y = keypoints_object.at(i).pt.y;
		
		if (i == 0){
			
		}
		
		cv::circle(outImg, cv::Point(x, y), 5, CV_RGB(255,0,0));
	} 

    
	//cv::Mat original_image = cv_ptr->image;
	
    // Example 1: Draw an example circle on the video stream
    /*outImg = cv_ptr->image.clone();   
    if (cv_ptr->image.rows > 60 && cv_ptr->image.cols > 60)
		cv::circle(outImg, cv::Point(50, 50), 10, CV_RGB(255,0,0));
*/
	//Example 2: make a grayscale of the image
    //cv::cvtColor(cv_ptr->image, outImg, CV_BGR2GRAY); 

	//Example 2.1: thresholding 
	/*cv::cvtColor(cv_ptr->image, outImg, CV_BGR2GRAY); 
	int T = 150;
	for (unsigned int i = 0; i < outImg.rows; i ++){
		for (unsigned int j = 0; j < outImg.cols; j ++){
			if ( outImg.at<uchar>(i,j) < T)
				outImg.at<uchar>(i,j) = 0;
			else 
				outImg.at<uchar>(i,j) = 255;
		}
	}*/
	
	//Example 2.2: thresholding using openCV
	//reference: http://docs.opencv.org/2.4/doc/tutorials/imgproc/threshold/threshold.html
	//cv::cvtColor(cv_ptr->image, outImg, CV_BGR2GRAY);
	//cv::threshold(outImg, outImg, 150, 255, 1);

	//Example 3: remove either the B, the G, or the R
	//note that the order of color channels is BGR by default, not RGB
	/*outImg = cv_ptr->image.clone(); 
	for (unsigned int i = 0; i < outImg.rows; i ++){
		for (unsigned int j = 0; j < outImg.cols; j ++){
			outImg.at<cv::Vec3b>(i,j)[2] = 0;  //set blue to 0
		
			//example of how to look up pixel values
			int b_ij = (int)outImg.at<cv::Vec3b>(i,j)[0];
			int g_ij = (int)outImg.at<cv::Vec3b>(i,j)[1];
			int r_ij = (int)outImg.at<cv::Vec3b>(i,j)[2];
			
			
		}
	}*/

	//Example 4: smoothing and image difference
	/*outImg = cv_ptr->image.clone(); 
	cv::GaussianBlur( outImg, outImg, cv::Size( 55, 55 ), 0, 0 );
	
	for (unsigned int i = 0; i < outImg.rows; i ++){
		for (unsigned int j = 0; j < outImg.cols; j ++){
			//example of how to look up pixel values
			int b_ij = (int)outImg.at<cv::Vec3b>(i,j)[0];
			int g_ij = (int)outImg.at<cv::Vec3b>(i,j)[1];
			int r_ij = (int)outImg.at<cv::Vec3b>(i,j)[2];
			
			//example of how to look up pixel values
			int b_ij2 = (int)cv_ptr->image.at<cv::Vec3b>(i,j)[0];
			int g_ij2 = (int)cv_ptr->image.at<cv::Vec3b>(i,j)[1];
			int r_ij2 = (int)cv_ptr->image.at<cv::Vec3b>(i,j)[2];
			
			outImg.at<cv::Vec3b>(i,j)[0] = b_ij2 - b_ij;  
			outImg.at<cv::Vec3b>(i,j)[1] = g_ij2 - g_ij;  
			outImg.at<cv::Vec3b>(i,j)[2] = r_ij2 - r_ij;  
		
			
		}
	}*/
	
	
	//Example 5: edge detection
	/*cv::cvtColor(cv_ptr->image, outImg, CV_BGR2GRAY);
	int edge_threshold = 50; //between 0 and 100
	int edge_canny_ratio = 3;
	int edge_kernel_size = 3;
	cv::GaussianBlur(outImg, outImg, cv::Size( 7, 7 ), 0, 0 );
	
	cv::Canny(outImg, outImg, edge_threshold, edge_threshold*edge_canny_ratio, edge_kernel_size ); 	*/


	//show input
    cv::imshow(OPENCV_WINDOW, cv_ptr->image);
    
	//show output
	cv::imshow(OUT_WINDOW, outImg);
    
	//pause for 3 ms
    cv::waitKey(3);
    
    // Output modified video stream
    image_pub_.publish(cv_ptr->toImageMsg());
  }
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "image_converter");
  ImageConverter ic;
  ros::spin();
  return 0;
}
