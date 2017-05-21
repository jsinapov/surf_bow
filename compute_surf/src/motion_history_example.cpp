#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/opencv.hpp>  

static const std::string OPENCV_WINDOW = "Image window";
static const std::string OUT_WINDOW = "Output window";

// various tracking parameters (in seconds)
const double MHI_DURATION = 1;
const double MAX_TIME_DELTA = 2.5;
const double MIN_TIME_DELTA = 1.1;
// number of cyclic frame buffer used for motion detection
// (should, probably, depend on FPS)
const int N = 4;

const int mhi_threshold = 30;

class MotionHistoryCalc
{
	ros::NodeHandle nh_;
	image_transport::ImageTransport it_;
	image_transport::Subscriber image_sub_;
	image_transport::Publisher image_pub_;
  
	// ring image buffer
	std::vector<cv::Mat> buf;
	int last;
	
	// where we story the motion history image
	cv::Mat mhi;
  
public:
	MotionHistoryCalc()
		: it_(nh_)
	{
		// Subscrive to input video feed and publish output video feed
		image_sub_ = it_.subscribe("/nav_kinect/rgb/image_color", 1, 
			&MotionHistoryCalc::imageCb, this);
		image_pub_ = it_.advertise("/image_converter/output_video", 1);

		last = 0;
		buf.resize(N);

		cv::namedWindow(OPENCV_WINDOW);
		cv::namedWindow(OUT_WINDOW);
	}

	~MotionHistoryCalc()
	{
		cv::destroyWindow(OPENCV_WINDOW);
		cv::destroyWindow(OUT_WINDOW);
	}
  
	void update_mhi( const cv::Mat& img, cv::Mat& dst, int diff_threshold )
	{
		double timestamp = (double)clock()/CLOCKS_PER_SEC; // get current time in seconds
		int idx1 = last, idx2;
		cv::Mat silh, orient, mask, segmask;

		cv::cvtColor( img, buf[last], CV_BGR2GRAY ); // convert frame to grayscale

		idx2 = (last + 1) % N; // index of (last - (N-1))th frame
		last = idx2;

		if( buf[idx1].size() != buf[idx2].size() )
			silh = cv::Mat::ones(img.size(), CV_8U)*255;
		else
			absdiff(buf[idx1], buf[idx2], silh); // get difference between frames
		
		cv::threshold( silh, silh, diff_threshold, 1, CV_THRESH_BINARY ); // and threshold it
		if( mhi.empty() )
			mhi = cv::Mat::zeros(silh.size(), CV_32F);
		
		cv::updateMotionHistory( silh, mhi, timestamp, MHI_DURATION ); // update MHI

		// convert MHI to blue 8u image
		mhi.convertTo(mask, CV_8U, 255./MHI_DURATION,
					(MHI_DURATION - timestamp)*255./MHI_DURATION);
		dst = cv::Mat::zeros(mask.size(), CV_8UC3);
		cv::insertChannel(mask, dst, 0);

		// calculate motion gradient orientation and valid orientation mask
		cv::calcMotionGradient( mhi, mask, orient, MAX_TIME_DELTA, MIN_TIME_DELTA, 3 );
		
		// segment motion: get sequence of motion components
		// segmask is marked motion components map. It is not used further
		std::vector<cv::Rect> brects;
		cv::segmentMotion(mhi, segmask, brects, timestamp, MAX_TIME_DELTA );

		// iterate through the motion components,
		// One more iteration (i == -1) corresponds to the whole image (global motion)
		for( int i = -1; i < (int)brects.size(); i++ ) {
			cv::Rect roi; cv::Scalar color; double magnitude;
			cv::Mat maski = mask;
			if( i < 0 ) { // case of the whole image
				roi = cv::Rect(0, 0, img.cols, img.rows);
				color = cv::Scalar::all(255);
				magnitude = 100;
			}
			else { // i-th motion component
				roi = brects[i];
				if( roi.area() < 3000 ) // reject very small components
					continue;
				color = cv::Scalar(0, 0, 255);
				magnitude = 30;
				maski = mask(roi);
			}
			
			// calculate orientation
			double angle = cv::calcGlobalOrientation( orient(roi), maski, mhi(roi), timestamp, MHI_DURATION);
			angle = 360.0 - angle;  // adjust for images with top-left origin

			int count = cv::norm( silh, cv::NORM_L1 ); // calculate number of points within silhouette ROI
			// check for the case of little motion
			if( count < roi.area() * 0.05 )
				continue;

			// draw a clock with arrow indicating the direction
			cv::Point center( roi.x + roi.width/2, roi.y + roi.height/2 );
			cv::circle( dst, center, cvRound(magnitude*1.2), color, 3, CV_AA, 0 );
			cv::line( dst, center, cv::Point( cvRound( center.x + magnitude*cos(angle*CV_PI/180)),
					cvRound( center.y - magnitude*sin(angle*CV_PI/180))), color, 3, CV_AA, 0 );
		}
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
    
		cv::Mat motion;
	
		update_mhi( cv_ptr->image, motion, mhi_threshold );
		
		//show input
		cv::imshow(OPENCV_WINDOW, cv_ptr->image);
    
		//show output
		cv::imshow(OUT_WINDOW, motion);
    
		//pause for 3 ms
		cv::waitKey(3);
    
		// Output modified video stream
		image_pub_.publish(cv_ptr->toImageMsg());
	}
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "image_converter");
  MotionHistoryCalc ic;
  ros::spin();
  return 0;
}
