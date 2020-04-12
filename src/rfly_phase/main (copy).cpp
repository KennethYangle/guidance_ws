
// include ROS Libraries
#include <ros/ros.h>

//#include <image_transport/image_transport.h>// image trasport is used for publishing and subscribing an image
#include <cv_bridge/cv_bridge.h> // bridge between ROS and OpenCV
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/Image.h>
#include <geometry_msgs/Point.h>

// OpenCV Libraries
#include <opencv2/opencv.hpp>
#include "opencv2/core/core.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/features2d/features2d.hpp"

// Standard C++ Headers
#include "stdio.h"
#include <iostream>

using namespace std;
using namespace cv; 

static const string OPENCV_WINDOW = "Image window1";
//static const string I2_WINDOW = "Image window2";

// Create a publisher object
ros::Publisher tform_pub;
geometry_msgs::Point tform_msg;

// Create pointer object to convert from Opencv to ROS and Vice Versa
cv_bridge::CvImagePtr cv_ptr_I1;
cv_bridge::CvImagePtr cv_ptr_I2;

// Variables for Phase Correlation
Mat hann,I164f,I264f;
Point2d shift;
Size s;
bool co = false;
double sec,nsec,time1,time2,delt;
//END............................

//............................START Phase Correlation Method................................... 

Mat correlation_flow(Mat I1, Mat I2, Mat hann){
		I1.convertTo(I164f, CV_64F);
		I2.convertTo(I264f, CV_64F);
		shift = phaseCorrelateRes(I164f, I264f, hann);
		Mat flow = Mat::zeros(3,1,CV_32FC1);
		
		if(abs(shift.x)>100 || abs(shift.y)>100){
			flow.at<float>(0,0) = 0;
			flow.at<float>(1,0) = 0;
			flow.at<float>(2,0) = 0;
		}
		else{
			flow.at<float>(0,0) = shift.x;
			flow.at<float>(1,0) = shift.y;
			flow.at<float>(2,0) = 1;
		}

		return flow;
	}

//.............................END Phase Correlation Method.........................................
// ****************************Call Back Function***************************************************
void imageCb(const sensor_msgs::ImageConstPtr& msg){
	if(!co ){
		co = true;
		//ROS_INFO_STREAM(" Alhamdulilah" );  
		try{
       		cv_ptr_I1 = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::MONO8);
		
		std_msgs::Header h = msg->header;
		sec = h.stamp.sec;
		nsec = h.stamp.nsec;
		time1 = sec + nsec/1000000000;
		}
	
		catch (cv_bridge::Exception& e){
		ROS_ERROR("cv_bridge exception: %s", e.what());
		return;}
	}
	
	else{
	
		//ROS_INFO_STREAM(" ALLAH hu AKBAR" );
		try{
       		cv_ptr_I2 = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::MONO8);
		}
	
		catch (cv_bridge::Exception& e){
		ROS_ERROR("cv_bridge exception: %s", e.what());
		return;}

		std_msgs::Header h = msg->header;
		sec = h.stamp.sec;
		nsec = h.stamp.nsec;
		time2 = sec + nsec/1000000000;
		delt = time2-time1;
		cout<<"delt "<<delt<<endl;

		Mat flow_co = correlation_flow(cv_ptr_I1->image, cv_ptr_I2->image, hann);

		tform_msg.x = 1.3*(flow_co.at<float>(0,0));

		tform_msg.y = 1.3*(flow_co.at<float>(1,0));

		tform_msg.z = delt;

		cout<< "X_flow "<<tform_msg.x<<"Y_flow "<<tform_msg.y<<endl;
		tform_pub.publish(tform_msg);
		cv_ptr_I2->image.copyTo(cv_ptr_I1->image);
		time1 = time2;

	}

};
// ****************************END Call Back Function********************************************* 
// ++++++++++++++++++++++++++++Start Main Function++++++++++++++++++++++++++++++++++++++++++++++++ 
int main(int argc, char** argv)
 {	
	s.height = 240;
	s.width = 320;
	createHanningWindow(hann, s, CV_64F);

	ros::init(argc, argv, "rfly_phase");
//create a ros node handle
	ros::NodeHandle nh;
	cv::namedWindow(OPENCV_WINDOW);
	//cv::namedWindow(I2_WINDOW);
// Create a subscriber object
	ros::Subscriber sub = nh.subscribe( "guidance/left_image" , 3 , &imageCb);//guidance/left_image
// publish the constant velocities on turtle2/cmd_vel
	tform_pub = nh.advertise<geometry_msgs::Point> ( "rfly/phase", 10);
   	ros::spin();
// rosrun topic_tools drop /guidance/left_image 1 2
 }
// ++++++++++++++++++++++++++++END Main Function++++++++++++++++++++++++++++++++++++++++++++++++ 
