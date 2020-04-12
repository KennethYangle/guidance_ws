
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

Mat I1_level3,I1_level2,I1_level1,I2_level3,I2_level2,I2_level1;  
Mat u_l3,u_l2,u_l1,v_l3,v_l2,v_l1;

float x_l3[40],x_l2[80],x_l1[160];
float y_l3[30],y_l2[60],y_l1[120];   

bool co = false;
bool valid = true;

int scale = 1;
int delta = 0;
int ddepth = CV_16S;

//............................START................................... 
Mat orb_features_projective(Mat I1, Mat I2){
		//imshow("I1",I1);
		//imshow("I2",I2);
		//cout<<I1.type()<<endl;
		//Mat I1,I2;
		//cvCvtColor(I11,I1,CV_RGB2GRAY);
		//cvCvtColor(I22,I2,CV_RGB2GRAY);
		//waitKey(3);
	
		//-- Step 1: Detect the keypoints using SURF Detector
		Ptr<ORB> detector = ORB::create();
		vector<KeyPoint> keypoints1, keypoints2;// initialise keypoint vectors
		detector->detect( I1, keypoints1 ); // detect the SURF Feature points in I1
		detector->detect( I2, keypoints2 ); // detect the SURF Feature points in I2

		cv::KeyPointsFilter::retainBest(keypoints1, 100);
		cv::KeyPointsFilter::retainBest(keypoints2, 100);

		//cout << "Features 1 size"<<keypoints1.size()<<endl;
		//cout << "Features 2 size"<<keypoints2.size()<<endl;

		//-- Step 2: Calculate descriptors (feature vectors)
		Ptr<ORB> extractor = ORB::create();// Create a 			SurfDescriptorExtractor Object
		Mat descriptors1, descriptors2;// initialise feature/descriptor vectors
		extractor->compute( I1, keypoints1, descriptors1 );// get features in I1 
		extractor->compute( I2, keypoints2, descriptors2 );// get features in I2

		//-- Step 3: Matching descriptor vectors with a brute force matcher
		BFMatcher matcher(NORM_L2); // Create BFMatcher Object
		vector< DMatch > matches; // initialise a vector 
		matcher.match( descriptors1, descriptors2, matches );// get the matched points

		//-- Draw matches
		//Mat img_matches_all;
		//drawMatches( I1, keypoints1, I2, keypoints2, matches, img_matches_all );
		//-- Show detected matches
		//imshow("Matches", img_matches_all );
		

		//=========== Filtering these matches to reject outliers ===============

		//-- Step1: Quick calculation of max and min distances between keypoints
		double max_dist = 0; double min_dist = 100;// define min and max distance pixels
		for( int i = 0; i < descriptors1.rows; i++ )
			{ double dist = matches[i].distance;
			if( dist < min_dist ) min_dist = dist;
			if( dist > max_dist ) max_dist = dist;
		}


		//-- Step2: Select the good matches
		vector< DMatch > good_matches; // Define a vector to store good matches
		for( int i = 0; i < descriptors1.rows; i++ )
		{ if( matches[i].distance < 3*min_dist )
		{ good_matches.push_back( matches[i]); }
		}


		// Show the good matches if you want
		Mat img_matches;
		drawMatches( I1, keypoints1, I2, keypoints2,
		good_matches, img_matches, Scalar::all(-1), Scalar::all(-1),
		vector<char>(), DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS );
		//-- Show detected matches
		imshow( OPENCV_WINDOW, img_matches );waitKey(1);


		//-- Localize the object
		vector<Point2f> obj;
		vector<Point2f> scene;
		for( int i = 0; i < good_matches.size(); i++ ){
		// -- Get the keypoints from the good matches
			obj.push_back( keypoints1[ good_matches[i].queryIdx ].pt );
			scene.push_back( keypoints2[ good_matches[i].trainIdx ].pt );
		}
		Mat flow = Mat::zeros(3,1,CV_32FC1);
		if(good_matches.size() > 20){
			Mat H1 = findHomography( obj, scene, CV_RANSAC );
			Mat H;H1.convertTo(H, CV_32FC1);
			//Mat pt1 = (Mat_ <float>(3,1)<<160,120,1);
			Mat pt = (Mat_ <float>(3,1)<<160,120,1);
			flow = H*pt - pt;
			flow.at<float>(2,0)= valid;	
			//cout << H<<endl;
			//cout << H1<<endl;
			//cout<<pt1<<endl<<pt<<endl;
		}
		//else{
			//cout<<"Not good"<<endl;	
		//}
return flow;
}
//.............................END.........................................
// Call Back Function
void imageCb(const sensor_msgs::ImageConstPtr& msg){
	if(!co ){
		co = true;
		//ROS_INFO_STREAM(" Alhamdulilah" );  
		try{
       		cv_ptr_I1 = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::MONO8);
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
		Mat flow_orb = orb_features_projective(cv_ptr_I1->image, cv_ptr_I2->image);
		float valid = flow_orb.at<float>(2,0);
		if(valid == 1)
		{
		tform_msg.x = flow_orb.at<float>(0,0);
		tform_msg.y = flow_orb.at<float>(1,0);
		tform_msg.z = float(1.31);
		cout<< "orb"<<endl;
		cout << flow_orb<<endl;
		}
		else
		{
		tform_msg.x = 0;
		tform_msg.y = 0;
		tform_msg.z = float(2.1);
		cout<< "not found"<<endl;
		}
		tform_pub.publish(tform_msg);
		cv_ptr_I2->image.copyTo(cv_ptr_I1->image);

	}

};
 
int main(int argc, char** argv)
 {
   	ros::init(argc, argv, "orb_pyramids");

//create a ros node handle
	ros::NodeHandle nh;
	cv::namedWindow(OPENCV_WINDOW);
	//cv::namedWindow(I2_WINDOW);
// Create a subscriber object
	ros::Subscriber sub = nh.subscribe( "guidance/left_image_drop" , 3 , &imageCb);//guidance/left_image
// publish the constant velocities on turtle2/cmd_vel
	tform_pub = nh.advertise<geometry_msgs::Point> ( "rfly/orbpyramids", 3);
   	ros::spin();
// rosrun topic_tools drop /guidance/left_image 1 2
 }
