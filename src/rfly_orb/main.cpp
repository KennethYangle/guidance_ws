
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

const int orb_th = 10;
static const string OPENCV_WINDOW = "Image window1";
//static const string I2_WINDOW = "Image window2";

// Create a publisher object
ros::Publisher tform_pub;
geometry_msgs::Point tform_msg;

// Create pointer object to convert from Opencv to ROS and Vice Versa
cv_bridge::CvImagePtr cv_ptr_I1;
cv_bridge::CvImagePtr cv_ptr_I2;

bool co = false;
float valid = 1;
double sec,nsec,time1,time2,delt;

int scale = 1;
int delta = 0;
int ddepth = CV_16S;

//............................START................................... 

Mat orb_features_projective(Mat I1, Mat I2){

		Mat flow = Mat::zeros(3,1,CV_32FC1);
		Mat x_flow;
		Mat y_flow;		
	
		//-- Step 1: Detect the keypoints using SURF Detector
		Ptr<ORB> detector = ORB::create(100,1.2f,8,10,0,2,ORB::HARRIS_SCORE,31,20);


		vector<KeyPoint> keypoints1, keypoints2;// initialise keypoint vectors
		detector->detect( I1, keypoints1 ); // detect the SURF Feature points in I1
		detector->detect( I2, keypoints2 ); // detect the SURF Feature points in I2

		//****** If Else Statement 1 Start Here*****************

		cv::KeyPointsFilter::retainBest(keypoints1, 200);
		cv::KeyPointsFilter::retainBest(keypoints2, 200);
		//****** If Else Statement 1 Ends Here*****************

		//****** If Else Statement 2 Start Here*****************
		if(keypoints1.size()>orb_th && keypoints2.size()>orb_th){

			Ptr<ORB> extractor = ORB::create();// Create a 			SurfDescriptorExtractor Object
			Mat descriptors1, descriptors2;// initialise feature/descriptor vectors
			extractor->compute( I1, keypoints1, descriptors1 );// get features in I1 
			extractor->compute( I2, keypoints2, descriptors2 );// get features in I2
			
			//-- Step 3: Matching descriptor vectors with a brute force matcher
			BFMatcher matcher(NORM_L2); // Create BFMatcher Object
			vector< DMatch > matches; // initialise a vector 
			try{
			matcher.match( descriptors1, descriptors2, matches );// get the matched points
			}
			catch (Exception& e){
			cout<<"Matching couldnot be performed "<<endl;
			return flow;}

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


			// // Show the good matches if you want
			// Mat img_matches;
			// drawMatches( I1, keypoints1, I2, keypoints2,
			// 	good_matches, img_matches, Scalar::all(-1), Scalar::all(-1),
			// 	vector<char>(), DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS );
			// //-- Show detected matches
			// imshow( OPENCV_WINDOW, img_matches );waitKey(1);


			//-- Localize the object
			vector<Point2f> obj;
			vector<Point2f> scene;
			for( int i = 0; i < good_matches.size(); i++ ){
			// -- Get the keypoints from the good matches
			obj.push_back( keypoints1[ good_matches[i].queryIdx ].pt );
			scene.push_back( keypoints2[ good_matches[i].trainIdx ].pt );
			}
			//****** If Else Statement 3 STARTS Here*****************
			if(good_matches.size() > orb_th){
			Mat mask;
			try{
       			Mat H1 = findHomography( obj, scene, mask, CV_RANSAC,5 );
				}
			catch (Exception& e){
			cout<<"Cannot Find Homography"<<endl;
			return flow;}
	
			for( uchar i = 0; i < mask.rows; i++ ){
				if ((int)mask.at<uchar>(0,i)==1){
				x_flow.push_back( scene.at(i).x - obj.at(i).x);
				y_flow.push_back( scene.at(i).y - obj.at(i).y);
				}
			}
			//cout<<x_flow<<endl;
			Mat x_flow_s,y_flow_s;
			cv::sort(x_flow, x_flow_s, CV_SORT_EVERY_COLUMN);
			cv::sort(y_flow, y_flow_s, CV_SORT_EVERY_COLUMN);
			//cout<<x_flow_s<<endl;
			float u = x_flow_s.at<float>((x_flow.rows/2),0);
			float v = y_flow_s.at<float>((y_flow.rows/2),0);
			flow.at<float>(0,0) = u;
			flow.at<float>(1,0) = v;
			flow.at<float>(2,0) = 1;
			}

			else{
			cout<<"Good matches are less than 40"<<endl;	
			}
			//****** If Else Statement 3 ENDS Here*****************
		}
		else{
			cout<<"Key Points are less than 100"<<endl;	
		}
		//****** If Else Statement 2 ENDS Here*****************

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

		Mat flow_orb = orb_features_projective(cv_ptr_I1->image, cv_ptr_I2->image);
		float check = flow_orb.at<float>(2,0);
		if(check == 1)
		{
		tform_msg.x = 1*(flow_orb.at<float>(0,0));
		tform_msg.y = 1*(flow_orb.at<float>(1,0));
		tform_msg.z = delt;
		}
		else
		{
		tform_msg.x = 0;
		tform_msg.y = 0;
		tform_msg.z = 0;
		}
		
		tform_pub.publish(tform_msg);
		cv_ptr_I2->image.copyTo(cv_ptr_I1->image);
		time1 = time2;

	}

};
 
int main(int argc, char** argv)
 {

   	ros::init(argc, argv, "rfly_orb");

//create a ros node handle
	ros::NodeHandle nh;
	//cv::namedWindow(OPENCV_WINDOW);
// Create a subscriber object
	ros::Subscriber sub = nh.subscribe( "guidance/left_image" , 1 , &imageCb);//guidance/left_image
// publish the constant velocities on turtle2/cmd_vel
	tform_pub = nh.advertise<geometry_msgs::Point> ( "rfly/orb", 1);
   	ros::spin();
// rosrun topic_tools drop /guidance/left_image 1 2
 }
