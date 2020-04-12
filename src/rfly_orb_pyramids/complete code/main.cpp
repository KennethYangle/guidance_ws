
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

static const string I1_WINDOW = "Image window1";
static const string I2_WINDOW = "Image window2";

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
//The following function calculates affine transform for any two images
cv::Mat aff(Mat I1, Mat I2, Mat u, Mat v, int a ){
	
	int top = a; int bottom = a;
	int left = a; int right = a;
	Mat image_const = Mat::ones(I1.rows-2*a,I1.cols-2*a,CV_32FC1);
	//Mat window = Mat::ones(I1.rows,I1.cols,CV_32FC1) ;
	Mat window;
	copyMakeBorder( image_const, window, top, bottom, left, right, BORDER_CONSTANT, 0 );
		// Generate grad_x and grad_y
		Mat It, It1, grad_x, grad_y,abs_grad_x, abs_grad_y,gxx,gyy, gx,gy;
		
		// Gradient X
  		Sobel( I1/8, grad_x, ddepth, 1, 0, 3, scale, delta, BORDER_DEFAULT );
		
	
		// Gradient Y
		Sobel( I1/8, grad_y, ddepth, 0, 1, 3, scale, delta, BORDER_DEFAULT );

		grad_x.convertTo(gxx, CV_32FC1);
		grad_y.convertTo(gyy, CV_32FC1);
		
		gx = window.mul(gxx);
		gy = window.mul(gyy);

		Mat I12t = -1*(I2-I1);
		I12t.convertTo(It1,CV_32FC1);
		It = window.mul(It1);	
		
		Mat P = gx.mul(u);
		Mat Q = gx.mul(v);

		Mat R = gy.mul(u);
		Mat S = gy.mul(v);
		
		Mat A1;
		A1.push_back(gx.reshape(1,gx.rows*gx.cols));
		A1.push_back(P.reshape(1,P.rows*P.cols));
		A1.push_back(Q.reshape(1,Q.rows*Q.cols));
		A1.push_back(gy.reshape(1,gy.rows*gy.cols));
		A1.push_back(R.reshape(1,R.rows*R.cols));
		A1.push_back(S.reshape(1,S.rows*S.cols));
		Mat A2 = A1.reshape(1,6);
		Mat A; transpose(A2,A);

		Mat B = It.reshape(1,It.rows*It.cols);
		Mat At; transpose(A,At);
		Mat Bt; transpose(B,Bt);
		Mat C1; invert(At*A,C1);
		Mat aff = C1*At*B;

		cv::FileStorage storage1("A.yml", cv::FileStorage::WRITE);
		storage1 << "A" << A;
		storage1.release(); 
		cv::FileStorage storage2("B.yml", cv::FileStorage::WRITE);
		storage2 << "B" << B;
		storage2.release();


		Mat t_form = Mat::zeros(2,3,CV_32FC1);	
		
		t_form.at<float>(0,0)=1+aff.at<float>(1,0);
		t_form.at<float>(0,1)=aff.at<float>(2,0);
		t_form.at<float>(0,2)=aff.at<float>(0,0);

		t_form.at<float>(1,0)=aff.at<float>(4,0);
		t_form.at<float>(1,1)=1+aff.at<float>(5,0);
		t_form.at<float>(1,2)=aff.at<float>(3,0);
	

return t_form;
}
//............................END..........................................

//............................START............................................
// The following function warps the image I by tform matrix
Mat image_warping(Mat I, Mat tform){
		
   		/// Set the dst image the same type and size as src
   		Mat dst = Mat::zeros( I.rows, I.cols, I.type() );
		/// Apply the Affine Transform just found to the src image
   		warpAffine( I, dst, tform, dst.size() );
		return dst;
}
//..............................END.........................................

//..............................START.........................................
Mat merger(Mat tform1, Mat tform2){
	
	Mat tform = Mat::zeros(2,3,CV_32FC1);	
		
	tform.at<float>(0,0)=(tform1.at<float>(0,0)+tform2.at<float>(0,0))/2;
	tform.at<float>(0,1)=(tform1.at<float>(0,1)+tform2.at<float>(0,1))/2;	
	tform.at<float>(0,2)=(tform1.at<float>(0,2)+tform2.at<float>(0,2));		

	tform.at<float>(1,0)=(tform1.at<float>(1,0)+tform2.at<float>(1,0))/2;
	tform.at<float>(1,1)=(tform1.at<float>(1,1)+tform2.at<float>(1,1))/2;	
	tform.at<float>(1,2)=(tform1.at<float>(1,2)+tform2.at<float>(1,2));

return tform;	
}
//..............................END...........................................

//..............................START.........................................
//This is the main function that uses above defined functions to calculate the optical flow
Mat flow_calc(Mat I1, Mat I2){
		for(int i=0;i<40;i++) x_l3[i] = i+1;
		for(int i=0;i<80;i++) x_l2[i] = i+1;
		for(int i=0;i<160;i++) x_l1[i] = i+1;
	
		Mat A_l3 = Mat(1, 40, CV_32F,x_l3);		
		cv::repeat(A_l3, 30, 1, u_l3);

		Mat A_l2 = Mat(1, 80, CV_32F,x_l2);		
		cv::repeat(A_l2, 60, 1, u_l2);

		Mat A_l1 = Mat(1, 160, CV_32F,x_l1);		
		cv::repeat(A_l1, 120, 1, u_l1);

// -- Creating v matrix for L1, L2,L3	
		for(int i=0;i<30;i++) y_l3[i] = i+1;
		for(int i=0;i<60;i++) y_l2[i] = i+1;
		for(int i=0;i<120;i++) y_l1[i] = i+1;

		Mat B_l3 = Mat(30, 1, CV_32F,y_l3);		
		cv::repeat(B_l3, 1, 40, v_l3);

		Mat B_l2 = Mat(60, 1, CV_32F,y_l2);		
		cv::repeat(B_l2, 1, 80, v_l2);

		Mat B_l1 = Mat(120, 1, CV_32F,y_l1);		
		cv::repeat(B_l1, 1, 160, v_l1); 
				
		pyrDown( I1, I1_level1, Size( 160, 120 ));
		pyrDown( I1_level1, I1_level2, Size( I1_level1.cols/2, I1_level1.rows/2));
		pyrDown( I1_level2, I1_level3, Size( I1_level2.cols/2, I1_level2.rows/2)); 


		pyrDown( I2, I2_level1, Size( I2.cols/2, I1.rows/2 ));
		pyrDown( I2_level1, I2_level2, Size( I2_level1.cols/2, I2_level1.rows/2));
		pyrDown( I2_level2, I2_level3, Size( I2_level2.cols/2, I2_level2.rows/2));
		
		float a1,a2,a3,b1,b2,b3;
//------------------------Processing Level 3--------------------------------
		
		Mat t_form3 = aff(I1_level3, I2_level3, u_l3, v_l3,5);
		//Mat t_form3_1 = aff(I1_level3, I2_level3, u_l3, v_l3,5);
		//Mat I1_level3_1 = image_warping(I1_level3, t_form3_1);
		//Mat t_form3_2 = aff(I1_level3_1, I2_level3, u_l3, v_l3,5);
		//Mat t_form3 =  merger(t_form3_1, t_form3_2);
//------------------------Processing Level 2--------------------------------
		Mat t_form3_ini;
		t_form3.copyTo(t_form3_ini);
		a3 = t_form3.at<float>(0,2);
		b3 = t_form3.at<float>(1,2);		
		
		t_form3_ini.at<float>(0,2)=2*a3;
		t_form3_ini.at<float>(1,2)=2*b3;
		
		Mat I1_level2_ini = image_warping(I1_level2, t_form3_ini);
		
		Mat t_form2 = aff(I1_level2_ini, I2_level2, u_l2, v_l2,10);

//------------------------Processing Level 1--------------------------------
		//Mat t_form2_ini;
		//t_form2.copyTo(t_form2_ini);

		//a2 = t_form2.at<float>(0,2);
		//b2 = t_form2.at<float>(1,2);		
		//t_form2_ini.at<float>(0,2)=4*a3+2*a2;
		//t_form2_ini.at<float>(1,2)=4*b3+2*b2;
		//Mat I1_level1_ini = image_warping(I1_level1, t_form2_ini);
		
		//Mat t_form1 = aff(I1_level1_ini, I2_level1, u_l1, v_l1,20);

//------------------------Finalizing----------------------------------------
		Mat a,b;
		t_form3.copyTo(a);
		t_form2.copyTo(b);
		
	
		Mat tform = (a+b)/2;

	
		tform.at<float>(0,2)=(8*a3+4*a2);			
		tform.at<float>(1,2)=(8*b3+4*b2);

		Mat pt1 = (Mat_ <float>(3,1)<<160,120,1);
		Mat pt = (Mat_ <float>(2,1)<<160,120);
		Mat flow = tform*pt1 - pt;
return flow;
}
//.............................END.........................................

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
		//drawMatches( I1, keypoints1, I2, keypoints2,
		//good_matches, img_matches, Scalar::all(-1), Scalar::all(-1),
		//vector<char>(), DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS );
		//-- Show detected matches
		//imshow( OPENCV_WINDOW, img_matches );


		//-- Localize the object
		vector<Point2f> obj;
		vector<Point2f> scene;
		for( int i = 0; i < good_matches.size(); i++ ){
		// -- Get the keypoints from the good matches
			obj.push_back( keypoints1[ good_matches[i].queryIdx ].pt );
			scene.push_back( keypoints2[ good_matches[i].trainIdx ].pt );
		}
		Mat flow = Mat::zeros(3,1,CV_32FC1);
		if(good_matches.size() > 16){
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
		ROS_INFO_STREAM(" Alhamdulilah" );  
		try{
       		cv_ptr_I1 = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::MONO8);
		}
	
		catch (cv_bridge::Exception& e){
		ROS_ERROR("cv_bridge exception: %s", e.what());
		return;}
	}
	
	else{
	
		ROS_INFO_STREAM(" ALLAH hu AKBAR" );
		try{
       		cv_ptr_I2 = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::MONO8);
		}
	
		catch (cv_bridge::Exception& e){
		ROS_ERROR("cv_bridge exception: %s", e.what());
		return;}
		Mat flow_pyramids = flow_calc(cv_ptr_I1->image, cv_ptr_I2->image);
		Mat flow_orb = orb_features_projective(cv_ptr_I1->image, cv_ptr_I2->image);
		float valid = flow_orb.at<float>(2,0);
		if(valid == 1)
		{
		tform_msg.x = flow_orb.at<float>(0,0);
		tform_msg.y = flow_orb.at<float>(1,0);
		cout<< "orb"<<endl;
		cout << flow_orb<<endl;
		}
		else
		{
		tform_msg.x = flow_pyramids.at<float>(0,0);
		tform_msg.y = flow_pyramids.at<float>(1,0);
		cout<< "pyramids"<<endl;
		cout << flow_pyramids<<endl;
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
	//cv::namedWindow(I1_WINDOW);
	//cv::namedWindow(I2_WINDOW);
// Create a subscriber object
	ros::Subscriber sub = nh.subscribe( "guidance/left_image" , 10 , &imageCb);//guidance/left_image
// publish the constant velocities on turtle2/cmd_vel
	tform_pub = nh.advertise<geometry_msgs::Point> ( "rfly/orbpyramids", 10);
   	ros::spin();
 }
