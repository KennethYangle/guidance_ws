// OpenCV Libraries
#include <opencv2/opencv.hpp>
//#include "opencv2/core/core.hpp"
//#include "opencv2/highgui/highgui.hpp"
//#include "opencv2/features2d/features2d.hpp"
//#include "opencv2/calib3d/calib3d.hpp"

#include <iostream>
#include <stdio.h>
#include <string>
#include <sstream>

using namespace std;
using namespace cv; 

static const string OPENCV_WINDOW = "Good Matches";

bool co = false;

double medianMat(Mat Input, int nVals){
// COMPUTE HISTOGRAM OF SINGLE CHANNEL MATRIX
float range[] = { 0, nVals };
const float* histRange = { range };
bool uniform = true; bool accumulate = false;
cv::Mat hist;
calcHist(&Input, 1, 0, cv::Mat(), hist, 1, &nVals, &histRange, uniform, accumulate);

// COMPUTE CUMULATIVE DISTRIBUTION FUNCTION (CDF)
cv::Mat cdf;
hist.copyTo(cdf);
for (int i = 1; i <= nVals-1; i++){
    cdf.at<float>(i) += cdf.at<float>(i - 1);
}
cdf /= Input.total();

// COMPUTE MEDIAN
double medianVal;
for (int i = 0; i <= nVals-1; i++){
    if (cdf.at<float>(i) >= 0.5) { medianVal = i;  break; }
}
return medianVal/nVals; 
}


Mat orb_features_projective(Mat I1, Mat I2){
		
	
		//-- Step 1: Detect the keypoints using SURF Detector
		Ptr<ORB> detector = ORB::create();
		vector<KeyPoint> keypoints1, keypoints2;// initialise keypoint vectors
		detector->detect( I1, keypoints1 ); // detect the SURF Feature points in I1
		detector->detect( I2, keypoints2 ); // detect the SURF Feature points in I2
		cv::KeyPointsFilter::retainBest(keypoints1, 100);
		cv::KeyPointsFilter::retainBest(keypoints2, 100);

		cout << "Features 1 size"<<keypoints1.size()<<endl;
		cout << "Features 2 size"<<keypoints2.size()<<endl;
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
		imshow( OPENCV_WINDOW, img_matches );


		//-- Localize the object
		vector<Point2f> obj;
		vector<Point2f> scene;
		for( int i = 0; i < matches.size(); i++ ){
		// -- Get the keypoints from the good matches
			obj.push_back( keypoints1[ matches[i].queryIdx ].pt );
			scene.push_back( keypoints2[ matches[i].trainIdx ].pt );
		}


		Mat flow = Mat::zeros(3,1,CV_32FC1);
		Mat x_flow;
		Mat y_flow;
		ostringstream oss;
		if(matches.size() > 50){
			Mat mask;
			Mat H1 = findHomography( obj, scene, mask, CV_RANSAC,5 );	
			for( uchar i = 0; i < mask.rows; i++ ){
				if ((int)mask.at<uchar>(0,i)==1){
				x_flow.push_back( scene.at(i).x - obj.at(i).x);
				y_flow.push_back( scene.at(i).y - obj.at(i).y);
				}
			}
			//cout<<x_flow<<endl;
			cout<<mean(x_flow)<<endl;
			cout<<mean(y_flow)<<endl;
		}

		else{
			cout<<"Not good"<<endl;	
		}
return flow;
}


int main(int argc, char** argv)
 {

		cv::namedWindow(OPENCV_WINDOW);
		Mat I1 = imread("Images4_I1.tif",0);
		Mat I2 = imread("Images4_I2.tif",0); 
		Mat flow =  orb_features_projective(I1, I2);
		//cout<<flow<<endl;	
		//cout <<"The type of image is  "<<I1.type()<<endl;
	waitKey(0);
   	return 0;
 }
