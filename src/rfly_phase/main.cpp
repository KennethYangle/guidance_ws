
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
#include <math.h>

using namespace std;
using namespace cv; 

//static const string OPENCV_WINDOW = "Image window1";
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

// Variables for Image Pyramids
Mat I1_level3,I1_level2,I1_level1,I2_level3,I2_level2,I2_level1;  
Mat u_l3,u_l2,u_l1,v_l3,v_l2,v_l1;

float x_l3[40],x_l2[80],x_l1[160];
float y_l3[30],y_l2[60],y_l1[120];   
float valid = 1;

int scale = 1;
int delta = 0;
int ddepth = CV_16S;
int cc = 0;
//END............................

//..............................START.........................................
// -- Creating u matrix for L1, L2,L3
	Mat x_val(int r, int col){
		float x[col];
		Mat u = Mat::zeros(r,col,CV_32FC1);
		for(int i=0;i<col;i++) x[i] = i+1;
		Mat A = Mat(1, col, CV_32F,x);		
		cv::repeat(A, r, 1, u);
		return u;
		}
// -- Creating v matrix for L1, L2,L3	
	Mat y_val(int r, int col){
		float y[r];
		Mat v = Mat::zeros(r,col,CV_32FC1);
		for(int i=0;i<r;i++) y[i] = i+1;
		Mat A = Mat(r, 1, CV_32F,y);		
		cv::repeat(A, 1, col, v);
		return v;
		}
//...............................END................................... 

//..............................START.........................................
Mat gradient_x(Mat I,Mat w){
		Mat grad_x = Mat::zeros(I.rows,I.cols,CV_32FC1);
		Mat gxx = Mat::zeros(I.rows,I.cols,CV_32FC1);		
		// Gradient X
  		Sobel( I/8, grad_x, ddepth, 1, 0, 3, scale, delta, BORDER_DEFAULT );
		grad_x.convertTo(gxx, CV_32FC1);
		Mat gx = w.mul(gxx);
		
		return gx;
		}
Mat gradient_y(Mat I,Mat w){
		Mat grad_y = Mat::zeros(I.rows,I.cols,CV_32FC1);
		Mat gyy = Mat::zeros(I.rows,I.cols,CV_32FC1);
		// Gradient Y
		Sobel( I/8, grad_y, ddepth, 0, 1, 3, scale, delta, BORDER_DEFAULT );
		grad_y.convertTo(gyy, CV_32FC1);
		Mat gy = w.mul(gyy);

		return gy;
		}

Mat gradient_t(Mat I11, Mat I22, Mat w){
		// Gradient It
		Mat I1(I11.rows,I11.cols,CV_32FC1);
		Mat I2(I11.rows,I11.cols,CV_32FC1);
		I11.convertTo(I1,CV_32FC1);
		I22.convertTo(I2,CV_32FC1);
		Mat Itt = I1-I2;
		Mat It = w.mul(Itt);

		return It;
		}
Mat window_w(int a, int r, int c){
		int top = a; int bottom = a;
		int left = a; int right = a;
		Mat image_const = Mat::ones(r-2*a,c-2*a,CV_32FC1);
		Mat w = Mat::zeros(r,c,CV_32FC1);
		copyMakeBorder( image_const, w, top, bottom, left, right, BORDER_CONSTANT, 0 );
		cv::FileStorage storage4("w.yml", cv::FileStorage::WRITE);
		storage4 << "w" << w;
		storage4.release();

		return w;
		}
Mat image_warping(Mat I, Mat tform){
		
   		/// Set the dst image the same type and size as src
   		Mat dst = Mat::zeros( I.rows, I.cols, I.type() );
		/// Apply the Affine Transform just found to the src image
   		warpAffine( I, dst, tform, dst.size() );
		return dst;
}
//...............................END..................................
Mat iteration(Mat I1,Mat I2, Mat u, Mat v, int a, int b){
			
	Mat w =  window_w(a,I1.rows, I1.cols);
	Mat t_form = Mat::eye(2,3,CV_32FC1);
	Mat t_form_s = Mat::zeros(2,3,CV_32FC1);
	Mat t_form_temp = Mat::zeros(2,3,CV_32FC1);
	Mat I11 = Mat::zeros(I1.rows,I1.cols,CV_32FC1);
	I1.copyTo(I11);
	for(int i = 0; i<b ; i++){
 
		Mat gx = gradient_x(I11,w);
		Mat gy = gradient_y(I11,w);
		Mat gt = gradient_t(I11,I2,w);
		
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

		Mat B = gt.reshape(1,gt.rows*gt.cols);

		Mat aff;
		//solve(A,B,aff,DECOMP_SVD);
		Mat At; transpose(A,At);
		Mat Bt; transpose(B,Bt);
		Mat C1; invert(At*A,C1);
		aff = C1*At*B;	
		
		t_form_s.at<float>(0,0)=aff.at<float>(1,0);
		t_form_s.at<float>(0,1)=aff.at<float>(2,0);
		t_form_s.at<float>(0,2)=aff.at<float>(0,0);
		t_form_s.at<float>(1,0)=aff.at<float>(4,0);
		t_form_s.at<float>(1,1)=aff.at<float>(5,0);
		t_form_s.at<float>(1,2)=aff.at<float>(3,0);

		t_form.copyTo(t_form_temp);
		t_form = t_form_temp + t_form_s;
		I11 = image_warping(I1, t_form);
	}

		return t_form;

}
//..............................START.........................................
//This is the main function that uses above defined functions to calculate the optical flow
Mat pyramids(Mat I1, Mat I2){
 				
		pyrDown( I1, I1_level1, Size( I1.cols/2, I1.rows/2 ));
		pyrDown( I1_level1, I1_level2, Size( I1_level1.cols/2, I1_level1.rows/2));
		pyrDown( I1_level2, I1_level3, Size( I1_level2.cols/2, I1_level2.rows/2)); 


		pyrDown( I2, I2_level1, Size( I2.cols/2, I1.rows/2 ));
		pyrDown( I2_level1, I2_level2, Size( I2_level1.cols/2, I2_level1.rows/2));
		pyrDown( I2_level2, I2_level3, Size( I2_level2.cols/2, I2_level2.rows/2));
		
		float a2,a3,b2,b3;
//...............Level3.................................
		Mat t_form3= iteration(I1_level3, I2_level3, u_l3, v_l3,5,10);		
//...............Level2.................................
		Mat t_form3_ini; t_form3.copyTo(t_form3_ini);
		a3 = t_form3.at<float>(0,2);
		b3 = t_form3.at<float>(1,2);
		
		t_form3_ini.at<float>(0,2) = 2*a3;
		t_form3_ini.at<float>(1,2) = 2*b3;
		
		Mat I1_level2_ini = image_warping(I1_level2,t_form3_ini);
		
		Mat t_form2= iteration(I1_level2_ini, I2_level2, u_l2, v_l2,10,5);

		a2 = t_form2.at<float>(0,2);
		b2 = t_form2.at<float>(1,2);
				
//..........................................................................
		Mat a,b;
		t_form3.copyTo(a);
		t_form2.copyTo(b);
	
		Mat tform = (a+b);

		tform.at<float>(0,0) = t_form3.at<float>(0,0) + t_form2.at<float>(0,0) - 1;
		tform.at<float>(1,1) = t_form3.at<float>(1,1) + t_form2.at<float>(1,1) - 1;
	
		tform.at<float>(0,2)=(8*a3 + 4*a2);			
		tform.at<float>(1,2)=(8*b3 + 4*b2);

		Mat pt1 = (Mat_ <float>(3,1)<<160,120,1);
		Mat pt = (Mat_ <float>(2,1)<<160,120);
		Mat flow = tform*pt1 - pt;

return flow;
}

//............................START Phase Correlation Method................................... 

Mat correlation_flow(Mat I1, Mat I2, Mat hann){
		I1.convertTo(I164f, CV_64F);
		I2.convertTo(I264f, CV_64F);
		shift = phaseCorrelate(I164f, I264f, hann);
		Mat flow = Mat::zeros(3,1,CV_32FC1);
		
		//if(abs(shift.x)>100 || abs(shift.y)>100){
		//	flow.at<float>(0,0) = 0;
		//	flow.at<float>(1,0) = 0;
		//	flow.at<float>(2,0) = 0;
		//}
		//else{
			flow.at<float>(0,0) = shift.x;
			flow.at<float>(1,0) = shift.y;
			flow.at<float>(2,0) = 1;
		//}

		return flow;
	}

//.............................END Phase Correlation Method.........................................
// ****************************Call Back Function***************************************************
int phase_cnt = 0;
double phase_total_time = 0;
void imageCb(const sensor_msgs::ImageConstPtr& msg){
	phase_cnt++;
	ros::Time begin_time = ros::Time::now();

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
		//cout<<"delt "<<delt<<endl;
		cc = cc+1; cout<<cc<<endl;

		Mat flow_co = correlation_flow(cv_ptr_I1->image, cv_ptr_I2->image, hann);
		// Mat flow_pyramids = pyramids(cv_ptr_I1->image, cv_ptr_I2->image);

		tform_msg.x = 1.4*(flow_co.at<float>(0,0));
		tform_msg.y = 1.4*(flow_co.at<float>(1,0));
		tform_msg.z = delt;

		cout<< "X_flow "<<tform_msg.x<<"Y_flow "<<tform_msg.y<<endl;
		tform_pub.publish(tform_msg);

		//Draw Circle
		//double radius = std::sqrt(tform_msg.x*tform_msg.x + tform_msg.y*tform_msg.y);
		//if(radius>5){
		//	Point center(cv_ptr_I1->image.cols >>1, cv_ptr_I1->image.rows>>1);
		//	circle(cv_ptr_I1->image , center, (int)radius, Scalar(0,255,0), 3);
		//	line(cv_ptr_I1->image , center , Point(center.x + (int)tform_msg.x , center.y + (int)tform_msg.y), Scalar(0,255,0), 3);
		//}
		//imshow(OPENCV_WINDOW , cv_ptr_I1->image);waitKey(3);

		cv_ptr_I2->image.copyTo(cv_ptr_I1->image);
		time1 = time2;
	}

	phase_total_time += (ros::Time::now() - begin_time).toSec();
	if (phase_cnt % 10 == 0) {
		cout << "phase_cnt: " << phase_cnt << ", phase_total_time: " << phase_total_time << endl;
	}
};
// ****************************END Call Back Function********************************************* 
// ++++++++++++++++++++++++++++Start Main Function++++++++++++++++++++++++++++++++++++++++++++++++ 
int main(int argc, char** argv)
 {	
	s.height = 240;
	s.width = 320;
	createHanningWindow(hann, s, CV_64F);

	u_l3 = x_val(30,40);u_l2 = x_val(60,80);u_l1 = x_val(120,160);
	v_l3 = y_val(30,40);v_l2 = y_val(60,80);v_l1 = y_val(120,160);

	ros::init(argc, argv, "rfly_phase");
//create a ros node handle
	ros::NodeHandle nh;
	//cv::namedWindow(OPENCV_WINDOW);
	//cv::namedWindow(I2_WINDOW);
// Create a subscriber object
	ros::Subscriber sub = nh.subscribe( "guidance/left_image" , 1 , &imageCb);//guidance/left_image
// publish the constant velocities on turtle2/cmd_vel
	tform_pub = nh.advertise<geometry_msgs::Point> ( "rfly/phase", 1);
   	ros::spin();
// rosrun topic_tools drop /guidance/left_image 1 2
 }
// ++++++++++++++++++++++++++++END Main Function++++++++++++++++++++++++++++++++++++++++++++++++ 
