// include ROS Libraries
#include <ros/ros.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/TransformStamped.h>
#include <sensor_msgs/LaserScan.h>
// OpenCV Libraries
#include <opencv2/opencv.hpp>
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/core/core.hpp"
#include "opencv2/highgui/highgui.hpp"
// C++ Libraries
#include <iostream>
#include <stdio.h>
#include <math.h>

using namespace std;
using namespace cv;


// Initialising Certain constant Matrices

Mat Cbe1 = Mat:: eye(3,3, CV_64FC1);
Mat Cbe2 = Mat:: eye(3,3, CV_64FC1);
Mat H1 =  (Mat_<double>(1,7) << 0, 0, 0, 1, 0, 0, 0);
Mat H3 =  (Mat_<double>(3,7) << 1, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0);
Mat Q =  0.01*(Mat::eye(7,7,CV_64FC1));
Mat R3 = 0.01*(Mat::eye(3,3,CV_64FC1));
Mat X = (Mat_<double>(7,1) << 0, 0, 0, 0, 0, 0, 0);
Mat P = Mat::eye(7,7,CV_64FC1);
Mat w = Mat::zeros(3,1,CV_64FC1);
Mat camera_param = (Mat_<double>(3,3) <<334.4204,0,149.4593,0,333.4688,114.9624,0,0,1);

// Create a publisher object for distance
ros::Publisher pub_dist;
geometry_msgs::Point dist_msg;

// Create a publisher object for velocity
ros::Publisher pub_vel;
geometry_msgs::Point vel_msg;

double R1 = 0.1;
double a,b,c,d,a1,b1,c1,d1,ds,ds2,time1,time2,delt,Tx_pyr,Ty_pyr,dt_pyr;
double x_d = 0; 
double y_d = 0;
bool co1 = false;
bool co2 = false;
bool co3 = false;
//...................Function Definition for Skew Matrix...........
Mat skewmatrix(Mat v){
	double v1 = v.at<double>(0,0);
	double v2 = v.at<double>(1,0);
	double v3 = v.at<double>(2,0);
	Mat R = (Mat_<double>(3,3) << 0, -v3, v2, v3, 0, -v1, -v2, v1, 0);
return R;
}
//.................................................................
//...................LKF_constants Definition......................
struct lkf_constants {
    Mat Ad;
    Mat G;
};
lkf_constants calculate_lkf_constants(Mat Acc, Mat Cbe, double Ts){
	lkf_constants L;
	Mat F = 9.8*Acc;
	double g = 9.8;
	Mat Ad = (Mat_<double>(7,7) << 1,0,0,0,-Ts,0,0,0,1,0,0,0,-Ts,0,0,0,1,0,0,0,-Ts,0,0,-Ts,1,0,0,Ts*Ts/2,0,0,0,0,1,0,0,0,0,0,0,0,1,0,0,0,0,0,0,0,1);

	double c31 = Cbe.at<double>(2,0);
	double c32 = Cbe.at<double>(2,1);
	double c33 = Cbe.at<double>(2,2);

	double f1 = F.at<double>(0,0);
	double f2 = F.at<double>(1,0);
	double f3 = F.at<double>(2,0);
	
	Mat G = (Mat_<double>(7,1) << (f1+c31*g)*Ts, (f2+c32*g)*Ts, (f3+c33*g)*Ts, -(f3+c33*g)*(Ts*Ts)/2, 0 , 0, 0);
	
	L.Ad = Ad;
	L.G = G;
	return L;	
}
//..........................................................
//...................LKF_Predict and LKF_Update Definition..
struct KF {
    Mat X;
    Mat P;	
};
void lkf_predict(Mat A, Mat G){
		Mat At; transpose(A,At);
        	X = A * X + G; 
        	P=A*P*At + Q;	
}
void lkf_update(Mat im_vel, int valid){

	double Vx = im_vel.at<double>(0,0);
	double Vy = im_vel.at<double>(1,0);
	
	Mat I = Mat::eye(7,7,CV_64FC1);

	double Z1 = ds2;
	Mat Z3 = (Mat_<double>(3,1)<<Vx,Vy,ds2);
    	if(valid==1){ 
		Mat H3t; transpose(H3,H3t);

        	Mat y=Z3-H3*X;
        	Mat S=H3*P*H3t+R3;
		Mat Si; invert(S,Si);
        	Mat K=P*H3t*Si;
        	X = X + K*y;
        	P=(I-K*H3)*P;
	}
    	else{ 
		Mat H1t; transpose(H1,H1t);

        	Mat y=Z1-H1*X;
        	Mat S=H1*P*H1t+R1;
		Mat Si; invert(S,Si);
        	Mat K=P*H1t*Si;
        	X = X + K*y;
        	P=(I-K*H1)*P;
	}	
}
//..........................................................
//..... Function Definition of Get Rotation.................
Mat getrotation (double a,double b,double c,double d){
	//The function converts quaternions into Cbe matrix
	double R11 = a*a + b*b - c*c - d*d;
	double R12 = 2*(b*c-a*d); 
	double R13 = 2*(b*d+a*c);
	double R21 = 2*(b*c+a*d);
	double R22 =  a*a - b*b + c*c - d*d;
	double R23 = 2*(c*d-a*b);
	double R31 = 2*(b*d-a*c);
	double R32 = 2*(c*d+a*b);
	double R33 = a*a - b*b - c*c + d*d;

	Mat Cbe = (Mat_<double>(3,3)<<R11,R12,R13,R21,R22,R23,R31,R32,R33); // 3.59 body to E
return Cbe;
}
//.........................................................
//......Function Definition of Get Angular Rates...........
Mat quat2rate( double delt,double a,double b,double c,double d,double a1,double b1,double c1,double d1 ){
    
	double dela = a-a1;
	double delb = b-b1;
	double delc = c-c1;
	double deld = d-d1;
    
	Mat A = (Mat_<double>(4,1)<< dela/delt, delb/delt, delc/delt, deld/delt);
	Mat B = (Mat_<double>(4,4)<< a, -b, -c, -d, b, a, -d, c, c, d, a, -b, d, -c, b, a);
	Mat Bi; invert(B,Bi);
	Mat wb = 2*180/M_PI*(Bi*A);
	Mat w =Mat::zeros(3,1,CV_64FC1);
	w.at<double>(0,0) = wb.at<double>(1,0);
	w.at<double>(1,0) = wb.at<double>(2,0);
	w.at<double>(2,0) = wb.at<double>(3,0);
	return w;
}
//.........................................................
//..........Function Definition for Image Velocity.........
Mat image_velocity(double u,double v,double Ts){

	Mat P1 = (Mat_<double>(2,1)<<160,120);
	Mat P2 = (Mat_<double>(2,1)<<160-u,120-v);

	Mat Cbi = (Mat_<double>(3,3)<<0,1,0,-1,0,0,0,0,1);
	Mat Cbit; transpose(Cbi,Cbit);

	Mat wc = Cbi*w;

	Mat inv_camera_param; invert(camera_param,inv_camera_param);

	Mat e3 = (Mat_<double>(3,1)<< 0,0,1);Mat e3t; transpose(e3,e3t);

	Mat p1_temp = (Mat_<double>(3,1)<< P1.at<double>(0,0),P1.at<double>(0,1),1); 
	Mat p2_temp = (Mat_<double>(3,1)<< P2.at<double>(0,0),P2.at<double>(0,1),1);

	Mat p1 = inv_camera_param*p1_temp;
	Mat p2 = inv_camera_param*p2_temp;

	Mat Ep1 = -ds*p1/(e3t*Cbe1*p1);
	Mat Ep2 = -ds2*p2/(e3t*Cbe2*p2);
	
	Mat I = Mat::eye(3,3,CV_64FC1);
	Mat sk_w = M_PI/180*skewmatrix(wc);
	Mat Stav = (Ep1-(I+Ts*sk_w)*Ep2)/Ts;
//Stav = (Ep1-(eye(3,3)+Ts*skewmatrix(pi/180*w))*Ep2)/Ts;

	double Tx = Stav.at<double>(0,0);
	double Ty = Stav.at<double>(0,1);
	Mat temp1 = (Mat_<double>(3,1)<<Tx,Ty,0);
	Mat vel = Cbit*temp1;
return vel;
}

// Call Back Function
void imuCb(const geometry_msgs::TransformStamped& msg){

	if(!co1 ){
	//	std_msgs::Header h = msg->header;
		double sec = msg.header.stamp.sec;
		double nsec = msg.header.stamp.nsec;
		time2 = sec + nsec/1000000000;
		co1 = true;
		cout << X <<endl;
	}
	else{
				
		time1 = time2;		
	//	std_msgs::Header h = msg->header;
		double sec = msg.header.stamp.sec;
		double nsec = msg.header.stamp.nsec;
		time2 = sec + nsec/1000000000;
		delt = time2-time1;

		double acc1 = msg.transform.translation.x;
		double acc2 = msg.transform.translation.y;
		double acc3 = msg.transform.translation.z;
		Mat Acc = (Mat_<double>(3,1) << acc1,acc2,acc3);

		a1 = a; b1 = b; c1 = c;d1 = d;
		a = msg.transform.rotation.w;
		b = msg.transform.rotation.x;
		c = msg.transform.rotation.y;
		d = msg.transform.rotation.z;

		Cbe2.copyTo(Cbe1);
		Cbe2 =  getrotation(a,b,c,d);
		w = quat2rate(delt,a,b,c,d,a1,b1,c1,d1);
		
		lkf_constants lkc = calculate_lkf_constants(Acc, Cbe2, delt);
		lkf_predict(lkc.Ad, lkc.G);
		Mat im_vel = Mat::zeros(2,1,CV_64FC1);
		lkf_update(im_vel, 0);
		
		double tempx = 	X.at<double>(0,0);
		double tempy = 	X.at<double>(1,0);
		double tempz = 	X.at<double>(2,0);
			
		Mat temp_X = (Mat_<double>(3,1)<<tempx,tempy,tempz);		
		Mat X_world = Cbe2*temp_X;
		double x_ds = x_d;
		double y_ds = y_d;

		x_d = x_ds + delt*X_world.at<double>(0,0);
		y_d = y_ds + delt*X_world.at<double>(1,0);
		double z_d = X.at<double>(3,0);

		dist_msg.x = x_d;
		dist_msg.y = y_d;
		dist_msg.z = z_d;

		vel_msg.x = X_world.at<double>(0,0);
		vel_msg.y = X_world.at<double>(1,0);
		vel_msg.z = X_world.at<double>(2,0);

		pub_dist.publish(dist_msg);
		pub_vel.publish(vel_msg);

		//cout << "x_d=" << x_d <<" y_d="<< y_d<< " z_d=" << z_d<<endl;
	}

};
// Call Back Function

void floworbCb(const geometry_msgs::Point& msg){

			double Tx = msg.x;
			double Ty = msg.y;
			double dt = msg.z;
			Mat im_vel;
			if (dt == 0){	
			im_vel  = image_velocity(Tx_pyr,Ty_pyr,dt_pyr);
			cout << "orb"<<endl;}
			else{
			im_vel = image_velocity(Tx,Ty,dt);
			cout << "pyramids"<<endl;}

			lkf_update(im_vel, 1);

};

void flowpyrCb(const geometry_msgs::Point& msg){

			 Tx_pyr = msg.x;
			 Ty_pyr = msg.y;
			 dt_pyr = msg.z;


};

void ultCb(const sensor_msgs::LaserScan& msg){

	if(!co2 ){
		vector<float> data = msg.ranges;
		ds2 = data[0];
		ds = ds2;
		co2 = true;
	}
	else{	
		ds = ds2;
		vector<float> data = msg.ranges;
		ds2 = data[0];
		//if (ds2<0.2 || ds2>20)
			ds2 = ds;

	}
}
//.........................................................
int main(int argc, char** argv){
	a=1; b = 0;  c = 0;  d = 0;
	a1=1;b1 = 0; c1 = 0; d1 = 0;
	ds = 1; ds2 = 1;
	Tx_pyr = 0; Ty_pyr = 0; dt_pyr = 0.05;

	time1 = 0; time2 =0; delt = 0.05;

	ros::init(argc, argv, "navigation2_Node");
//create a ros node handle
	ros::NodeHandle nh;
	ros::Subscriber sub1 = nh.subscribe( "guidance/imu" , 1 , &imuCb);//guidance/left_image
	ros::Subscriber sub2 = nh.subscribe( "guidance/ultrasonic" , 1 , &ultCb);//guidance/left_image
	ros::Subscriber sub3 = nh.subscribe( "rfly/orb" , 1 , &floworbCb);//guidance/left_image
	ros::Subscriber sub4 = nh.subscribe( "rfly/pyramids" , 1 , &flowpyrCb);//guidance/left_image

	pub_dist = nh.advertise<geometry_msgs::Point> ( "rfly/dist", 1);
	pub_vel = nh.advertise<geometry_msgs::Point> ( "rfly/velocity", 1);

	ros::spin();
}

