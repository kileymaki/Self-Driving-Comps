/*
 * This class registers and updates the front camera sensor.
 *
 * This class also handles video processing and lane finding based
 * upon video inputs to handle lane finding and tracking using
 * methods found in the paper "A lane-curve detection based on LCF"
 */

#include <boost/bind.hpp>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/sensors/sensors.hh>
#include <gazebo/common/common.hh>
#include <stdio.h>
#include <vector>
#include <iostream>
#include <numeric>
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include <opencv2/opencv.hpp>
#include "sdcCameraSensor.hh"
#include "fadiff.h"

using namespace fadbad;
using namespace gazebo;
using namespace cv;

// Register this plugin with the simulator
GZ_REGISTER_SENSOR_PLUGIN(sdcCameraSensor)

// Pointer to the update event connection
event::ConnectionPtr updateConnection;
sensors::MultiCameraSensorPtr parentSensor;

// Cascade Classifier information using CPU
CascadeClassifier cpu_stop_sign;
String cascade_file_path = "OpenCV/haarcascade_stop.xml";

F<double> delG(const F<double>& x, const F<double>& y) {
	F<double> dG = sqrt(pow(x,2)+pow(y,2));
	return dG;
}

void sdcCameraSensor::Load(sensors::SensorPtr _sensor, sdf::ElementPtr /*_sdf*/){
		// Get the parent sensor.
		this->parentSensor =
		boost::dynamic_pointer_cast<sensors::MultiCameraSensor>(_sensor);

		// Make sure the parent sensor is valid.
		if (!this->parentSensor)
		{
				gzerr << "Couldn't find a camera\n";
				return;
		}

		// Connect to the sensor update event.
		this->updateConnection = this->parentSensor->ConnectUpdated(boost::bind(&sdcCameraSensor::OnUpdate, this));

		// Make sure the parent sensor is active.
		this->parentSensor->SetActive(true);
		//std::cout << this->parentSensor->GetNoise() << std::endl;
}

// Called by the world update start event
void sdcCameraSensor::OnUpdate() {
	// pull raw data from camera sensor object
	const unsigned char* img = this->parentSensor->GetImageData(0);
	Mat image = Mat(this->parentSensor->GetImageHeight(0), this->parentSensor->GetImageWidth(0), CV_8UC3, const_cast<unsigned char*>(img));

	//Select Region of Interest (ROI) for lane detection - currently this is the bottom half of the image.
	//set area for ROI as a rectangle
	//Rect ROI = cv::Rect(0, (4*image.rows)/5, image.cols, image.rows/5);
	Rect ROI = cv::Rect(0, image.rows/2, image.cols, image.rows/2);
	Mat imageROI = image(ROI);
	//rectangle(image,ROI,Scalar(0,255,0),2);

	// Canny algorithm for edge dectection
	Mat contours;
	Canny(imageROI,contours,50,150);
	//Mat contoursInv;
	//threshold(contours,contoursInv,128,255,THRESH_BINARY_INV);
	// Hough Transform detects lines within the edge map, stores result in lines.
	float PI = 3.14159;
	std::vector<Vec2f> lines;
	HoughLines(contours,lines,1,PI/180, 100);
	//HoughLinesP(contours,lines,1,PI/180,80, 30, 10 );

	//print out line angles
	// for (std::vector<Vec2f>::const_iterator i = lines.begin(); i != lines.end(); ++i)
	// std::cout << *i << ' ' << std::endl;
	// std::cout << "=====================================\n";


	std::vector<Vec2f>::const_iterator it = lines.begin();

	// white line grid overlay for reference points on displayed image
	//line(imageROI, Point((imageROI.cols/2)-10,0), Point((imageROI.cols/2)+10,0), Scalar(255,255,255), 2);
	//line(imageROI, Point(imageROI.cols/2,10), Point(imageROI.cols/2,-10), Scalar(255,255,255), 2);

	Vec2f left_lane_marker = Vec2f(0.0, PI);
	Vec2f right_lane_marker = Vec2f(0.0, 0.0);
	//std::cout << left_lane_marker[0];

	while (it!=lines.end()) {
			float rho= (*it)[0];   // first element is distance rho
			float theta= (*it)[1]; // second element is angle theta
				//std::cout << "rho: " << rho << "| theta: " << theta << std::endl;
				if ( 0 < theta < (PI/2 - 0.1) && theta < left_lane_marker[1]) {
					left_lane_marker = Vec2f(rho,theta);
					//std::cout << "Left lane data updated! rho: " << rho << "| theta: " << theta << std::endl;
				}

				if ((PI/2 + 0.1) < theta < (PI - 0.1) && theta > right_lane_marker[1]) {
					right_lane_marker = Vec2f(rho,theta);
				}

			// Point pt1(rho/cos(theta),0);
			// Point pt2((rho-imageROI.rows*sin(theta))/cos(theta),imageROI.rows);
			// line(imageROI, pt1, pt2, Scalar(0,0,255), 3);

			++it;
	}
	// ATTN: need to have ROI.rows in numerator because of trig stuff. It isnt an oversight!
	// Changed things so the lines are drawn on the image, not the region of interest. hopefully
	// this will make some of the issues easier to debug
	//std::cout << "left theta angle: " << left_lane_marker[1] << "| right theta angle: " << right_lane_marker[1] << std::endl;
	//draw left lane marker
	Point leftp1 = Point(left_lane_marker[0]/cos(left_lane_marker[1]),0.5*image.rows);
	Point leftp2 = Point((left_lane_marker[0] - (imageROI.rows) * sin(left_lane_marker[1])) / cos(left_lane_marker[1]), (image.rows));
	line(image, leftp1, leftp2, Scalar(255), 3);

	//draw right lane marker
	Point rightp1 = Point(right_lane_marker[0]/cos(right_lane_marker[1]),0.5*image.rows);
	Point rightp2 = Point((right_lane_marker[0] - (imageROI.rows) * sin(right_lane_marker[1])) / cos(right_lane_marker[1]), (image.rows));
	line(image, rightp1, rightp2, Scalar(255), 3);

	// std::cout << leftp1 << "\t" << leftp2 << "\t" << rightp1 << "\t" << rightp2 << "\t" << std::endl;

	//BEGIN HAAR CASCADE OBJECT DETECTION
	if(!cpu_stop_sign.load(cascade_file_path)){ printf("--(!)Error loading cascade\n");};
	std::vector<Rect> stopSigns;
	cpu_stop_sign.detectMultiScale( image, stopSigns, 1.1, 2, 0|CASCADE_SCALE_IMAGE, Size(30, 30) );
	if(stopSigns.size() > 0){
		sdcSensorData::stopSignFrameCount++;
	}else{
		sdcSensorData::stopSignFrameCount = 0;
	}

	double avgSize = 0;
	for( int i = 0; i < stopSigns.size(); i++ ){
	   cv::rectangle(image, stopSigns[i], Scalar(0,0,255),3,LINE_8,0);

	   avgSize += stopSigns[i].width * stopSigns[i].height;
	}
	avgSize = avgSize / stopSigns.size();

	if(stopSigns.size() > 0){
		sdcSensorData::sizeOfStopSign = avgSize;
	}else{
		sdcSensorData::sizeOfStopSign = 0;
	}


	// BEGIN LCF LANE DETECTION
	// This algorithm was decribed in the paper "A lane-curve detection based on an LCF"
	// Each step that corresponds to an equation will be labelled accordingly.
	double leftNearLaneSlope, rightNearLaneSlope, leftLaneIntercept, rightLaneIntercept;
	double a, b, c, d, e, u, v, n, k, lane_midpoint, eps = 100.0;
	float FOCAL_LENGTH = 554.382; //lambda in Park et. al.
	float Tz = 0.85; // in meters
	// float xf = leftp1.x;
	// A = eps * i / lambda^2 * Tz
	// A = 25 * i / 471.2247

	//TBH THIS STUFF SHOULD NOT BE SET EVERY UPDATE NEEDS TO BE MOVED ~~~~~~~
	std::vector<double> vec_of_i_vals(101);
	std::iota(std::begin(vec_of_i_vals), std::end(vec_of_i_vals), -50.);
	//vec_of_i_vals.push_back(48.);
	//vec_of_i_vals.push_back(-48.);
	//vec_of_i_vals.push_back(-100.);
	// for (std::vector<double>::const_iterator i = vec_of_i_vals.begin(); i != vec_of_i_vals.end(); ++i)
	// std::cout << *i << ' ' << std::endl;
	// std::cout << "=====================================\n";


	// Using the two lane markers
	Point vanishingPoint;

	// using the left lane
	if (leftp1.x - leftp2.x != 0) {
			leftNearLaneSlope = (1.*leftp2.y - leftp1.y)/(leftp2.x - leftp1.x);
	}

	// using the right lane
	if (rightp2.x - rightp1.x != 0) {
			rightNearLaneSlope = (1.*rightp2.y - rightp1.y)/(rightp2.x - rightp1.x);
	}
	 // std::cout << "left slope: " << leftNearLaneSlope << "\t|\t" << "right slope: "<< rightNearLaneSlope << std::endl;

	// Calculate y-intercepts of the lanes
	//b = y - mx
	leftLaneIntercept = leftp1.y - leftNearLaneSlope*leftp1.x; //((0-leftp1.y)/leftNearLaneSlope)+leftp1.x;
	rightLaneIntercept = rightp1.y - rightNearLaneSlope*rightp1.x;// ((0-rightp1.y)/rightNearLaneSlope)+rightp1.x;
	// std::cout << "left intercept: " << leftLaneIntercept << "\t|\t" << "right intecept: "<< rightLaneIntercept << std::endl;
	/////////////////////////////////////////////////////////////////////////////////////////////////////////
	// SOLVE VANISHING POINT (u,v), which is intersection of two lines
	//v is the height component of the vanishing point, described as vp = (u,v)
	//v = image.rows/2;
	//au + c = bu + d
	//u = (d - c) / (a - b)


	u = abs((leftLaneIntercept - rightLaneIntercept) / (leftNearLaneSlope - rightNearLaneSlope));
	v = (leftNearLaneSlope * u) + leftLaneIntercept;
	Point vp = Point(u,v);
	// std::cout << "Vanishing Point: " << vp.x << "|" << vp.y << std::endl;
	circle(image,vp, 4, Scalar(0,255,0), 3);

	lane_midpoint = (leftp2.x + rightp2.x)/2;

	// n is our position relative to the center of the lane directly in front of us.
	n = (image.cols/2) - lane_midpoint;

	// Naive lane detection is better than no lane detection
	sdcSensorData::UpdateCameraData(n);

	line(image, Point(lane_midpoint, 480), Point(lane_midpoint, 0), Scalar(0, 255, 0), 1);
	Point nfa_p1, nfa_p2, ffa_p1, ffa_p2;


	/////////////////////////////////////////////////////////////////////////////////////////////////////////
	/////////////////////////////////////////////////////////////////////////////////////////////////////////
	/////////////////////////////////////LEFT LANE MARKER CALCULATION////////////////////////////////////////
	/////////////////////////////////////////////////////////////////////////////////////////////////////////
	/////////////////////////////////////////////////////////////////////////////////////////////////////////
	// std::cout << "n: " << n << "\tlane midpoint: " << lane_midpoint << std::endl;

	a = leftNearLaneSlope/2;
	b = (v+leftLaneIntercept)/2;
	c = pow(leftNearLaneSlope,2)/4;
	d = a * (leftLaneIntercept-v);

	//DRAW LEFT NEAR FIELD AND FAR FIELD ASYMPTOTES

	ffa_p1.x = 0.;
	ffa_p2.x = u;
	ffa_p1.y = v; //Cy, center of computer image
	ffa_p2.y = v;

	nfa_p1.x = 0.;
	nfa_p2.x = u;
	nfa_p1.y = (leftNearLaneSlope)*(nfa_p1.x) + leftLaneIntercept;//(b + (d/(2*sqrt(c))));
	nfa_p2.y = (leftNearLaneSlope)*(nfa_p2.x) + leftLaneIntercept;//(b + (d/(2*sqrt(c))));
	// std::cout << (b + (d/(2*sqrt(c)))) << "\t" << leftLaneIntercept << std::endl;
	line(image, nfa_p1, nfa_p2, Scalar(255,255,0), 1, CV_AA);
	line(image, ffa_p1, ffa_p2, Scalar(255,255,0), 1, CV_AA);
	//std::cout << "ASYMPTOTES: " << nfa_p1 << "\t" << nfa_p2 << "\t" << ffa_p1 << "\t" << ffa_p2 << "\t" << std::endl;
	// std::cout << "VARIABLES: " << a << "\t" << b << "\t" << c << "\t" << d << "\t" << std::endl;
	int left_max_score = 0, left_optimal_i = 0;

	//generate curvature list
	for (std::vector<double>::const_iterator q = vec_of_i_vals.begin(); q != vec_of_i_vals.end(); q++) {
		//std::cout << "left current curve is: " << *q << std::endl;
		e = pow((b-v),2) + (eps * a * *q);


		std::vector<int> left_Histogram(31);
		std::vector<Point> left_curve_points_top, left_curve_points_bot;


		for (float x = 0.; x < u; x++ ) {
			float left_y_top = (a * x) + b + sqrt( c*pow(x,2) + (d * x) + e);
			float left_y_bot = (a * x) + b - sqrt( c*pow(x,2) + (d * x) + e);

			if(left_y_top >= v) {
					Point left_curve_point_top = Point(x,left_y_top);
					left_curve_points_top.push_back(left_curve_point_top);
			}

			if(left_y_bot >= v) {
					Point left_curve_point_bot = Point(x,left_y_bot);
					left_curve_points_bot.push_back(left_curve_point_bot);
			}
		}

		// if(left_curve_points_top.size() > 1) {
		// 	for (int j = 0; j < left_curve_points_top.size() - 1; j++) {
		// 		line(image, left_curve_points_top[j], left_curve_points_top[j + 1], Scalar(0,0,255), 1, CV_AA);
		// 	}
		// }

		double left_curve_magnitude = 0;
		if(left_curve_points_bot.size() > 1) {
			for (int j = 0; j < left_curve_points_bot.size() - 1; j++) {
				//This is where we would do LROI calculations
				//LROI is a triangle, the top point is height of vanishing point.
				//LROI is only calculated for the bottom half of the LCF
				//line(image, left_curve_points_bot[j], left_curve_points_bot[j + 1], Scalar(0,0,255), 1, CV_AA);
				double xf,yf;
				double ypf_pos, ypf_neg,g;

				xf = left_curve_points_bot[j].x;
				yf = left_curve_points_bot[j].y;
				g = contours.at<uchar>(yf,xf);
				//std::cout << g << std::endl;
				left_curve_magnitude += g;

			}
		}

		//double left_curve_magnitude_total = ((1.*left_curve_points_bot.size()) - left_curve_magnitude)/(1.*left_curve_points_bot.size());
		double left_curve_magnitude_total = ((1.*left_curve_points_bot.size()) - left_curve_magnitude);
		if (left_curve_magnitude_total >= left_max_score) {
			//std::cout << left_score << " should be greater than " << left_max_score;
			left_max_score = left_curve_magnitude_total;
			//std::cout << ". our new high score is : " << left_score;
			left_optimal_i = *q;
			//std::cout << ", which is given by curve number: " << left_optimal_i << std::endl;
			//std::cout << left_optimal_i << std::endl;
		}

		// std::cout << *q << " | ";
		// for (std::vector<int>::const_iterator i = left_Histogram.begin(); i != left_Histogram.end(); ++i){
		// 	std::cout << *i << ' '; //<< std::endl;
		// }
		// std::cout << " | " << left_score << " | " << left_optimal_i << " | \n";
	}

	//DISPLAY LEFT LANE CURVE WITH OPTIMAL CURVATURE VALUE
	e = pow((b-v),2) + (eps * a * left_optimal_i);
	std::vector<Point> left_curve_points_top, left_curve_points_bot;

	for (float x = 0.; x < 640; x++ ) {
		float left_y_top = (a * x) + b - sqrt( c*pow(x,2) + (d * x) + e);
		float left_y_bot = (a * x) + b + sqrt( c*pow(x,2) + (d * x) + e);

		if(left_y_top >= v) {
				Point left_curve_point_top = Point(x,left_y_top);
				left_curve_points_top.push_back(left_curve_point_top);
		}

		if(left_y_bot >= v) {
				Point left_curve_point_bot = Point(x,left_y_bot);
				left_curve_points_bot.push_back(left_curve_point_bot);
		}
	}

	// if(left_curve_points_top.size() > 1) {
	// 	for (int i = 0; i < left_curve_points_top.size() - 1; i++){
	// 		line(image, left_curve_points_top[i], left_curve_points_top[i + 1], Scalar(0,255,0), 3, CV_AA);
	// 	}
	// }

	if(left_curve_points_bot.size() > 1) {
		for (int i = 0; i < left_curve_points_bot.size() - 1; i++){
			line(image, left_curve_points_bot[i], left_curve_points_bot[i + 1], Scalar(0,255,0), 3, CV_AA);
		}
	}

//std::cout << "==============================================================================" << std::endl;


	/////////////////////////////////////////////////////////////////////////////////////////////////////////
	/////////////////////////////////////////////////////////////////////////////////////////////////////////
	////////////////////////////////////RIGHT LANE MARKER CALCULATION////////////////////////////////////////
	/////////////////////////////////////////////////////////////////////////////////////////////////////////
	/////////////////////////////////////////////////////////////////////////////////////////////////////////
	a = rightNearLaneSlope/2;
	b = (v+rightLaneIntercept)/2;
	c = pow(rightNearLaneSlope,2)/4;
	d = a * (rightLaneIntercept-v);

	//DRAW RIGHT NEAR FIELD AND FAR FIELD ASYMPTOTES
	//Point nfa_p1, nfa_p2, ffa_p1, ffa_p2;
	ffa_p1.x = 640.;
	ffa_p2.x = u;
	ffa_p1.y = v; //Cy, center of computer image
	ffa_p2.y = v;

	nfa_p1.x = 640.;
	nfa_p2.x = u;
	nfa_p1.y = (rightNearLaneSlope)*(nfa_p1.x) + rightLaneIntercept;//(b + (d/(2*sqrt(c))));
	nfa_p2.y = (rightNearLaneSlope)*(nfa_p2.x) + rightLaneIntercept;//(b + (d/(2*sqrt(c))));
	// std::cout << (b + (d/(2*sqrt(c)))) << "\t" << leftLaneIntercept << std::endl;
	line(image, nfa_p1, nfa_p2, Scalar(255,255,0), 1, CV_AA);
	line(image, ffa_p1, ffa_p2, Scalar(255,255,0), 1, CV_AA);

	//std::cout << "ASYMPTOTES: " << nfa_p1 << "\t" << nfa_p2 << "\t" << ffa_p1 << "\t" << ffa_p2 << "\t" << std::endl;
	// std::cout << "VARIABLES: " << a << "\t" << b << "\t" << c << "\t" << d << "\t" << std::endl;
	double right_max_score = 0, right_optimal_i = 0;

	//generate curvature list
	for (std::vector<double>::const_iterator q = vec_of_i_vals.begin(); q != vec_of_i_vals.end(); q++) {
		//std::cout << " right current curve is: " << *q;
		e = pow((b-v),2) + (eps * a * *q);

		std::vector<int> right_Histogram(31);
		std::vector<Point> right_curve_points_top, right_curve_points_bot;


		for (float x = u+1; x < 640. ; x++ ) {
			float right_y_top = (a * x) + b - sqrt( c*pow(x,2) + (d * x) + e);
			float right_y_bot = (a * x) + b + sqrt( c*pow(x,2) + (d * x) + e);

			if(right_y_top >= v) {
					Point right_curve_point_top = Point(x,right_y_top);
					right_curve_points_top.push_back(right_curve_point_top);
			}

			if(right_y_bot >= v) {
					Point right_curve_point_bot = Point(x,right_y_bot);
					right_curve_points_bot.push_back(right_curve_point_bot);
			}
		}

		// if(right_curve_points_top.size() > 1) {
		// 	for (int j = 0; j < right_curve_points_top.size() - 1; j++){
		// 		line(image, right_curve_points_top[j], right_curve_points_top[j + 1], Scalar(0,0,255), 1, CV_AA);
		// 	}
		// }
		double curve_magnitude = 0;
		if(right_curve_points_bot.size() > 1) {
			for (int j = 0; j < right_curve_points_bot.size() - 1; j++) {
				//This is where we would do LROI calculations
				//LROI is a triangle, the top point is height of vanishing point.
				//LROI is only calculated for the bottom half of the LCF
				//line(image, right_curve_points_bot[j], right_curve_points_bot[j + 1], Scalar(0,0,255), 1, CV_AA);
				double xf,yf;
				double ypf_pos, ypf_neg,g;
				xf = right_curve_points_bot[j].x;
				yf = right_curve_points_bot[j].y;
				g = contours.at<uchar>(yf,xf);
				//std::cout << g << std::endl;
				curve_magnitude += g;

			}
		}

		//double curve_magnitude_total = ((1.*right_curve_points_bot.size()) - curve_magnitude)/(1.*right_curve_points_bot.size());
		double curve_magnitude_total = ((1.*right_curve_points_bot.size()) - curve_magnitude);
		// std::cout << "Curve " << *q << " score is: " << curve_magnitude_total << std::endl;
		//int right_score = std::accumulate(right_Histogram.begin(), right_Histogram.end(), 0);
		if (curve_magnitude_total > right_max_score) {
			right_max_score = curve_magnitude_total;
			// std::cout << curve_magnitude_total << " should be greater than " << right_max_score;
			// std::cout << ". our new high score is : " << curve_magnitude_total;
			right_optimal_i = *q;
			//std::cout << ", which is given by curve number: " << right_optimal_i << std::endl;
			//std::cout << right_optimal_i << std::endl;
		}
	}

	//std::cout << right_optimal_i << std::endl;

	//DISPLAY RIGHT LANE CURVE WITH OPTIMAL CURVATURE VALUE
	e = pow((b-v),2) + (eps * a * right_optimal_i);
	std::vector<Point> right_curve_points_top, right_curve_points_bot;

	for (float x = u+1.; x < 640. ; x++ ) {
		float right_y_top = (a * x) + b - sqrt( c*pow(x,2) + (d * x) + e);
		float right_y_bot = (a * x) + b + sqrt( c*pow(x,2) + (d * x) + e);

		if(right_y_top >= v) {
				Point right_curve_point_top = Point(x,right_y_top);
				right_curve_points_top.push_back(right_curve_point_top);
		}

		if(right_y_bot >= v) {
				Point right_curve_point_bot = Point(x,right_y_bot);
				right_curve_points_bot.push_back(right_curve_point_bot);
		}
	}

	// if(right_curve_points_top.size() > 1) {
	// 	for (int i = 0; i < right_curve_points_top.size() - 1; i++){
	// 		line(image, right_curve_points_top[i], right_curve_points_top[i + 1], Scalar(0,255,0), 3, CV_AA);
	// 	}
	// }

	if(right_curve_points_bot.size() > 1) {
		for (int i = 0; i < right_curve_points_bot.size() - 1; i++){
			line(image, right_curve_points_bot[i], right_curve_points_bot[i + 1], Scalar(0,255,0), 3, CV_AA);
		}
	}

///////// END LCF LANE DETECTION

	namedWindow("Lane Detection", WINDOW_AUTOSIZE);
	imshow("Lane Detection", contours);
	namedWindow("Camera View", WINDOW_AUTOSIZE);
	imshow("Camera View", image);
	waitKey(4);
}
