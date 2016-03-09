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

//FADBAD-wrapped forward differentiation for gradient calculations
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
		if(!cpu_stop_sign.load(cascade_file_path)) {
			std::cout << "Unable to load cascade classifier xml file!" << std::endl;
		}
}

// Called by the world update start event
void sdcCameraSensor::OnUpdate() {

	// Pull raw data from camera sensor object as an unsigned character array with 3 channels.
	const unsigned char* img = this->parentSensor->GetImageData(0);
	Mat image = Mat(this->parentSensor->GetImageHeight(0), this->parentSensor->GetImageWidth(0), CV_8UC3, const_cast<unsigned char*>(img));

	//Select Region of Interest (ROI) for lane detection - currently this is the bottom half of the image.
	//set area for ROI as a rectangle

	Rect ROI = cv::Rect(0, image.rows/2, image.cols, image.rows/2);
	Mat imageROI = image(ROI);

	// Canny algorithm for edge dectection
	Mat contours, contours_thresh;
	Canny(imageROI,contours,50,150);
	threshold(contours,contours_thresh,127,255, THRESH_BINARY);

	Mat imageGray;
	Mat imageInv = Mat(imageROI.rows, imageROI.cols, CV_8UC1, Scalar(0));
	cvtColor(imageROI,imageGray,CV_BGR2GRAY);
	adaptiveThreshold(imageGray,imageInv, 255,CV_ADAPTIVE_THRESH_MEAN_C,CV_THRESH_BINARY,201, -20 );

// BEGIN STRAIGHT LANE DETECTION
	// Hough Transform detects lines within the edge map, stores result in lines.
	float PI = 3.14159;
	std::vector<Vec2f> lines;
	HoughLines(contours,lines,1,PI/180, 75);

	std::vector<Vec2f>::const_iterator it = lines.begin();

	Vec2f left_lane_marker = Vec2f(0.0, PI);
	Vec2f right_lane_marker = Vec2f(0.0, 0.0);

	while (it!=lines.end()) {
			float rho= (*it)[0];   // first element is distance rho
			float theta= (*it)[1]; // second element is angle theta

				if ( 0 < theta < (PI/2 - 0.1) && theta < left_lane_marker[1]) {
					left_lane_marker = Vec2f(rho,theta);
				}
				if ((PI/2 + 0.1) < theta < (PI - 0.1) && theta > right_lane_marker[1]) {
					right_lane_marker = Vec2f(rho,theta);
				}

			// This code can be uncommented to display all of the lines found with the Hough Line Transform
			// Point pt1(rho/cos(theta),0);
			// Point pt2((rho-imageROI.rows*sin(theta))/cos(theta),imageROI.rows);
			// line(image, pt1, pt2, Scalar(0,0,255), 1);
			++it;
	}

	// ATTN: need to have imageROI.rows in numerator because of trig. It isnt an oversight!
	// Lines are drawn on the image, not the region of interest.

	// Update our steering based on the lane markers
	double angle_average = (left_lane_marker[1]+right_lane_marker[1])/2;
	double newSteeringAmount = -20*PI*(fabs(angle_average - PI/2))+50;
	sdcSensorData::UpdateSteeringMagnitude(newSteeringAmount);

	//draw left lane marker
	Point leftp1 = Point(left_lane_marker[0]/cos(left_lane_marker[1]),0.5*image.rows);
	Point leftp2 = Point((left_lane_marker[0] - (imageROI.rows) * sin(left_lane_marker[1])) / cos(left_lane_marker[1]), (image.rows));
	line(image, leftp1, leftp2, Scalar(255), 3);

	//draw right lane marker
	Point rightp1 = Point(right_lane_marker[0]/cos(right_lane_marker[1]),0.5*image.rows);
	Point rightp2 = Point((right_lane_marker[0] - (imageROI.rows) * sin(right_lane_marker[1])) / cos(right_lane_marker[1]), (image.rows));
	line(image, rightp1, rightp2, Scalar(255), 3);
// END STRAIGHT LANE DETECTION

// BEGIN HAAR CASCADE OBJECT DETECTION
	std::vector<Rect> stopSigns;
	//This is the OpenCV built in detector for objects decribed by a cascade xml file
	cpu_stop_sign.detectMultiScale( image, stopSigns, 1.1, 2, 0|CASCADE_SCALE_IMAGE, Size(30, 30) );

	//Make sure stop sign is detected across multiple consecutive frames
	if(stopSigns.size() > 0){
		sdcSensorData::stopSignFrameCount++;
	}else{
		sdcSensorData::stopSignFrameCount = 0;
	}

	double avgSize = 0;
	if(stopSigns.size() > 0){
		for( int i = 0; i < stopSigns.size(); i++ ){

			//check if area of detected stop sign contains any red or not
		   cv::rectangle(image, stopSigns[i], Scalar(0,0,255),3,LINE_8,0);
		   avgSize += stopSigns[i].width * stopSigns[i].height;
		}
		avgSize = avgSize / stopSigns.size();
	}
	if(stopSigns.size() > 0){
		sdcSensorData::sizeOfStopSign = avgSize;
	}else{
		sdcSensorData::sizeOfStopSign = 0;
	}
// END HAAR CASCADE OBJECT DETECTION

// BEGIN LCF LANE DETECTION
	// This algorithm was decribed in the paper "A lane-curve detection based on an LCF"
	// Where possible, we have kept the variables named the same as described by the paper.

	double leftNearLaneSlope, rightNearLaneSlope, leftLaneIntercept, rightLaneIntercept;
	double a, b, c, d, e, n, u, v, k, lane_midpoint, eps = 200.0;

	std::vector<double> vec_of_i_vals(50);
	std::iota(std::begin(vec_of_i_vals), std::end(vec_of_i_vals), -25);

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

	// Calculate y-intercepts of the lanes
	//b = y - mx
	leftLaneIntercept = leftp1.y - leftNearLaneSlope*leftp1.x;
	rightLaneIntercept = rightp1.y - rightNearLaneSlope*rightp1.x;

	// SOLVE VANISHING POINT (u,v), which is intersection of two lines
	//v is the height component of the vanishing point, described as vp = (u,v)

	u = abs((leftLaneIntercept - rightLaneIntercept) / (leftNearLaneSlope - rightNearLaneSlope));
	v = (leftNearLaneSlope * u) + leftLaneIntercept;
	Point vp = Point(u,v);
	circle(image,vp, 4, Scalar(30,255,0), 3);

	lane_midpoint = (leftp2.x + rightp2.x)/2;

	// n is our position relative to the center of the lane directly in front of us.
	n = (image.cols/2) - lane_midpoint;

	// Push data to brain
	sdcSensorData::UpdateCameraData(n);

	// Display midpoint of current lane
	line(image, Point(lane_midpoint, 480), Point(lane_midpoint, 0), Scalar(0, 255, 0), 1);
	line(image, Point(lane_midpoint, 480), vp, Scalar(0, 255, 0), 1);


	/////////////////////////////////////////////////////////////////////////////////////////////////////////
	/////////////////////////////////////LEFT LANE MARKER CALCULATION////////////////////////////////////////
	/////////////////////////////////////////////////////////////////////////////////////////////////////////

	Point nfa_p1, nfa_p2, ffa_p1, ffa_p2;

	a = leftNearLaneSlope/2;
	b = (v+leftLaneIntercept)/2;
	c = pow(leftNearLaneSlope,2)/4;
	d = a * (leftLaneIntercept-v);

	//DRAW LEFT NEAR FIELD AND FAR FIELD ASYMPTOTES

	ffa_p1.x = 0.;
	ffa_p2.x = u; // Cx, center of computer image
	ffa_p1.y = v; //Cy, center of computer image
	ffa_p2.y = v;

	nfa_p1.x = 0.;
	nfa_p2.x = u;
	nfa_p1.y = (leftNearLaneSlope)*(nfa_p1.x) + leftLaneIntercept;
	nfa_p2.y = (leftNearLaneSlope)*(nfa_p2.x) + leftLaneIntercept;

	line(image, nfa_p1, nfa_p2, Scalar(255,255,0), 1, CV_AA);
	line(image, ffa_p1, ffa_p2, Scalar(255,255,0), 1, CV_AA);
	double left_max_score = 0, left_optimal_i = 0;

	//generate curvature list
	for (std::vector<double>::const_iterator q = vec_of_i_vals.begin(); q != vec_of_i_vals.end(); q++) {
		// calculate e as decribed in Park et al. 2001
		e = pow((b-v),2) + (eps * a * *q);

		std::vector<Point> left_curve_points_top, left_curve_points_bot;

		for (int x = 0; x < u; x++ ) {
			double left_y_top = (a * x) + b - sqrt( c*pow(x,2) + (d * x) + e);
			double left_y_bot = (a * x) + b + sqrt( c*pow(x,2) + (d * x) + e);

			if(left_y_top >= v) {
					Point left_curve_point_top = Point(x,left_y_top);
					left_curve_points_top.push_back(left_curve_point_top);
			}

			if(left_y_bot >= v) {
					Point left_curve_point_bot = Point(x,left_y_bot);
					left_curve_points_bot.push_back(left_curve_point_bot);
			}
		}

		// Not necessarily needed since we examine the bottom half of the LCF
		// but this is the section that Park et al. 2001 uses for scoring

		// if(left_curve_points_top.size() > 1) {
		// 	for (int j = 0; j < left_curve_points_top.size() - 1; j++) {
		// 		line(image, left_curve_points_top[j], left_curve_points_top[j + 1], Scalar(0,0,255), 1, CV_AA);
		// 	}
		// }

		double left_curve_magnitude = 0;
		if(left_curve_points_bot.size() > 1) {
			for (int j = 0; j < left_curve_points_bot.size(); j++) {
				//This is where we would do LROI calculations
				//LROI is a triangle, the top point is height of vanishing point.
				//LROI is only calculated for the bottom half of the LCF

				//Uncomment to show all of the parabolic paths considered
				//line(image, left_curve_points_bot[j], left_curve_points_bot[j + 1], Scalar(0,0,255), 1, CV_AA);
				int xf,yf;
				int g;
				xf = left_curve_points_bot[j].x;
				yf = left_curve_points_bot[j].y;

				g = contours_thresh.at<unsigned char>(yf-240,xf);
				double bin_g = g/255.;
				left_curve_magnitude += bin_g;

			}
		}

		// Make sure to update the optimal i value if the current curve's score is higher
		// than the current high score

		double left_curve_magnitude_total = (left_curve_points_bot.size() - left_curve_magnitude);
		if (left_curve_magnitude_total > left_max_score) {
			left_max_score = left_curve_magnitude_total;
			left_optimal_i = *q;
		}
	}

	//DISPLAY LEFT LANE CURVE WITH OPTIMAL CURVATURE VALUE
	e = pow((b-v),2) + (eps * a * left_optimal_i);
	std::vector<Point> left_curve_points_top, left_curve_points_bot;

	for (float x = 0.; x < 640.; x++ ) {
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

	if(left_curve_points_top.size() > 1) {
		for (int i = 0; i < left_curve_points_top.size() - 1; i++){
			line(image, left_curve_points_top[i], left_curve_points_top[i + 1], Scalar(0,255,0), 3, CV_AA);
		}
	}

	if(left_curve_points_bot.size() > 1) {
		for (int i = 0; i < left_curve_points_bot.size(); i++){
			line(image, left_curve_points_bot[i], left_curve_points_bot[i + 1], Scalar(0,255,0), 3, CV_AA);
		}
	}

	/////////////////////////////////////////////////////////////////////////////////////////////////////////
	////////////////////////////////////RIGHT LANE MARKER CALCULATION////////////////////////////////////////
	/////////////////////////////////////////////////////////////////////////////////////////////////////////

	a = rightNearLaneSlope/2;
	b = (v+rightLaneIntercept)/2;
	c = pow(rightNearLaneSlope,2)/4;
	d = a * (rightLaneIntercept-v);

	//DRAW RIGHT NEAR FIELD AND FAR FIELD ASYMPTOTES
	ffa_p1.x = 639.;
	ffa_p2.x = u;
	ffa_p1.y = v; //Cy, center of computer image
	ffa_p2.y = v;

	nfa_p1.x = 639.;
	nfa_p2.x = u;
	nfa_p1.y = (rightNearLaneSlope)*(nfa_p1.x) + rightLaneIntercept;
	nfa_p2.y = (rightNearLaneSlope)*(nfa_p2.x) + rightLaneIntercept;

	line(image, nfa_p1, nfa_p2, Scalar(255,255,0), 1, CV_AA);
	line(image, ffa_p1, ffa_p2, Scalar(255,255,0), 1, CV_AA);

	double right_max_score = 0, right_optimal_i = 0;

	//generate curvature list
	for (std::vector<double>::const_iterator q = vec_of_i_vals.begin(); q != vec_of_i_vals.end(); q++) {
		// calculate e as decribed in Park et al. 2001
		e = pow((b-v),2) + (eps * a * *q);

		std::vector<Point> right_curve_points_top, right_curve_points_bot;

		for (float x = u+1; x < 640; x++ ) {
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

		// Not necessarily needed since we examine the bottom half of the LCF
		// but this is the section that Park et al. 2001 uses for scoring

		// if(right_curve_points_top.size() > 1) {
		// 	for (int j = 0; j < right_curve_points_top.size() - 1; j++){
		// 		line(image, right_curve_points_top[j], right_curve_points_top[j + 1], Scalar(0,0,255), 1, CV_AA);
		// 	}
		// }

		double right_curve_magnitude = 0;
		if(right_curve_points_bot.size() > 1) {
			for (int j = 0; j < right_curve_points_bot.size(); j++) {
				//This is where we would do LROI calculations
				//LROI is a triangle, the top point is height of vanishing point.
				//LROI is only calculated for the bottom half of the LCF

				//Uncomment to show all of the parabolic paths considered
				//line(image, right_curve_points_bot[j], right_curve_points_bot[j + 1], Scalar(0,0,255), 1, CV_AA);

				int xf,yf;
				int dg;
				xf = right_curve_points_bot[j].x;
				yf = right_curve_points_bot[j].y;
				dg = contours_thresh.at<unsigned char>(yf-240,xf);
				int bin_dg = dg/255;
				right_curve_magnitude += bin_dg;

			}
		}

		double right_curve_magnitude_total = ((1.*right_curve_points_bot.size()) - right_curve_magnitude);

		// Make sure to update the optimal i value if the current curve's score is higher
		// than the current high score
		if (right_curve_magnitude_total > right_max_score) {
			right_max_score = right_curve_magnitude_total;
			right_optimal_i = *q;
		}
	}

	//DISPLAY RIGHT LANE CURVE WITH OPTIMAL CURVATURE VALUE
	e = pow((b-v),2) + (eps * a * right_optimal_i);
	std::vector<Point> right_curve_points_top, right_curve_points_bot;

	for (float x = 0.; x < 640.; x++ ) {
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

	if(right_curve_points_top.size() > 1) {
		for (int i = 0; i < right_curve_points_top.size() - 1; i++){
			line(image, right_curve_points_top[i], right_curve_points_top[i + 1], Scalar(0,255,0), 3, CV_AA);
		}
	}

	if(right_curve_points_bot.size() > 1) {
		for (int i = 0; i < right_curve_points_bot.size(); i++){
			line(image, right_curve_points_bot[i], right_curve_points_bot[i + 1], Scalar(0,255,0), 3, CV_AA);
		}
	}

// END LCF LANE DETECTION /////////////////////////////
	// Display results to GUI
	namedWindow("Camera View", WINDOW_AUTOSIZE);
	imshow("Camera View", image);
	waitKey(4);
}
