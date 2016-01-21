#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"

#include <iostream>
/*
using namespace cv;
using namespace std;

    int main(int argc, char* argv[]) {
	    // Read input image
	    Mat image= cv::imread(argv[1]);
	    if (!image.data)
	    return 0; 

	   // Canny algorithm
	    Mat contours;
	    Canny(image,contours,50,350);
	    Mat contoursInv;
	    threshold(contours,contoursInv,128,255,THRESH_BINARY_INV);

	    //HOUGH TRANSFORM
	    std::vector<Vec2f> lines;
	    if (houghVote < 1 or lines.size() > 2) { // we lost all lines. reset 
	        houghVote = 200; 
	    }

	    else { 
	    	houghVote += 25;
	    } 


	    while(lines.size() < 5 && houghVote > 0) {
	        HoughLines(contours,lines,1,PI/180, houghVote);
	        houghVote -= 5;
	    }
	    std::cout << houghVote << "\n";
	    Mat result(contours.rows,contours.cols,CV_8U,Scalar(255));
	    image.copyTo(result);

		// Draw the limes
	    std::vector<Vec2f>::const_iterator it= lines.begin();
	    Mat hough(image.size(),CV_8U,Scalar(0));
	    while (it!=lines.end()) {

	        float rho= (*it)[0];   // first element is distance rho
	        float theta= (*it)[1]; // second element is angle theta

	        //if (theta < PI/20. || theta > 19.*PI/20.) { // filter theta angle to find lines with theta between 30 and 150 degrees (mostly vertical)

	            // point of intersection of the line with first row
	            Point pt1(rho/cos(theta),0);        
	            // point of intersection of the line with last row
	            Point pt2((rho-result.rows*sin(theta))/cos(theta),result.rows);
	            // draw a white line
	            line( result, pt1, pt2, Scalar(255), 8); 
	            line( hough, pt1, pt2, Scalar(255), 8);
	        //}

	        //std::cout << "line: (" << rho << "," << theta << ")\n"; 
	        ++it;
	    }

	    //NOW COMPUTE PROBABILISTIC HOUGH LINE TRANSFORM
		// Create LineFinder instance
	    LineFinder ld;

	   // Set probabilistic Hough parameters
	    ld.setLineLengthAndGap(60,10);
	    ld.setMinVote(4);

	   // Detect lines
	    std::vector<Vec4i> li= ld.findLines(contours);
	    Mat houghP(image.size(),CV_8U,Scalar(0));
	    ld.drawDetectedLines(houghP);


	    //bitwise and the two images, add a ROI

*/