
//  main.cpp
//  CompsLaneDetection
//
//  Created by selfcar on 1/20/16.
//  Copyright (c) 2016 selfcar. All rights reserved.
//
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/objdetect/objdetect.hpp"
#include <iostream>

using namespace cv;
using namespace std;

int main(int argc, char* argv[]) {
    // Read input image
    Mat image = cv::imread(argv[1]);
    if (!image.data)
        return 0;
    
    // Load Face cascade (.xml file)
    CascadeClassifier face_cascade;
    //face_cascade.load( "caasdasdasdscade.xml" );
    if ( !face_cascade.load("/Users/selfcar/Desktop/haarcascade_stop.xml") ){ printf("--(!)Error loading\n");}
    // Detect faces
    std::vector<Rect> faces;
    face_cascade.detectMultiScale( image, faces, 1.1, 2, 0|CV_HAAR_SCALE_IMAGE, Size(30, 30) );
    for( int i = 0; i < faces.size(); i++ )
    {
    //std::cout << faces.size();
       Point center( faces[i].x + faces[i].width*0.5, faces[i].y + faces[i].height*0.5 );
       ellipse( image, center, Size( faces[i].width*0.5, faces[i].height*0.5), 0, 0, 360, Scalar( 255, 0, 255 ), 4, 8, 0 );
    }
    imshow( "Detected Face", image );
    
    waitKey(0);
    return 0;
 /*
    Mat imageROI = image(cv::Rect(0, image.rows/2, image.cols, image.rows/2));
    
    // Canny algorithm
    Mat contours;
    Canny(imageROI,contours,50,350);
    Mat contoursInv;
    threshold(contours,contoursInv,128,255,THRESH_BINARY_INV);
    
    //HOUGH TRANSFORM
    int houghVote;
    float PI = 3.14159;
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
    Mat hough(imageROI.size(),CV_8U,Scalar(0));
    //std::cout << image.rows;
   
    while (it!=lines.end()) {
        
        float rho= (*it)[0];   // first element is distance rho
        float theta= (*it)[1]; // second element is angle theta
        
        //if (theta < PI/20. || theta > 19.*PI/20.) { // filter theta angle to find lines with theta between 30 and 150 degrees (mostly vertical)
        
        // point of intersection of the line with first row
        Point pt1(rho/cos(theta),0);
        // point of intersection of the line with last row
        Point pt2((rho-result.rows*sin(theta))/cos(theta),result.rows);
        // draw a white line
        line(result, pt1, pt2, Scalar(255), 3);
        line(imageROI, pt1, pt2, Scalar(255), 3);
        //}
        
        //std::cout << "line: (" << rho << "," << theta << ")\n";
        ++it;
    }
    namedWindow("MyWindow", CV_WINDOW_AUTOSIZE); //create a window with the name "MyWindow"
    
    imshow("MyWindow", image); //display the image which is stored in the 'img' in the "MyWindow" window

    waitKey(0); //wait infinite time for a keypress
    
    destroyWindow("MyWindow"); //destroy the window with the name, "MyWindow"
    destroyWindow("MyWindow2");
    destroyWindow("MyWindow3");
  */
}



/*
 // Load Face cascade (.xml file)
 CascadeClassifier face_cascade;
 face_cascade.load( "cascade.xml" );
 
 // Detect faces
 std::vector<Rect> faces;
 face_cascade.detectMultiScale( image, faces, 1.1, 2, 0|CV_HAAR_SCALE_IMAGE, Size(30, 30) );
 
 // Draw circles on the detected faces
 for( int i = 0; i < faces.size(); i++ )
 {
 Point center( faces[i].x + faces[i].width*0.5, faces[i].y + faces[i].height*0.5 );
 ellipse( image, center, Size( faces[i].width*0.5, faces[i].height*0.5), 0, 0, 360, Scalar( 255, 0, 255 ), 4, 8, 0 );
 }
 
 imshow( "Detected Face", image );
 
 waitKey(0);
 return 0;
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