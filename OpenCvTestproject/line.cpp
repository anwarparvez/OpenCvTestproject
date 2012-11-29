/*------------------------------------------------------------------------------------------*\
   This file contains material supporting chapter 7 of the cookbook:  
   Computer Vision Programming using the OpenCV Library. 
   by Robert Laganiere, Packt Publishing, 2011.

   This program is free software; permission is hereby granted to use, copy, modify, 
   and distribute this source code, or portions thereof, for any purpose, without fee, 
   subject to the restriction that the copyright notice may not be removed 
   or altered from any source or altered source distribution. 
   The software is released on an as-is basis and without any warranties of any kind. 
   In particular, the software is not guaranteed to be fault-tolerant or free from failure. 
   The author disclaims all warranties with regard to this software, any use, 
   and any consequent failure, is purely the responsibility of the user.
 
   Copyright (C) 2010-2011 Robert Laganiere, www.laganiere.name
\*------------------------------------------------------------------------------------------*/

#include <iostream>
#include <vector>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

#include "linefinder.h"


#define PI 3.1415926

int main()
{
	// Read input image
	cv::Mat image= cv::imread("Tulips.jpg",0);
	if (!image.data)
		return 0; 

    // Display the image
	cv::namedWindow("Original Image");
	cv::imshow("Original Image",image);
/*
	// Compute Sobel
	EdgeDetector ed;
	ed.computeSobel(image);

    // Display the Sobel orientation
	cv::namedWindow("Sobel (orientation)");
	cv::imshow("Sobel (orientation)",ed.getSobelOrientationImage());
	cv::imwrite("ori.bmp",ed.getSobelOrientationImage());

    // Display the Sobel low threshold
	cv::namedWindow("Sobel (low threshold)");
	cv::imshow("Sobel (low threshold)",ed.getBinaryMap(125));

    // Display the Sobel high threshold
	cv::namedWindow("Sobel (high threshold)");
	cv::imshow("Sobel (high threshold)",ed.getBinaryMap(350));
*/
	// Apply Canny algorithm
	cv::Mat contours;
	cv::Canny(image,contours,125,350);
	cv::Mat contoursInv;
	cv::threshold(contours,contoursInv,128,255,cv::THRESH_BINARY_INV);

    // Display the image of contours
	cv::namedWindow("Canny Contours");
	cv::imshow("Canny Contours",contoursInv);

	// Create a test image
	cv::Mat test(200,200,CV_8U,cv::Scalar(0));
	cv::line(test,cv::Point(100,0),cv::Point(200,200),cv::Scalar(255));
	cv::line(test,cv::Point(0,50),cv::Point(200,200),cv::Scalar(255));
	cv::line(test,cv::Point(0,200),cv::Point(200,0),cv::Scalar(255));
	cv::line(test,cv::Point(200,0),cv::Point(0,200),cv::Scalar(255));
	cv::line(test,cv::Point(100,0),cv::Point(100,200),cv::Scalar(255));
	cv::line(test,cv::Point(0,100),cv::Point(200,100),cv::Scalar(255));

    // Display the test image
	cv::namedWindow("Test Image");
	cv::imshow("Test Image",test);
	cv::imwrite("test.bmp",test);

	// Hough tranform for line detection
	std::vector<cv::Vec2f> lines;
	cv::HoughLines(contours,lines,1,PI/180,60);

	// Draw the lines
	cv::Mat result(contours.rows,contours.cols,CV_8U,cv::Scalar(255));
	image.copyTo(result);

	std::cout << "Lines detected: " << lines.size() << std::endl;

	std::vector<cv::Vec2f>::const_iterator it= lines.begin();
	while (it!=lines.end()) {

		float rho= (*it)[0];   // first element is distance rho
		float theta= (*it)[1]; // second element is angle theta
		
		if (theta < PI/4. || theta > 3.*PI/4.) { // ~vertical line
		
			// point of intersection of the line with first row
			cv::Point pt1(rho/cos(theta),0);        
			// point of intersection of the line with last row
			cv::Point pt2((rho-result.rows*sin(theta))/cos(theta),result.rows);
			// draw a white line
			cv::line( result, pt1, pt2, cv::Scalar(255), 1); 

		} else { // ~horizontal line

			// point of intersection of the line with first column
			cv::Point pt1(0,rho/sin(theta));        
			// point of intersection of the line with last column
			cv::Point pt2(result.cols,(rho-result.cols*cos(theta))/sin(theta));
			// draw a white line
			cv::line( result, pt1, pt2, cv::Scalar(255), 1); 
		}

		std::cout << "line: (" << rho << "," << theta << ")\n"; 

		++it;
	}

    // Display the detected line image
	cv::namedWindow("Detected Lines with Hough");
	cv::imshow("Detected Lines with Hough",result);

	// Create LineFinder instance
	LineFinder ld;

	// Set probabilistic Hough parameters
	ld.setLineLengthAndGap(100,20);
	ld.setMinVote(80);

	cv::Mat image2= cv::imread("Tulips.jpg",1);
	// Detect lines
	std::vector<cv::Vec4i> li= ld.findLines(contours);
	ld.drawDetectedLines(image2);
	cv::namedWindow("Detected Lines with HoughP");
	cv::imshow("Detected Lines with HoughP",image2);


	cv::waitKey();
	return 0;
}