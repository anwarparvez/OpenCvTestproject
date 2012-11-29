#include<iostream>
#include "cv.h"
#include "highgui.h"
#include "EdgeFinder.h"

using namespace std;
using namespace cv;
#define FILEPATH "Koala.jpg"

int main()
{
	Mat image=imread("Koala.jpg");
	namedWindow("Image");
	imshow("Image",image);
	
	cv::Mat result;
	cv::flip(image,result,1); // positive for horizontal
							  // 0 for vertical,              
							  // negative for both


	//findEdgesandCorners();

	EdgeFinder ef;
	ef.findEdgesandCorners(FILEPATH);

	cv::namedWindow("Output Image");
	cv::imshow("Output Image", result);

	waitKey();

	return 0;
}