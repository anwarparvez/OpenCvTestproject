#pragma once
#include <cv.h>
#include<iostream>
#include<highgui.h>
using namespace cv;

class Morphological
{
public:
	Morphological(void);
	~Morphological(void);
	IplImage* opening(IplImage *image,int rows,int cols,int shape,int iter);
	IplImage* closing(IplImage *image,int rows,int cols,int shape,int iter);
};
