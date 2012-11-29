#pragma once
#include "cv.h"
#include <vector>
#include <iostream>
#include <limits>
#include "Morphological.h"
#include "Voxel.h"
#include "Ransac.h"
using namespace cv;
using namespace std;

class EdgeFinder
{
public:
	EdgeFinder(void);
	~EdgeFinder(void);

	vector<CvPoint*> verticalLineholder;
vector<CvPoint*> otherLines;
Voxel *voxelGrid;
IplImage* pImage;
bool isVertical;
int* imageMap;

void getEdges(Mat& gray, Mat& grayInt, Mat& grayInt1) {
	Mat _tmp,_tmp1,gray32f,res;

	gray.convertTo(gray32f,CV_32FC1,1.0/255.0);

	GaussianBlur(gray32f,gray32f,Size(11,11),0.75);

	Sobel(gray32f,_tmp,-1,1,0,3);	//sobel for dx
	//Sobel(gray32f,_tmp1,-1,1,0,3,-1.0);	//sobel for -dx
	//_tmp = abs(_tmp) + abs(_tmp1);
	//_tmp.copyTo(_tmp,(_tmp > 0.0));
	//_tmp1.copyTo(_tmp1,(_tmp1 > 0.0));
	_tmp1 = abs(_tmp); // + (_tmp1 == 0.0);
	_tmp1.copyTo(res,(_tmp1 > 0.2));
	//res = -res + 1.0;



	double maxVal,minVal;
	minMaxLoc(_tmp,&minVal,&maxVal);

	cv::log(/*(_tmp - minVal) / (maxVal - minVal)*/res,_tmp);
	_tmp = -_tmp * 0.17;
	//_tmp.convertTo(grayInt1,CV_32SC1);
	res.convertTo(grayInt1,CV_32FC1); 
	Sobel(gray32f,_tmp,-1,0,1,3);	//sobel for dy
	//Sobel(gray32f,_tmp1,-1,0,2,3,-1.0);	//sobel for -dy
	//_tmp = abs(_tmp) + abs(_tmp1);
	//_tmp = (_tmp + _tmp1 + 2.0) / 4.0;
	_tmp1 = abs(_tmp);
	 res.setTo(Scalar(0));
	_tmp1.copyTo(res,(_tmp1 > 0.2));
	//res = -res+1.0;
	
	minMaxLoc(_tmp,&minVal,&maxVal);
	cv::log(/*(_tmp - minVal) / (maxVal - minVal)*/res,_tmp);
	
	_tmp = -_tmp * 0.17;
	//_tmp.convertTo(grayInt,CV_32SC1);
	res.convertTo(grayInt,CV_32FC1);
	for(int i =0; i< grayInt.rows; i++){
		float *temp = grayInt.ptr<float>(i);
		for(int j =0 ; j <grayInt.cols; j++)
			temp[j] = fabs(temp[j]);
	}
	
}
 bool sortOut(CvPoint &p1, CvPoint &p2){
	 return p1.x == p2.x ? p1.y < p2.y : p1.x < p2.x ; 
 }
 bool sortLines(const CvPoint* p1, const CvPoint* p2){
	  
	 //p1[0].x == p2[0].x ? (p1[0].y == p2[0].y ? (p1[1].x == p2[1].x ? p1[1].y > p1[1].y : p1[1].x < p2[1].x) : p1[0].y > p1[1].y) : p1[0].x < p2[0].x;
	 if(p1[0].x < p2[0].x && p1[1].x < p2[1].x)
		 return true;
	 if(p1[0].x > p2[0].x && p1[1].x > p2[1].x)
		 return false;
	 return false;
 }
 
 CvMat * circleOperation(IplImage *inputImage){

	IplImage* hsv = cvCreateImage(cvGetSize(inputImage),IPL_DEPTH_8U,3);
	cvCvtColor(inputImage,hsv,CV_RGB2HSV);
	cvNamedWindow("tempShow");

	CvMat* mask = cvCreateMat(hsv->height,hsv->width,CV_8UC1);
	cvInRangeS(hsv,cvScalar(0.14*256,0.60*256,0.20*256,0),cvScalar(1.0*256,1.0*256,1.0*256,0),mask);

	//cvReleaseImage(&hsv);
		cvShowImage("tempShow",mask);
		cvWaitKey(0);
	IplConvKernel* strucElem1 = cvCreateStructuringElementEx(21,21,10,10,CV_SHAPE_RECT,NULL);
	IplConvKernel* strucElem2 = cvCreateStructuringElementEx(11,11,5,5,CV_SHAPE_RECT,NULL);
	
	/*cvDilate(mask,mask,strucElem2);
	cvErode(mask,mask,strucElem1);*/
	cvErode(mask,mask,strucElem2);
	cvDilate(mask,mask,strucElem2);
	cvReleaseStructuringElement(&strucElem1);
	cvReleaseStructuringElement(&strucElem2);

	
	cvShowImage("tempShow",mask);
	cvWaitKey(0);

	return mask;
}
 void imageMapinit(IplImage *img,int size){
	 
	  /*imageMap = new int*[img->height];
	  for(int in = 0;in<img->height;in++)
		  imageMap[in] = new int[img->width];
	  for(int i =0 ;i<img->height;i++)
		  for(int j =0; j<img->width;j++)
			  imageMap[i][j] = 0;
	  printf("height = %d width = %d\n",img->height, img->width);*/
	 for(int i =0;i<size;i++)
		 imageMap[i] = 0;
 }
 ///////////////////////////////////////////////
 void displayLines(IplImage* imgSource){

	return;
}
 //////////////////////////////////////////////////////
 void imageResize(IplImage *srcImage, IplImage *destImage){

     cvPyrDown(srcImage,destImage);
 	return;
 }
 ////////////////////////////////////////////////////////
 IplImage* getReallocation(IplImage *src, int depth,int nchannel){

 	return cvCreateImage(cvGetSize(src), depth, nchannel);



 }
 //////////////////////////////////////////////////////////////
 int findVerticallines(vector<int> *indexHolder, CvSeq * linez){

	int ux[2] = {1,0};
    int u[2];
	int sum = 0;
	for(int i =0 ;i<indexHolder->size(); i++){
	   CvPoint* points = (CvPoint*)cvGetSeqElem(linez,indexHolder->at(i));
	   u[0] = abs(points[0].x - points[1].x);
	   u[1] = abs(points[0].y - points[1].y);
       sum += u[0]*ux[0] + u[1]*ux[1]; 
	}
	printf("The sum is :%d",sum);
	return sum < indexHolder->size()*10;
}
  void findEdgesandCorners(const char * filename)
 {

	Morphological morph;
	//cvNamedWindow("cornerPoints");
	bool set = false;
    IplImage* img = cvLoadImage(filename); 
	IplImage* imgOnwork = NULL;
    IplImage* out = NULL;
    IplImage* finalOut = NULL;
	vector<int> *indexHolder;
    
	// assert(img->width % 2 == 0 && img->height % 2 == 0);
	if(img->height >500 || img->width > 650){
		
         imgOnwork = cvCreateImage(cvSize(img->width/2,img->height/2),img->depth,img->nChannels);
         imageResize(img,imgOnwork);
		// imageMapinit(imgOnwork);
         set = true;
	
	}else{
		imgOnwork = cvCreateImage(cvGetSize(img),img->depth,img->nChannels);
		//imageMapinit(imgOnwork);
		cvCopyImage(img,imgOnwork);
	}

    //cvReleaseImage(&img);
	out = getReallocation(imgOnwork, imgOnwork->depth,imgOnwork->nChannels);
	finalOut = cvCreateImage(cvGetSize(imgOnwork),imgOnwork->depth, 1);
	IplImage* finalOutSecond = cvCreateImage(cvGetSize(imgOnwork),IPL_DEPTH_8U,1);
	IplImage* histImage = cvCreateImage(cvSize(300,240),8,1);
	cvSet(histImage,cvScalarAll(255),0);
	
	//cornerOut = getReallocation(imgOnwork,img->depth,1);
    if(imgOnwork == NULL || out == NULL || finalOut == NULL){
    	if(!finalOut)
    	printf("We are exiting\n");
    	exit(0);
    }
	//cvSmooth(imgOnwork,out, CV_GAUSSIAN,3,3);// gaussian smoothing may not be the good one to introduce
	cvCopyImage(imgOnwork,out);
	//getTheSharpenedImage(out);
	//cvAdd(imgOnwork,out,out,NULL);
	//  showImage("edgeImages",out);
	 //.. waitKey(0);
    cvCvtColor(out,finalOut,CV_RGB2GRAY);
	//cvSobel(finalOut,finalOutSecond,1,0,3);
	//cvAbs(finalOutSecond,finalOutSecond);
	//cvShowImage("edgeImages",finalOutSecond);
	//waitKey(0);
	//cvCreateHistogram(finalOut,histImage);
	
	
	cvNormalize(finalOut,finalOutSecond,0.0,255.0,CV_MINMAX);
	//cvCreateHistogram(finalOutSecond,histImage);
	
	
	cvReleaseImage(&histImage);
	cvCopy(finalOutSecond,finalOut);
	//finalOutSecond = laplace(finalOutSecond);
	//cvAdd(finalOut,finalOutSecond,finalOut,NULL);
	cvSmooth(finalOut,finalOut,CV_GAUSSIAN,3,3);
	Mat gray1,gray2,gray3;
	Mat LfinalOut = (Mat)finalOut;
	getEdges(LfinalOut,gray1,gray2);
	cv::add(gray1,gray2,gray3);
	//cv::Canny(gray3,gray3,30,100,3);
	gray3.convertTo(gray1,CV_8U,255);
	//cv::Canny(gray1,gray1,30,100);
	IplImage TempGray = (IplImage)gray1;
	finalOut = &TempGray;
	finalOut = morph.closing(finalOut,3,3,0,1);
	//..cvShowImage("edgeImages",finalOut);
	//waitKey(0);
	//cvCanny(finalOut,finalOut,30,100,3);//10,10
	//finalOut = morph.closing(finalOut,3,3,0,1);
	//cvShowImage("edgeImages",finalOut);
	//waitKey(0);
	
	//HoughLineSegment(finalOut);
	IplImage* gray = cvCreateImage(cvGetSize(imgOnwork), IPL_DEPTH_8U, 1);
	cvCvtColor(imgOnwork,gray,CV_RGB2GRAY);
	CvMemStorage* store = cvCreateMemStorage(0);
			//CvMat* mat = cvCreateMat(1,imgOnwork->height*imgOnwork->width,CV_32FC4);
	char c;
	c ='l';// getchar();
	CvSeq* linez = NULL;
	CvSeq* circles = NULL;

		linez = cvHoughLines2(finalOut,store,CV_HOUGH_PROBABILISTIC,1.0,CV_PI/180,100,40.0,20.0);//20,10.0,10.0

	if(linez == NULL || linez->total < 0){
	 printf("Programe is exiting\n");
	 printf("Evacuate now\n");
     exit(0);
	}
	
    int size =  linez == NULL ? circles->total : linez->total;
	printf("linez size %d\n",linez->total);
	//cvReleaseImage(&finalOut);
	
	imageMap = new int[linez->total];
	imageMapinit(imgOnwork,linez->total);
	int trailRunner = 0;
	int colorVal = 255;
	isVertical = false;
	displayLines(finalOut);
	Ransac ransac;
	while(trailRunner++ < 3){
	  //IplImage*  imgForoutput = cvCreateImage(cvGetSize(imgOnwork),imgOnwork->depth,imgOnwork->nChannels);
	  //cvCopyImage(img,imgForoutput);
	  printf("Start Ransac\n");

      indexHolder = ransac.initiateVanishing(linez, imageMap); 
	 
	if(findVerticallines(indexHolder, linez)){
           printf("Found the vertical lines\n");	
		   for(int i = 0; i < indexHolder->size(); i++){
				  CvPoint* pointtoHold = new CvPoint[2];
				  CvPoint* tempHold = (CvPoint*) cvGetSeqElem(linez,indexHolder->at(i));
				  pointtoHold[0] = tempHold[0];
				  pointtoHold[1] = tempHold[1];
				  verticalLineholder.push_back( pointtoHold);
				  cvLine(imgOnwork,verticalLineholder.at(verticalLineholder.size()-1)[0],verticalLineholder.at(verticalLineholder.size()-1)[1],cvScalar(255,0,colorVal),1,CV_AA,0);
		   }
			 
			  isVertical = true;
	 }	  
	 else{
	  for(int i = 0;i<indexHolder->size();i++){
				
		/*float rho = points[0];
		  float theta = points[1];
		  double a = cos(theta); 
		  double b= sin(theta);
		  double x0 = rho*a;double y0 = b*rho;
		  cv::Point ptr1(cvRound(x0 + 50*(-b)),cvRound(y0 + 50*(a)));
		  cv::Point ptr2(cvRound(x0-50*(-b)),cvRound(y0 - 50*(a)));*/
		  CvPoint* tempVarPoint = new CvPoint[2];
		  if(c == 'l' || c == 'L'){
			 CvPoint* points= (CvPoint*) cvGetSeqElem(linez,indexHolder->at(i));
			 tempVarPoint[0] = points[0];
			 tempVarPoint[1] = points[1];
			 otherLines.push_back(tempVarPoint);
			 
			 if(trailRunner == 1)
				 cvLine(imgOnwork,points[0],points[1],cvScalar(0,255,0),1,CV_AA,0);
			 else if (trailRunner == 2)
				 cvLine(imgOnwork,points[0],points[1],cvScalar(255,0,colorVal),1,CV_AA,0);
			 else
				 cvLine(imgOnwork,points[0],points[1],cvScalar(0,127,colorVal),1,CV_AA,0);
		  }
		  else{
				float* floatpoints = (float*) cvGetSeqElem(circles,i);
				CvPoint cvPoints = cvPoint(cvRound(floatpoints[0]),cvRound(floatpoints[1]));
				CvScalar val = cvGet2D(finalOut, cvPoints.y,cvPoints.x);
				if(val.val[0]<1) continue;
				   cvCircle(imgOnwork,cvPoints,cvRound(floatpoints[2]),CV_RGB(255,0,0),1,CV_AA,0);
		  }

			
	  }
	 } 
	  for(int i = 0;i < indexHolder->size(); i++ )
		  imageMap[indexHolder->at(i)] = 1;
	  }

	//cvCvtColor(imgOnwork,cornerOut,CV_RGB2GRAY);
      imshow("edgeImages",(Mat)imgOnwork);
	  cvWaitKey(0);
	 cvReleaseImage(&imgOnwork);
    
	imshow("cornerPoints",(Mat)finalOut);
    
	cvReleaseMemStorage(&store); 
	cvReleaseImage(&out);

 return;

}
};
