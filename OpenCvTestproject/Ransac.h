#pragma once
#include <cv.h>
#include<vector>
#include<algorithm>

using namespace std;
#define DEPTH_WIDTH  640
#define DEPTH_HEIGHT 480
#define BUFFSIZE    (DEPTH_WIDTH * DEPTH_HEIGHT)



#define MAX_DEPTH 1500
#define MIN_DEPTH 525

#define RANSAC_THRESHOLD 5
#define INLIER_THRESHOLD 0.50
#define MAXIMUM_TRIALS 1200
#define MAXIMUM_SIM 5


class Ransac
{
public:
	Ransac(void);
	~Ransac(void);
	
struct point
{
    double x;
    double y;
    double z;
};
struct plane_s
{
    point A;
    point B;
    point C;
    point normal;
};
struct Line{
  point A;
  point B;
  double det;
  double coeffX;
  double coeffY;
  double C;
};
int width ;
int height;
CvSeq* ransacLines;
int write_xyz ;
int numOfvisitedPoints;
int trial_runs;

int sub_sampling;  // prime number for sub sampling

char fileName[ 301 ]; //= "depth_frame0.txt";// provide the input depth file name

int npoints_b ;//tracking the number of points in a depth range

double fx_d ;
double fy_d ;
double cx_d ;
double cy_d ;

//point results[BUFFSIZE];
FILE *outFile;
int *imageMapRan;
int num_inliers;
std::vector<int> inliers;
plane_s plane;
Line Lines1,Lines2;
plane_s best_plane;
point vanishPoint;
int best_num_inliers ;
std::vector<int> best_inliers;
int maxDepth;
int minDepth ;

//double depth[BUFFSIZE];
double dist[1000];
long deter;

//double depthLookup[2048];
int found ;
// Local Function Prototypes required by the processings
/*void    depthToWorld(int x, int y, int depthValue, point *result);
double  RawDepthToCM(int depthValue);
void    maxmin_depth();
void    generateDepthLookup( );
int     Filter( int min, int max );
int     is_colinear( int *idx );
int   random_indexes( int *idx, int max );
void    Normal();
void    Ransac();
int    find_lines();
void    find_inliers();
vector<int>* initiateVanishing( CvSeq *line, int *imageMaps );*/


///////////////////////////// End of Declaration//////////////////////////////////////////////


////////////////////////////////////////////////////////// Start of the Defination////////////////////////////////////////////
/*The following function find the max range and min range of the depth image*/

void init(CvSeq *line, int *maps){
    ransacLines = line;
	imageMapRan = maps;
	//if(!best_inliers.empty())
		//best_inliers.clear();
	srand( time( NULL ));
    return;
}
/************************************************************/

vector<int>* initiateVanishing( CvSeq *line, int *imageMaps )
{
   // outFile = fopen("Output.txt", "w+" );
    init(line, imageMaps);
 //   generateDepthLookup();  // Initialize the depth table for scaling purpose
   /* ifstream files( fileName );
    if ( files.is_open() )
    {
        int i;
        for ( i = 0; i < BUFFSIZE; i++)
            files >> depth[i];
        files.close();
    }
    else
    {
        printf( " Unable to open %s.\n", fileName );
        exit(-1);
    }*/

  //  maxmin_depth();
    int simulation = 0;
	best_num_inliers = 0;
    for (simulation = 0; simulation < MAXIMUM_SIM; simulation++)
    {
        found = 0;
       
        npoints_b = 0;
        num_inliers = 0;
		RansacProcess();
       if (num_inliers > 0){
           printf("Model found.\n");
        }
        else{
           printf("No model found.\n");
		   best_inliers.clear();
        }

    }
    //fclose(outFile);
    //FILE *fp = freopen("points.txt","w",stdout);
    //for(int i=0;i<best_num_inliers;i++){
      //printf("%lf %lf %lf\n",results[best_inliers[i]].x,results[best_inliers[i]].y,results[best_inliers[i]].z);
    //}

    //fclose(fp);

	return &best_inliers;
}

void formLines(){
 
	return;
}
////////////////ransac algorithm///////////////////////////////////
/*this function filter the point information using the subsampling then call the find_plane function to choose random three numbers
from the point cloud and find the plane normal to define the plane then it tries to find the inliers for the plane and store the inlieers
if the inliers is greater then the best number of inliers then it is voted as the best model found in this way the whole process run
several times*/
void RansacProcess()
{
    int count = 0;
    int i;
    double prob = 0.90;
    double f = 0.0;  // percentage of inliers
    int N = trial_runs;
    double no_outliers = 2.2204e-16;
	npoints_b = ransacLines->total; //Filter( MIN_DEPTH, MAX_DEPTH  );
	for(i = 0 ; i < npoints_b; i++ )
		if(imageMapRan[i] == 1)
			numOfvisitedPoints++;
	if(abs(npoints_b - numOfvisitedPoints) < 3){
		printf("we cant find any models insufficient data\n");
		return ;
	}
    printf("No. of lines %d\n",npoints_b);
    while ( N > count )
    {
		if( find_lines( )){
            printf("Not possible to find any models insufficient data may be\n");		 
			break;
		}
        find_inliers( );

        if (num_inliers > best_num_inliers)
        {
            ///finding the best plane///////////////////////
            best_num_inliers = num_inliers;
			////FOR PLANES SO NO USE OF IT NOW////////////////
        
            found = count;
			//std:vector<int>::iterator beg = best_inliers.begin();
			//std::vector<int>::iterator ends = best_inliers.end();
			if(!best_inliers.empty() )
			    best_inliers.clear();
			best_inliers.resize(inliers.size());
			copy(inliers.begin(),inliers.end(),best_inliers.begin());
            /*for (i = 0; i < num_inliers; i++)
                best_inliers[i] = inliers[i];*/

            f = (double) num_inliers / npoints_b;

            no_outliers = 1.0 - pow(f,2);
            if ( no_outliers < 2.2204e-16){
                no_outliers = 2.2204e-16;
            }
            else if (  no_outliers > ( 1.0 - 2.2204e-16 ) ){
                no_outliers = ( 1.0 - 2.2204e-16 );
            }

            N = (int) ( log( 1.0 - prob ) / log( no_outliers ) );

        }
        count++;
        if (count > trial_runs){
            printf("RANSAC reached maximum trials of %d.\n", trial_runs );
            break;
        }
		if(!inliers.empty())
			inliers.clear();
    }
	printf("Number of Inliers %d .\n", best_inliers.size() );
}

///////////Add Time Defination//////////////
////////////////////////////////////////////
/*The following function find the inliers
  by implementing the dot products between the normal vector of the plane and vector that lies over the plane and connect the
  one of the base point of the plane and the point under observation
  */
void find_inliers()
{
    int i = 0;
    num_inliers = 0;
	point midPoint;
	double scale,vanishVec;
	point v1,v2;
    for (i = 0; i < npoints_b; i++){
		/////////this is for the plane and 3D operation//////////////////////////////
     
		///////////////////////End of the operation /////////////////////////////////
		if(imageMapRan[i] != 0)
			continue;
		CvPoint* p = (CvPoint*)cvGetSeqElem(ransacLines, i);
		midPoint.x = ((double)(p[0].x + p[1].x))/2;
		midPoint.y = ((double)(p[0].y + p[1].y))/2;
		//scale = midPoint.x*midPoint.x + midPoint.y*midPoint.y;
		v1.x = fabs( p[0].x - midPoint.x);
		v1.y = fabs( p[0].y - midPoint.y );
		v2.x = fabs( vanishPoint.x - midPoint.x );
		v2.y = fabs( vanishPoint.y - midPoint.y );
		scale = sqrt(v1.x*v1.x + v1.y*v1.y);
		v1.x /= scale;
		v1.y /= scale;
		dist[i] = v1.x*v2.x + v1.y*v2.y;
		vanishVec = sqrt(v2.x*v2.x+v2.y*v2.y);
		if ( fabs( dist[i] - vanishVec) < RANSAC_THRESHOLD  )
        {
           // inliers[num_inliers] = i;
			inliers.push_back(i);
			num_inliers++;
        }
    }
}
/************************************************************************************/
/*The following function find randomly select three points and then ensure that they are not colinear then find the normal by calling
the Normal() correspond to the plane formed by these three points*/
void lineConstruction(Line *Lines1,CvPoint *p){
	Lines1->A.x = p[0].x;
	Lines1->A.y = p[0].y;
	Lines1->B.x = p[1].x;
	Lines1->B.y = p[1].y;
	Lines1->coeffX = Lines1->B.y - Lines1->A.y;
	Lines1->coeffY = Lines1->A.x - Lines1->B.x;
	Lines1->C = Lines1->coeffX*Lines1->A.x + Lines1->coeffY*Lines1->A.y;
}
int find_lines( )
{
    int idx[3];
    int count = 0;
    do
    {
       if(random_indexes( idx, npoints_b ))
		   return 1;
        count++;
    } while( is_colinear( idx ) && count < 100 );

    if (count >= 100)
    {
        printf( "unable to find  random points.\n");
		return 1;
       // exit (-1);
    }

	CvPoint* p1 =(CvPoint*) cvGetSeqElem(ransacLines,idx[0]); 
	CvPoint* p2 =(CvPoint*) cvGetSeqElem(ransacLines,idx[1]);
	lineConstruction(&Lines1, p1);
	lineConstruction(&Lines2, p2);
	//////////////this is for the plane operation and 3D operations//////////////////////////
	
    Normal();
	return 0;
}

int random_indexes( int *idx, int max )
{
    int i;
	int countS;
	int low = 0;
	int high = max;
	//srand((unsigned int)time(0));
	int range = (high - low);
	int rand_number;
	for (i = 0; i < 2; i++ ){// as we are looking for lines range is being changed from 3 to 2
		countS = 0;
		do{
		  rand_number = low + int(range*rand()/(RAND_MAX + 1.0));
		  idx[ i ] = rand() % max;//rand_number;//randomGenerator(0, max);//rand()% max;
		  countS++;
		}while(imageMapRan[ idx[ i ] ] != 0 && countS < 3*npoints_b);
	//	if(countS >= 3*npoints_b)
	//		return 1;
	}
	return 0;
}
/*following function try to find out the colineariy of three points by triangulation technique*/
int is_colinear( int *idx )
{
    double AB;
    double AC;
    double BC;

    //point A;
   // point B;
    //point C;
    if(imageMapRan[idx[0]] != 0 && imageMapRan[idx[1]] !=0)
		return 1;
    if (idx[0] == idx[1]) //|| idx[0] == idx[2] || idx[1] == idx[2] ) as line contains only two points 
        return 1;

	CvPoint* p1= (CvPoint*)cvGetSeqElem(ransacLines, idx[0] ); // for lines we need only two points
    CvPoint* p2= (CvPoint*)cvGetSeqElem(ransacLines, idx[1] );
	long A =  p1[1].y - p1[0].y;
	long B =  p1[0].x - p1[1].x;
    long C =  p2[1].y - p2[0].y;
	long D =  p2[0].x - p2[1].x;
	deter = A*D - B*C;
	if(deter == 0)
		return 1;
	else
		return 0;
    //C = results[ idx[2] ];

//////////////this is for the planes////////////////////////////
   /* AB = sqrt( ((B.x - A.x) *  (B.x - A.x ))  + ((B.y - A.y) *  (B.y - A.y )) + ((B.z - A.z) *  (B.z - A.z )));
    AC = sqrt( ((C.x - A.x) *  (C.x - A.x ))  + ((C.y - A.y) *  (C.y - A.y )) + ((C.z - A.z) *  (C.z - A.z )));
    BC = sqrt( ((C.x - B.x) *  (C.x - B.x ))  + ((C.y - B.y) *  (C.y - B.y )) + ((C.z - B.z) *  (C.z - B.z )));

    double sum = AB + AC;

    if ( sum == BC )
        return 1;
    else if((AB+BC)==AC)
        return 1;
    else if((AC+BC)==AB)
        return 1;
    else
        return 0;*/
}
/*find the normal and scale it to get the unit vector along the normal components*/
void Normal()
{
  //  double AB[3];
   // double AC[3];
   // double n1[3];
    //double scale = 1;
	double determinant = Lines1.coeffX*Lines2.coeffY - Lines1.coeffY*Lines2.coeffX; 
	double x1 = ((double)(Lines2.coeffY*Lines1.C - Lines1.coeffY*Lines2.C))/determinant;
	double y1 = ((double)(Lines1.coeffX*Lines2.C - Lines2.coeffX*Lines1.C))/determinant;

	if(fabs(x1 - floor(x1))<0.5)
		vanishPoint.x =floor( x1);
	else
		vanishPoint.x = ceil(x1);
	vanishPoint.y = fabs(y1 - floor(y1))<0.5 ? floor(y1) : ceil(y1);
//////////////this is for the plane part and three dimension//////////////////////////////////


}
/*Implementation of subsampling */
/*int Filter( int min, int max )
{
    int counter = 0;
    int index = 0;
    for (int i = 0; i < height; i++)
    {
        for (int j = 0; j < width; j++)
        {
            if ( depth[index] < max  && depth[index] > min )
            {
                if (!(index % sub_sampling))
                {
                    depthToWorld( j, i, depth[index], &(results[counter]) );
                    counter++;
                }
            }
            index++;
        }
    }
    return counter;
}*/
/*****************************************************************************/
/*The following sequence of functions are responsible for mapping the depth information into relative xyz coordinate*/
/*void generateDepthLookup()
{
    for (int i = 0; i < 2048; i++)
        depthLookup[i] = RawDepthToCM( i );
}*/

double  RawDepthToCM(int depthValue)
{
    return (depthValue < 2047)  ? (double) (0.1236 * 100 * tan(depthValue / 2842.5 + 1.1863) ) : 0.0;
}

/*void depthToWorld(int x, int y, int depthValue, point *result)
{
    double value =  depthLookup[depthValue];
    result->x = (double)((x - cx_d) * value * fx_d);
    result->y = (double)((y - cy_d) * value * fy_d);
    result->z = (double)(value);
}
*/
/************************************************/

};
