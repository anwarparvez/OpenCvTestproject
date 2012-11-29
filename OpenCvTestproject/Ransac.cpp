#include "Ransac.h"

Ransac::Ransac(void)
{
width       = DEPTH_WIDTH;
 height      = DEPTH_HEIGHT;
ransacLines = NULL;
write_xyz = 1;
numOfvisitedPoints = 0;
trial_runs = MAXIMUM_TRIALS;

sub_sampling = 119;  // prime number for sub sampling



npoints_b = 0;//tracking the number of points in a depth range

fx_d = 1.0 / 5.9421434211923247e+02;
fy_d = 1.0 / 5.9104053696870778e+02;
cx_d = 3.3930780975300314e+02;
cy_d = 2.4273913761751615e+02;



best_num_inliers = 0;

maxDepth=0;
 minDepth =10000;

found = -1;
}

Ransac::~Ransac(void)
{
}
