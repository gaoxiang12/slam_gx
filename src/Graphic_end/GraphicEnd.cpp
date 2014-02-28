#include "GraphicEnd.h"
#include "const.h"
#include "FeatureGrabber.h"
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>

#include <vector>

using namespace std;
using namespace cv;

GraphicEnd::GraphicEnd()
{
    pImageReader = new ImageReader(parameter_file_addr);
    pFeatureGrabber = new FeatureGrabber(pImageReader->GetParameters("feature_name"));

    if (vision == true)
    {
        namedWindow("slam_gx");
    }
    
}

GraphicEnd::~GraphicEnd()
{
    delete pImageReader;
    delete pFeatureGrabber;
}

int GraphicEnd::run()
{
    if (debug_info)
    {
        cout<<"calling GraphicEnd::run()..."<<endl;
    }

    while (pImageReader->IsEnd() == false )
    {
        if (pImageReader->Next() == 0)
            break;
        Mat rgb = pImageReader->GetRGB();
        Mat dep = pImageReader->GetDep();
        pFeatureGrabber->SetRGBDep(rgb, dep);
        vector<KeyPoint> keyPoints = pFeatureGrabber->GetKeyPoints();
        Mat desc = pFeatureGrabber->GetDescriptors();
        if (vision == true)
        {
            Mat image_with_keypoints;
            drawKeypoints(rgb, keyPoints, image_with_keypoints, Scalar::all(-1), 0);
            imshow("slam_gx", rgb);
            waitKey(10000);
        }

    }// end of while
    return 1;
}

int GraphicEnd::run_once()
{
    if (debug_info)
    {
        cout<<"calling GraphicEnd::run_once()..."<<endl;
    }
    if (pImageReader->Next() == 0)
        return 0;
    Mat rgb = pImageReader->GetRGB();
    Mat dep = pImageReader->GetDep();
    pFeatureGrabber->SetRGBDep(rgb, dep);
    vector<KeyPoint> keyPoints = pFeatureGrabber->GetKeyPoints();
    Mat desc = pFeatureGrabber->GetDescriptors();
    if (vision == true)
    {
        Mat image_with_keypoints;
        drawKeypoints(rgb, keyPoints, image_with_keypoints, Scalar::all(-1), 0);
        imshow("slam_gx", image_with_keypoints);
        waitKey(10000);
    }

    
    return 1;
}
