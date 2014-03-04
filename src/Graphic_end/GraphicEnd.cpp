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
    pFeatureGrabber = new FeatureGrabber(
                                         pImageReader->GetParameters("detector_name"),
                                         pImageReader->GetParameters("descriptor_name"));
    pFeatureManager = new FeatureManager(3, pFeatureGrabber);
    
    if (vision == true)
    {
        namedWindow("slam_gx");
    }

    _loops = 0;
}

GraphicEnd::~GraphicEnd()
{
    delete pImageReader;
    delete pFeatureGrabber;
    delete pFeatureManager;
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
        cout<<"GraphicEnd::loop"<<_loops<<"..."<<endl;
    }
    if (pImageReader->Next() == 0)
        return 0;
    //读取rgb图与深度图    
    Mat rgb = pImageReader->GetRGB();
    Mat dep = pImageReader->GetDep();
    pFeatureGrabber->SetRGBDep(rgb, dep);

    //抓取当前图形的特征点与描述子
    vector<KeyPoint> keyPoints = pFeatureGrabber->GetKeyPoints();
    Mat desc = pFeatureGrabber->GetDescriptors();

    //将当前图像的特征与机器人位置传送至特征数据库
    pFeatureManager->Input(keyPoints, desc, _robot_curr);
    
    
    if (vision == true)
    {
        Mat image_with_keypoints;
        drawKeypoints(rgb, keyPoints, image_with_keypoints, Scalar::all(-1), 0);
        imshow("slam_gx", image_with_keypoints);
        waitKey(10000);
    }

    _loops++;
    return 1;
}
