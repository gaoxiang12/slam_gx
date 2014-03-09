#include "GraphicEnd.h"
#include "const.h"
#include "FeatureGrabber.h"
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <fstream>
#include <vector>
#include <string>

using namespace std;
using namespace cv;

GraphicEnd::GraphicEnd()
{
    pImageReader = new ImageReader(parameter_file_addr);
    pFeatureGrabber = new FeatureGrabber(
                                         pImageReader->GetParameters("detector_name"),
                                         pImageReader->GetParameters("descriptor_name"));

    pFeatureManager = new FeatureManager(atoi(pImageReader->GetParameters("save_if_seen").c_str()),
                                         pFeatureGrabber,
                                         -atoi(pImageReader->GetParameters("del_not_seen").c_str()));
    
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

    return 1;
}

int GraphicEnd::run_once()
{
    _success = false;
    stringstream ss;
    string logfile;

    ofstream fout;
    static ofstream frobot("log/robot.path");
    frobot<<_loops<<": "<<_robot_curr[0]<<", "<<_robot_curr[1]<<", "<<_robot_curr[2]<<endl;
    
    ss<<"log/log_"<<_loops<<".txt";
    ss>>logfile;
    fout.open(logfile.c_str());
    
    if (debug_info)
    {
        cout<<"\n-- GraphicEnd::loop "<<_loops<<" --"<<endl;
        cout<<"graphic end : robot in on "<<_robot_curr[0]<<", "<<_robot_curr[1]<<", rotation = "<<_robot_curr[2]<<endl;
    }
    if (pImageReader->Next() == 0) //存在下一张图片
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
        pFeatureManager->DumpAllLandmarks(fout);
        imshow("slam_gx", image_with_keypoints);
        string s = pImageReader->GetParameters("step_time");
        waitKey(atoi(s.c_str()));
    }
    fout.close();

    _loops++;
    return 1;
}
