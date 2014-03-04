#include "FeatureGrabber.h"
#include "const.h"

#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <fstream>
#include <iostream>
#include <Eigen/Core>
#include <Eigen/Geometry>

using namespace std;
using namespace cv;

vector<KeyPoint> FeatureGrabber::GetKeyPoints()
{
    if (debug_info)
    {
        cout<<"grabbing features, detector is "<<_detector <<endl
            <<"descriptor is "<<_descriptor<<endl;
        if (_rgb.empty())
            cout<<"rgb is empty!"<<_rgb.rows<<","<<_rgb.cols<<endl;
    }
    
    initModule_nonfree();
    
    Ptr<FeatureDetector> detector = FeatureDetector::create(_detector);

    if (detector.empty())
    {
        cerr<<"fail to create detector!"<<endl;
        return _keypoints;
    }
    _keypoints.clear();

    detector->detect(_rgb, _keypoints);

    if (debug_info)
    {
        cout<<"detect "<<_keypoints.size()<<" keypoints."<<endl;
        ofstream fout("keypoint.txt");
        fout<<_keypoints[0].pt<<endl;
        fout.close();

    }
    return _keypoints;
}

Mat FeatureGrabber::GetDescriptors()
{
    if (debug_info)
    {
        cout<<"computing descriptors ... "<<endl;
    }

    Ptr<DescriptorExtractor> descriptor_extractor = DescriptorExtractor::create( _descriptor );
    Mat descriptors;
    descriptor_extractor->compute(_rgb, _keypoints, descriptors);

    if (debug_info)
    {
        cout<<"descriptor size is "<<descriptors.rows<<","<<descriptors.cols<<endl;

        ofstream fout("descriptor.txt");
        fout<<descriptors;
        fout.close();
    }
    
    return descriptors;
}

Point3f FeatureGrabber::ComputeFeaturePos(int index, SE2 robot_pos)
{
    if (index > _keypoints.size())
    {
        cerr<<"ComputeFeaturePos: index out of range. "<<endl;
        return Point3f(0., 0., 0.);
    }

    return this->ComputeFeaturePos(_keypoints[index], robot_pos);
}

Point3f FeatureGrabber::ComputeFeaturePos(KeyPoint kp, SE2 robot_pos)
{
    float kx = kp.pt.x;
    float ky = kp.pt.y;
    
    Point3f pos;
    uchar c;
    unsigned short d = _dep.at<unsigned short>(round(ky), round(kx));
    if (debug_info)
    {
        //cout<<"d = "<<d<<endl;
    }
    //先计算在相机坐标下的位置点，z朝前，x朝右，y朝下
    pos.z = d/camera_factor;   //z pos
    pos.x = (kx - camera_cx) * pos.z/camera_fx;      //x pos
    pos.y = (ky - camera_cy) * pos.z/camera_fy;      //y pos

    //转到机器人坐标系，y朝前，x朝右，z朝上
    Point3f pr;
    pr.y = pos.z;
    pr.x = pos.x;
    pr.z = -pos.y;

    //加上机器人的旋转和位移，就得到路标点在世界坐标系下的表示
    Eigen::Vector2d pw = robot_pos * Eigen::Vector2d(pr.x, pr.y);
    Point3f p(pw[0], pw[1], pr.z);
    return p;
}
