#include "FeatureGrabber.h"
#include "const.h"

#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <fstream>
#include <iostream>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <algorithm>

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

    //对_keypoints按重要性排序
    sort(_keypoints.begin(), _keypoints.end(), CompareKeyPoint());
    
    if (debug_info)
    {
        cout<<"detect "<<_keypoints.size()<<" keypoints."<<endl;
    }
    return _keypoints;
}

Mat FeatureGrabber::GetDescriptors()
{
    Ptr<DescriptorExtractor> descriptor_extractor = DescriptorExtractor::create( _descriptor );
    Mat descriptors;
    descriptor_extractor->compute(_rgb, _keypoints, descriptors);

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
    //先计算在相机坐标下的位置点，z朝前，x朝右，y朝下
    pos.z = d/camera_factor;   //z pos
    if (set_max_depth )
    {
        if (pos.z == 0.0)
            //如果深度图中找不到数据，说明该点可能太远。这里置成最大距离。
            pos.z = max_depth;
    }
    else
    {
        if (pos.z == 0.0)
            return Point3f(0, 0, 0);
    }
    
    pos.x = (kx - camera_cx) * pos.z/camera_fx;      //x pos
    pos.y = (ky - camera_cy) * pos.z/camera_fy;      //y pos

    //转到机器人坐标系，y朝前，x朝右，z朝上
    Eigen::Vector3d pr = cv2g2o(pos);
    
    //加上机器人的旋转和位移，就得到路标点在世界坐标系下的表示，注意要取逆
    //这是个2d的变换，不影响第三个维度，所以最后的z是不动的 
    Eigen::Vector2d pw = robot_pos.inverse() * Eigen::Vector2d(pr[0], pr[1]);

    //再转换到cv的坐标系下
    return g2o2cv( Eigen::Vector3d(pw[0], pw[1], pr[2]) );
}

Eigen::Vector2d FeatureGrabber::GetObservation2d(KeyPoint& kp)
{
    //产生一个2维的观察向量
    float kx = kp.pt.x;
    float ky = kp.pt.y;

    unsigned short d = _dep.at<unsigned short>(round(ky), round(kx));

    //先计算在cv下的坐标
    Point3f pos;
    pos.z = d/camera_factor;   //z pos
    if (set_max_depth )
    {
        if (pos.z == 0.0)
            //如果深度图中找不到数据，说明该点可能太远。这里置成最大距离。
            pos.z = max_depth;
    }
    else
    {
        if (pos.z == 0.0)
            return Eigen::Vector2d(0, 0);
    }
    
    pos.x = (kx - camera_cx) * pos.z/camera_fx;      //x pos
    pos.y = (ky - camera_cy) * pos.z/camera_fy;      //y pos

    //转换到g2o坐标下
    Eigen::Vector3d pr = cv2g2o(pos);

    return Eigen::Vector2d( pr[0], pr[1] );
}
