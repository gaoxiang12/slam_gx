#include "FeatureGrabber.h"
#include "const.h"

#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <fstream>
#include <iostream>
#include <Eigen/Core>

using namespace std;
using namespace cv;

vector<KeyPoint> FeatureGrabber::GetKeyPoints()
{
    if (debug_info)
    {
        cout<<"grabbing features, detector is "<<_detector
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

Eigen::Vector3d FeatureGrabber::ComputeFeaturePos(int index, SE2 robot_pos)
{
    if (index > _keypoints.size())
    {
        cerr<<"ComputeFeaturePos: index out of range. "<<endl;
        return Eigen::Vector3d(0., 0., 0.);
    }

    KeyPoint kp = _keypoints[ index ];
    float kx = kp.pt.x;
    float ky = kp.pt.y;
    
    Eigen::Vector3d pos;
    float d = _dep.at<Vec3f>(ky, kx)[ 0 ];
    pos[ 2 ] = d/camera_factor;   //z pos
    pos[ 0 ] = (kx - camera_cx) * pos[ 2 ]/camera_fx;      //x pos
    pos[ 1 ] = (ky - camera_cy) * pos[ 2 ]/camera_fy;      //y pos

    return pos;
}
