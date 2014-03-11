#pragma once

/*****************************************
 * FeatureGrabber: 提取特征的类，用于提取目标图像中的keypoint和descriptor
 * 基类为FeatureGrabberBase, 继承接口供GraphicEnd调用

 ****************************************/

#include "const.h"
#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/nonfree/nonfree.hpp>
#include <g2o/types/slam2d/se2.h>
#include <Eigen/Core>
#include <vector>

using namespace cv;
using namespace std;
using namespace g2o;


class FeatureGrabberBase
{
 public:
    FeatureGrabberBase(string detector, string descriptor) {
        _detector = detector;
        _descriptor = descriptor;
    }
    ~FeatureGrabberBase() {
        
    }
    virtual void SetRGBDep(Mat& rgb, Mat& dep) {
        
    }
    virtual vector<KeyPoint> GetKeyPoints() =0;
    virtual Mat GetDescriptors() =0;
    virtual Point3f ComputeFeaturePos(int index, SE2 robot_pos)=0;
    //根据机器人的位置与相机参数,计算特征点所在位置，返回在openCV下的位置坐标
    
    virtual Point3f ComputeFeaturePos(KeyPoint kp, SE2 robot_pos)=0;
    virtual Eigen::Vector2d GetObservation2d(KeyPoint& kp) =0;
 protected:
    string _detector;
    string _descriptor;
};

//////////////////////////////////////////
// 默认的继承类
class FeatureGrabber: public FeatureGrabberBase
{
 public:
 FeatureGrabber(string detector, string descriptor)
     : FeatureGrabberBase(detector, descriptor)
    {
    }
    virtual void SetRGBDep(Mat& rgb, Mat& dep) {
        _rgb = rgb;
        _dep = dep;
    }
    virtual vector<KeyPoint> GetKeyPoints();
    virtual Mat GetDescriptors();
    virtual Point3f ComputeFeaturePos(int index, SE2 robot_pos);
    virtual Point3f ComputeFeaturePos(KeyPoint kp, SE2 robot_pos);
    virtual Eigen::Vector2d GetObservation2d(KeyPoint& kp);
 protected:
    vector<KeyPoint> _keypoints;
    Mat _rgb, _dep;
};

//为了能够对keypoint进行排序，需要定义一个比较类
class CompareKeyPoint
{
 public:
    bool operator() (const KeyPoint& kp1, const KeyPoint& kp2)
    {
        return kp1.response < kp2.response;
    }
};

//为了使用unique()，定义关键点的位置比较类
class CompareKeyPointEqualPosition
{
 public:
    bool operator() (const KeyPoint& kp1, const KeyPoint& kp2)
    {
        return kp1.pt == kp2.pt;
    }
};
