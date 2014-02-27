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

#include <vector>

using namespace cv;
using namespace std;


class FeatureGrabberBase
{
 public:
    FeatureGrabberBase(string featureName) {
        _featureName = featureName;
    }
    ~FeatureGrabberBase() {
        
    }
    virtual void SetRGBDep(Mat& rgb, Mat& dep) {
        
    }
    virtual vector<KeyPoint> GetKeyPoints() =0;
    virtual Mat GetDescriptors() =0;

 protected:
    string _featureName;
};

//////////////////////////////////////////
// 默认的继承类
class FeatureGrabber: public FeatureGrabberBase
{
 public:
 FeatureGrabber(string featureName) : FeatureGrabberBase(featureName)
    {
        _featureName = featureName;
        initModule_nonfree();
    }
    virtual void SetRGBDep(Mat& rgb, Mat& dep) {
        _rgb = rgb;
        _dep = dep;
    }
    virtual vector<KeyPoint> GetKeyPoints();
    virtual Mat GetDescriptors();

 protected:
    vector<KeyPoint> _keypoints;
    Mat _rgb, _dep;
};
