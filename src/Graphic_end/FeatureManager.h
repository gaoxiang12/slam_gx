/*****************************************
 * Feature Manager 管理得到的特征并进行帧间配准
 * 管理机制详细过程请参见文档
 ****************************************/
#include <opencv2/core/core.hpp>
#include <Eigen/Core>
#include <g2o/types/slam2d/se2.h>
#include <opencv2/features2d/features2d.hpp>
#include <list>

#include "FeatureGrabber.h"

using namespace std;
using namespace cv;
using namespace g2o;

struct LANDMARK
{
    LANDMARK(int ID=0, Mat pos=Mat(), Mat desc=Mat(), int exist=0)
    {
        _ID = ID;
        _pos = pos;
        _descriptor = desc;
        _exist_frames = exist;
    }
    int _ID;
    Mat _pos;
    Mat _descriptor;
    int _exist_frames;   //连续存在的帧数
};

class FeatureManager
{
 public:
    FeatureManager(int save_if_seen = 10, FeatureGrabberBase* p = NULL)
    {
        _pFeatureGrabber = p;
        _save_if_seen = save_if_seen;
    }
        
    void Input(vector<KeyPoint>& keypoints, Mat feature_descriptor, SE2 robot_curr);  //输入当前帧的特征点描述子与机器人位置

    void ReportStatus();
 private:
    //内部函数
    vector<DMatch> Match(Mat des1, Mat des2);
    void RANSAC();
 protected:
    FeatureGrabberBase* _pFeatureGrabber;
    
 protected:
    Mat _landmark_pos;   //路标的位置

    list<LANDMARK> _landmark_buffer;   //缓存
    list<LANDMARK> _landmark_library;  //库

    // 参数定义
    int _save_if_seen;  //多少帧连续看见该特征，则存储之
    
};
