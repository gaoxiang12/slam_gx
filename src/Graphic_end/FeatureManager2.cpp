/* ****************************************
 * FeatureManager2 的实现
 ****************************************/

#include "FeatureManager.h"
#include "const.h"
#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/nonfree/features2d.hpp>
#include <opencv2/flann/flann.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/core/eigen.hpp>
#include <cmath>
#include <fstream>
#include <iostream>
FeatureManager2::FeatureManager2(FeatureGrabberBase* p)
{
    _keyFrame_id = -1;
    _pFeatureGrabber = p;
    
}

FeatureManager2::~FeatureManager2()
{
    
}

void FeatureManager2::Input( vector<KeyPoint>& keypoints, Mat feature_descriptor, SE2& robot_curr, int frame_id)
{
    //Step1, 将关键帧与输入进行两两匹配
    int ransac_success = pairwiseAlign(keypoints, feature_descriptor, robot_curr);

    if (ransac_success == 0)
    {
        //定义新的关键帧
        generate_new_keyframe(frame_id, robot_curr, keypoints, feature_descriptor);
    }
    else
    {
        
    }
}

int FeatureManager2::pairwiseAlign(vector<KeyPoint>& keypoints, Mat feature_descriptor, SE2& robot_curr)
{
    if (_kf_landmarks.size() == 0)
        return 0;
    //首先取出每个关键帧的特征描述子
    Mat kf_desp(_kf_landmarks.size(), feature_descriptor.cols, int(feature_descriptor.type()));
    for (size_t i=0; i<_kf_landmarks.size(); i++)
    {
        _kf_landmarks[i]._descriptor.row(0).copyTo(kf_desp.row(i));
    }

    //将这些描述子与新来的描述子匹配,关键帧中的是query, 新来的是train
    vector<DMatch> matches = match(kf_desp, feature_descriptor);

    //用匹配上的特征来算Ransac
    SE2 new_robot = RANSAC(matches, keypoints);

    return 1;
}

SE2 FeatureManager2::RANSAC( vector<DMatch>& matches, vector<KeyPoint>& new_kp)
{
    //Object & img
    vector<Point3f> obj;
    vector<Point2f> img;
    for (size_t i=0; i<matches.size(); i++)
    {
        obj.push_back( _kf_landmarks[matches[i].queryIdx]._pos_cv );
        img.push_back( new_kp[matches[i].trainIdx].pt );
    }
    double camera_matrix[3][3] = { { camera_fx, 0, camera_cx }, { 0, camera_fy ,camera_cy }, { 0, 0, 1 }};
    Mat cameraMatrix(3,3,CV_64F, camera_matrix);

    Mat rvec, tvec;  //旋转向量与平移向量

    Mat inliers;     //正常值，DA正确的
    solvePnPRansac(obj, img, cameraMatrix, Mat(), rvec, tvec, false, 100, 8.0, 100, inliers);
    
    //将旋转向量转换成旋转矩阵
    Mat R;
    Rodrigues(rvec, R);
    //再转到Eigen
    Eigen::Matrix3f r;
    cv2eigen(R, r);

    //再转换到机器人坐标系
    //首先是平移向量
    Point3f t(-tvec.at<double>(0,0), -tvec.at<double>(0,1), -tvec.at<double>(0,2));
    //转换至g2o坐标系
    Eigen::Vector3d tg = cv2g2o(t);
    //然后是旋转矩阵，注意到cv算的是 [u,v] = RCX，而在g2o中是 z = T^(-1) x，所以需要求逆

    Matrix3f rv = r.inverse();
    Eigen::Vector3f v = rv.eulerAngles(2,0,1);  //转化到欧拉角，主要关心的旋转方向是cv的y轴，代表机器人的旋转角

    SE2 s( tg[0], tg[1], -v[2]);
    return s;
}

int FeatureManager2::generate_new_keyframe(int frame_id, SE2& robot_curr, vector<KeyPoint>& keypoints, Mat feature_descriptor)
{
    cout<<"manager2: generate new frame"<<endl;
    _keyFrame_id = frame_id;
    _kf_landmarks.clear();
    //生成每个特征点的三维坐标，更新关键帧的路标
    for (size_t i=0; i<keypoints.size(); i++)
    {
        Point3f p = _pFeatureGrabber->ComputeFeaturePos( i, robot_curr );
        LANDMARK landmark(0, p, Eigen::Vector3d(0,0,0), feature_descriptor.row(i), 1);
        if (landmark._pos_cv == Point3f(0,0,0))
            continue;
        landmark._pos_g2o = cv2g2o(landmark._pos_cv);
        _kf_landmarks.push_back(landmark);
    }

    return 1;
}
