#include "FeatureManager.h"
#include "const.h"
#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/nonfree/features2d.hpp>
#include <opencv2/flann/flann.hpp>
#include <opencv2/calib3d/calib3d.hpp>

#include <fstream>
#include <iostream>

using namespace std;

//！重点
//该函数实现了特征的比对和管理
void FeatureManager::Input( vector<KeyPoint>& keypoints, Mat feature_descriptor, SE2 robot_curr)
{
    if (debug_info)
    {
        cout<<"FeatureManager::Collecting input data..."<<endl;
    }
    vector<int> good_landmark_idx;  //可以用作本次机器人位姿计算的路标库索引
    Mat new_feature;                //新来的特征，未在库中见到过
    vector<KeyPoint> good_keypoints;
    
    //step 1: 将库与输出数据比较
    if (debug_info)
    {
        cout<<"Step 1, compare new features to library"<<endl;
    }
    if (_landmark_library.empty() == false)
    {
        Mat landmark_desp(int(_landmark_library.size()), feature_descriptor.cols, int(feature_descriptor.type()));  //路标库的描述子矩阵

        size_t i = 0;
        //取出所有库中的描述子
        for (list<LANDMARK>::iterator iter = _landmark_library.begin();
             iter != _landmark_library.end(); iter++)
        {
            (*iter)._descriptor.row(0).copyTo(landmark_desp.row(i));
            i++;
        }

        //将库与新来的特征匹配
        vector<DMatch> matches = Match(feature_descriptor, landmark_desp);
        vector<bool> good_feature;  //存储每一个新来特征是否被匹配到
        good_feature.resize(feature_descriptor.rows);
        
        //成功的部分用于计算机器人的位置，失败的部分放入缓存
        for(i=0; i<matches.size(); i++)
        {
            good_landmark_idx.push_back(matches[ i ].trainIdx);
            good_feature[ matches[ i ].queryIdx ] = true;
            good_keypoints.push_back(keypoints[matches[i].queryIdx]);
        }
    
        for(i=0; i<good_feature.size(); i++)
        {
            if (good_feature[ i ] == false)
            {
                new_feature.push_back(feature_descriptor.row(good_feature[ i ]));
            }
        }
    }//end of if (_lankmarks_library.empty() == false)
    else
    {
        if (debug_info)
        {
            cout<<"landmark library is empty. All features will be treated as new feature."<<endl;
        }
        new_feature = feature_descriptor;
    }

    //step 2: 用RANSAC将匹配成功的路标用来计算机器人的位置
    if (debug_info)
    {
        cout<<"Step 2: RANSAC"<<endl;
    }
    if (good_landmark_idx.empty()==false)
    {
        RANSAC(good_landmark_idx, good_keypoints);
    }

    //step 3: 用缓存中的特征去比对新来的特征，计算它们的持续时间
    cout<<"step 3, compare new features to buffer"<<endl;
    //取出缓存中所有特征的描述子
    Mat buffer_desp(int(_landmark_buffer.size()), new_feature.cols, feature_descriptor.type());
    size_t i = 0;
    for (list<LANDMARK>::iterator iter = _landmark_buffer.begin();
         iter != _landmark_buffer.end(); iter++)
    {
        (*iter)._descriptor.row(0).copyTo(buffer_desp.row(i));
        i++;
    }

    vector<DMatch> matches = Match(new_feature, buffer_desp);

    //将匹配成功的缓存路标 持续时间＋1
    vector<bool> good_buffer;  //标记缓存中每一个路标是否得到匹配
    vector<bool> good_new_feature; //标记每一个新特征是否被匹配到
    good_buffer.resize(_landmark_buffer.size());
    good_new_feature.resize(new_feature.rows);
    for (i=0; i<matches.size(); i++)
    {
        good_buffer[ matches[ i ].queryIdx ] = true;
        good_new_feature[ matches[ i ].trainIdx ] = true;
    }

    int hitbuffer = 0;
    int moveToLib = 0;
    i=0;
    for (list<LANDMARK>::iterator iter = _landmark_buffer.begin();
         iter != _landmark_buffer.end(); )
    {
        if (good_buffer[ i ] == true){
            hitbuffer++;
            (*iter)._exist_frames++;
        }
        //检查该路标是否超过10帧以上，若是，则丢到路标库中
        if ((*iter)._exist_frames > _save_if_seen )
        {
            _landmark_library.push_back(*iter);
            iter = _landmark_buffer.erase(iter);  //从缓存中删除
            moveToLib++;
        }
        else
            iter++;  //否则就继续往下走
        i++;
    }

    //将未匹配到的新特征作为新路标放到缓存中
    for (i=0; i<good_new_feature.size(); i++)
    {
        if (good_new_feature[ i ] == false)
        {
            LANDMARK landmark(0, Mat(), new_feature.row(i), 1);
            _landmark_buffer.push_back(landmark);
        }
    }

    //end of step 3
    if (debug_info)
    {
        cout<<"\n*** Feature Manager Report ***"<<endl;
        cout<<"landmark library : "<<_landmark_library.size()<<endl;
        cout<<"landmark buffer : "<<_landmark_buffer.size()<<endl;
        cout<<"Hit "<<good_landmark_idx.size()<<" landmarks in lib"<<endl;
        cout<<"Hit "<<hitbuffer<<" landmarks in buffer"<<endl;
        cout<<"Add "<<moveToLib<<" landmark from buffer to lib"<<endl;
        cout<<"Add "<<new_feature.rows-hitbuffer<<" new landmarks to buffer"<<endl;
        cout<<"*** Feature Manager Report ***\n"<<endl;
    }
}

//用RANSAC架构求解PnP问题
void FeatureManager::RANSAC(vector<int>& good_landmark_idx, vector<KeyPoint>& keypoints)
{
    // 构造目标点的序列
    Mat objectPoints(good_landmark_idx.size(), 3, CV_64FC1);
    
    for (size_t i=0; i<good_landmark_idx.size(); i++)
    {
        //Note: 由于list元素不能随机访问，所以这里只能用这种效率低下的做法
        //期待后面有所改进吧
        list<LANDMARK>::iterator iter = _landmark_library.begin();
        for (int ix=0; ix<good_landmark_idx[i]; ix++)
            iter++;
        LANDMARK t = *iter;
        t._pos.row(0).copyTo(objectPoints.row(i));
    }

    // 构造图像点的序列
    vector<Point2f> imagePoints;
    for (size_t i=0; i<keypoints.size(); i++)
    {
        imagePoints.push_back(keypoints[i].pt);
    }

    double camera_matrix[3][3] = { { camera_fx, 0, camera_cx }, { 0, camera_fy ,camera_cy }, { 0, 0, 1 }};
    Mat cameraMatrix(3,3,CV_64F, camera_matrix);

    Mat rvec, tvec;  //旋转向量与平移向量

    solvePnPRansac(objectPoints, imagePoints, cameraMatrix, NULL, rvec, tvec);

    if (debug_info)
    {
        cout<<"RANSAC results: "<<endl;
        cout<<"rvec = "<<rvec<<endl;
        cout<<"tvec = "<<tvec<<endl;
    }
}

void FeatureManager::ReportStatus()
{
    //通过标准输出流向外界报告自身的状态
    ofstream fout("FeatureManager.txt");
    fout<<"*** Feature Manager Report ***"<<endl;
    fout<<"landmark library : "<<_landmark_library.size()<<endl;
    fout<<"landmark buffer : "<<_landmark_buffer.size()<<endl;
    fout.close();
}

//将参数中的描述子与_landmark_desp中存储的描述子进行对比，使用FLANN
vector<DMatch> FeatureManager::Match(Mat des1, Mat des2)
{
    if (debug_info)
    {
        cout<<"FeatureManager::Start matching..."<<endl;
    }
    
    FlannBasedMatcher matcher;
    vector<DMatch> matches;

    if (des1.empty() || des2.empty())
    {
        return matches;
    }
    
    matcher.match( des1, des2, matches);
    double max_dist = 0, min_dist = 100;
    for (int i=0; i<des1.rows; i++)
    {
        double dist = matches[ i ].distance;
        if (dist < min_dist)
            min_dist = dist;
        if (dist > max_dist)
            max_dist = dist;
    }

    if (debug_info)
    {
        cout<<"Match:: max dist = "<<max_dist<<", min dist = "<<min_dist<<endl;
    }

    //choose good matches
    vector<DMatch> good_matches;
    
    for (int i=0; i<des1.rows; i++)
    {
        if (matches[ i ].distance <= max(2*min_dist, match_min_dist))
        {
            good_matches.push_back(matches[ i ]);
        }
    }

    if (debug_info)
    {
        cout<<"total good matches : "<<good_matches.size()<<endl;
    }
    return good_matches;
}
