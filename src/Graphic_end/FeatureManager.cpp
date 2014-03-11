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

using namespace std;

//！重点
//该函数实现了特征的比对和管理
void FeatureManager::Input( vector<KeyPoint>& keypoints, Mat feature_descriptor, SE2& robot_curr)
{
    _success = false;
    if (debug_info)
    {
        cout<<"FeatureManager::Collecting input data..."<<endl;
    }
    vector<int> good_landmark_idx;  //可以用作本次机器人位姿计算的路标库索引
    Mat new_feature;                //新来的特征，未在库中见到过
    vector<KeyPoint> good_keypoints;
    vector<KeyPoint> new_keypoints;
    
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
        //注意是拿新来的特征作为query，仓库中的作为train
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
                //未和库中特征匹配到
                new_feature.push_back(feature_descriptor.row(i));
                new_keypoints.push_back(keypoints[i]);
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
        new_keypoints = keypoints;
    }

    //step 2: 用RANSAC将匹配成功的路标用来计算机器人的位置
    if (debug_info)
    {
        cout<<"Step 2: RANSAC"<<endl;
    }
    if (good_landmark_idx.empty()==false)
    {
        _match_idx = good_landmark_idx;
        _match_keypoints = good_keypoints;
        
        SE2 robot_new = RANSAC(good_landmark_idx, good_keypoints);
        if (_success == false )
        {
            if (debug_info)
            {
                cout<<"RANSAC failed 'cause inlier is to few"<<endl;
                return;
            }
        }
        //检查新的位置与原先的位置是否差的太多
        Vector3d old_r = robot_curr.toVector(), new_r = robot_new.toVector();
        double d = fabs(old_r[0]-new_r[0]) + fabs(old_r[1]-new_r[1]) + fabs(old_r[2]-new_r[2]);
        if (d < max_pos_change && (new_r != Vector3d(0,0,0)))
        {
            if (debug_info)
            {
                cout<<"RANSAC success"<<endl;
                cout<<"before : robot in on "<<robot_curr[0]<<", "<<robot_curr[1]<<", rotation = "<<robot_curr[2]<<endl;
                cout<<"after: robot is on "<<robot_new[0]<<", "<<robot_new[1]<<", rotation = "<<robot_new[2]<<endl;
                _success = true;
            }
            //在这里直接进行更改并不是太好，建议挪至slam端进行
            robot_curr = robot_new;
        }
        else
        {
            if (debug_info)
            {
                cout<<"RANSAC failed 'cause delta is too large"<<endl;
                cout<<"before : robot in on "<<robot_curr[0]<<", "<<robot_curr[1]<<", rotation = "<<robot_curr[2]<<endl;
                cout<<"after: robot is on "<<robot_new[0]<<", "<<robot_new[1]<<", rotation = "<<robot_new[2]<<endl;
            }
        }

        
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

    vector<DMatch> matches = Match(new_feature, buffer_desp); //将新来特征与缓存中的描述子匹配

    //将匹配成功的缓存路标 持续时间＋1, 未匹配到的路标持续时间－1
    vector<bool> good_buffer;  //标记缓存中每一个路标是否得到匹配
    vector<bool> good_new_feature; //标记每一个新特征是否被匹配到

    good_buffer.resize(_landmark_buffer.size());
    good_new_feature.resize(new_feature.rows);
    
    for (i=0; i<matches.size(); i++)
    {
        good_buffer[ matches[ i ].trainIdx ] = true;
        good_new_feature[ matches[ i ].queryIdx ] = true;
    }

    int hitbuffer = 0;
    int moveToLib = 0;
    int delete_from_buffer=0;
    i=0;
    for (list<LANDMARK>::iterator iter = _landmark_buffer.begin();
         iter != _landmark_buffer.end(); )
    {
        if (good_buffer[ i ] == true){
            //成功匹配
            hitbuffer++;
            (*iter)._exist_frames++;
        }
        else
        {
            //未匹配
            //if (iter->_exist_frames <= 0)
                (*iter)._exist_frames--;
        }
        //检查该路标是否超过 _save_if_seen 以上，若是，则丢到路标库中
        if ((*iter)._exist_frames > _save_if_seen )
        {
            if (moveToLib > max_landmark_per_loop)
            {
                //认为该帧中已经有足够的路标，丢弃剩下的路标
                iter = _landmark_buffer.erase(iter);
            }
            else
            {
                //该帧还没有足够的路标，所以这个路标可以从缓存中移动至库中
                _landmark_library.push_back(*iter);
                iter = _landmark_buffer.erase(iter);
                moveToLib++;
            }
        }
        else if (iter->_exist_frames < _delete_if_not_seen)
        {
            // 很长时间未看到此特征，删除之
            iter = _landmark_buffer.erase(iter);
            delete_from_buffer++;
        }
        else
            iter++;  //否则就继续往下走
        i++;
    }

    ofstream kf("kp.txt");
    cout<<"good new feature:"<<good_new_feature.size()<<endl;
    //将未匹配到的新特征作为新路标放到缓存中，首先将计算路标的世界坐标
    int add_landmark_buffer=0;
    for (i=0; i<good_new_feature.size(); i++)
    {
        if (good_new_feature[ i ] == false)
        {
            add_landmark_buffer++;
            //放入缓存中，注意这里是唯一计算位置的地方
            LANDMARK landmark(0, Point3f(0,0,0), Eigen::Vector3d(0,0,0), new_feature.row(i), 1);
            kf<<new_keypoints[i].pt<<endl;
            landmark._pos_cv = _pFeatureGrabber->ComputeFeaturePos(new_keypoints[i], robot_curr);
            if (landmark._pos_cv == Point3f(0.,0.,0.)) //没有深度信息
                continue;
            landmark._pos_g2o = cv2g2o(landmark._pos_cv);
            _landmark_buffer.push_back(landmark);
        }
    }
    kf.close();

    //end of step 3
    if (debug_info)
    {
        cout<<"\n*** Feature Manager Report ***"<<endl;
        cout<<"landmark library : "<<_landmark_library.size()<<endl;
        cout<<"landmark buffer : "<<_landmark_buffer.size()<<endl;
        cout<<"Hit "<<good_landmark_idx.size()<<" landmarks in lib"<<endl;
        cout<<"Hit "<<hitbuffer<<" landmarks in buffer"<<endl;
        cout<<"Add "<<moveToLib<<" landmark from buffer to lib"<<endl;
        cout<<"Add "<<add_landmark_buffer<<" new landmarks to buffer"<<endl;
        cout<<"Delete "<<delete_from_buffer<<" old landmarks from buffer"<<endl;
        cout<<"*** Feature Manager Report ***\n"<<endl;
    }
}

//用RANSAC架构求解PnP问题
//这里使用了OpenCV的 solvePnPRansac函数，所以必须在opencv坐标系下计算，否则会带来问题
SE2 FeatureManager::RANSAC(vector<int>& good_landmark_idx, vector<KeyPoint>& keypoints)
{
    // 构造目标点的序列
    if (debug_info)
    {
        cout<<"RANSAC: total good_landmark is "<<good_landmark_idx.size()<<endl;
        cout<<"RANSAC: total keypoints is "<<keypoints.size()<<endl;
    }
    ofstream fout("ransac_object.txt");
    
    vector<Point3f> objectPoints;  //目标点，有x,y,z三个坐标，世界坐标系下
    
    for (size_t i=0; i<good_landmark_idx.size(); i++)
    {
        //Note: 由于list元素不能随机访问，所以这里只能用这种效率低下的做法
        //期待后面有所改进吧
        LANDMARK t = GetLandmark(good_landmark_idx[i]);
        objectPoints.push_back(t._pos_cv);
        fout<<t._pos_cv<<endl;
    }
    fout.close();

    fout.open("ransac_keypoint.txt");
    // 构造图像点的序列，有u,v两个坐标，在本地坐标系下
    vector<Point2f> imagePoints;
    for (size_t i=0; i<keypoints.size(); i++)
    {
        imagePoints.push_back(keypoints[i].pt);
        fout<<keypoints[i].pt<<endl;
    }
    fout.close();

    if (debug_info)
    {
        cout<<"solving pnp ..."<<endl;
    }

    double camera_matrix[3][3] = { { camera_fx, 0, camera_cx }, { 0, camera_fy ,camera_cy }, { 0, 0, 1 }};
    Mat cameraMatrix(3,3,CV_64F, camera_matrix);

    Mat rvec, tvec;  //旋转向量与平移向量
    _rvec.copyTo(rvec);
    _tvec.copyTo(tvec);

    Mat inliers;     //正常值，DA正确的
    solvePnPRansac(objectPoints, imagePoints, cameraMatrix, Mat(), rvec, tvec, true, 100, 8.0, 100, inliers);

    //如果inlier太少的话，说明ransac是失败的。
    /*
    if (inliers.rows < 5)
    {
        _success = false;
        return SE2(0,0,0);
    }
    
    if (debug_info)
    {
        cout<<"RANSAC: inliers = "<<inliers.rows<<endl;
    }
    */
    _success = true;
    _rvec = rvec;
    _tvec = tvec;
    
    //存储inlier，因为在g2o优化中还会用到
    _inlier.clear();
    _inlier.resize(good_landmark_idx.size()); 

    
    for (int i=0; i<inliers.rows; i++)
    {
        //_inlier[ inliers.at<int>(i, 0) ] = true;
        _inlier[i] = true;   //全部当成inlier，测试g2o的robust核的处理能力
    }
    
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

    if (debug_info)
    {
        cout<<"RANSAC results: "<<endl;
        cout<<"t = "<<s[0]<<", "<<s[1]<<endl;
        cout<<"r = "<<s[2]<<endl;
        cout<<"v = "<<v[0]<<", "<<v[1]<<", "<<v[2]<<endl;
    }
    return s;
    
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
        cout<<"des1 or des2 is empty."<<endl;
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
