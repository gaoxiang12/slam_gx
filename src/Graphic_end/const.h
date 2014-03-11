/**************************************
 * const.h 定义常量
 * 这只是为了编程方便而在此进行定义，需要经常更改的参数建议移动至parameters.yaml文件，这样就不必重新编译
 *************************************/
#pragma once
#include <string>
#include <opencv2/core/core.hpp>
#include <Eigen/Core>
using namespace cv;
using namespace std;

const bool debug_info = true; //是否输出调试信息
const string parameter_file_addr = "/home/y/code/slam_gx/parameters.yaml";
const bool vision = true;
//////////////////////////////////////////
// 图像特征点相关参数
const bool set_max_depth = false;    //是否设置最大距离
const double max_depth = 25.0;     //默认得不到深度数据时，采用最大深度，并且在SLAM端要加上较大的不确定性，以m为单位
//匹配相关
const double match_min_dist = 50;  //小于这个距离才算匹配成功
const int max_landmark_per_loop = 100; //每回合增加到库中的路标上限，因为一帧内加入太多的路标用处也不大


//////////////////////////////////////////
// Camera matrix
const double camera_fx = 520.9, camera_fy = 521.0,
    camera_cx = 325.1, camera_cy = 249.7, camera_factor = 5000.0;

const double camera_matrix[3][3] = { { camera_fx, 0, camera_cx }, { 0, camera_fy ,camera_cy }, { 0, 0, 1 }};

const double max_pos_change = 0.2;

////////////////////////////////////////
//图优化参数
const int ROBOT_START_ID = 0;
const int LANDMARK_START_ID = 10000; //暂定，这样最多只能处理10000帧
const double landmarkNoiseX = 0.05, landmarkNoiseXL = 2, landmarkNoiseY = 0.05;   //路标点测量的噪声估计值，认为x方向超过最大距离时，误差很大，否则误差约在cm级别
const double transNoiseX = 0.005, transNoiseY = 0.005; //惯性测量设备误差
const double rotationNoise = 0.05;//角度测量设备误差
const bool add_ransac_odo = false; //是否以ransac结果作为惯性测量设备的输出
const bool robust_kernel = true; //是否使用鲁棒化的优化核
const int max_match_per_loop = 50;  //每一帧最多增长多少个边
//////////////////////////////////////////
//内联工具函数
inline int ROBOT_ID(int& id)
{
    int d =  id+ROBOT_START_ID;
    id++;
    return d;
}

inline int LANDMARK_ID(int& id){
    int d = id + LANDMARK_START_ID;
    id++;
    return d;
}
     
inline Point3f g2o2cv(Eigen::Vector3d p)
{
    return Point3f(-p[1], -p[2], p[0]);
}

inline Eigen::Vector3d cv2g2o(Point3f p)
{
    return Eigen::Vector3d(p.z, -p.x, -p.y);
}
