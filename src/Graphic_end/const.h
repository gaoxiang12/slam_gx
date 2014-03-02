/**************************************
 * const.h 定义常量的地方
 *************************************/
#pragma once
#include <string>
#include <opencv2/core/core.hpp>

using namespace cv;
using namespace std;

const bool debug_info = true; //是否输出调试信息
const string parameter_file_addr = "/home/y/code/slam_gx/parameters.yaml";
const bool vision = true;
//////////////////////////////////////////
// 图像特征点相关参数
const int surf_minHessian = 400;

//匹配相关
const double match_min_dist = 0.02;  //小于这个距离才算匹配成功

//////////////////////////////////////////
// Camera matrix
const double camera_fx = 520.9, camera_fy = 521.0,
    camera_cx = 325.1, camera_cy = 249.7, camera_factor = 5000.0;

const double camera_matrix[3][3] = { { camera_fx, 0, camera_cx }, { 0, camera_fy ,camera_cy }, { 0, 0, 1 }};
