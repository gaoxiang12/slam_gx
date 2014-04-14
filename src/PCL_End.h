#pragma once
/* ****************************************
 * PCL_End: 将得到的数据进行可视化显示，并存储成点云格式
 ****************************************/
#include "const.h"
#include <string>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/io/pcd_io.h>
#include <pcl/registration/transforms.h>
#include <pcl/filters/voxel_grid.h>
using namespace std;

typedef pcl::PointXYZRGB PointT;
typedef pcl::PointCloud<PointT> PointCloud;

const float grid_size = 0.01;

class PCL_End
{
 public:
    PCL_End(const string& data_addr);
    ~PCL_End();

 public:
    void save();
    void save(string addr)
    {
        pcl::io::savePCDFileASCII(addr, *_data);
    }
    //往原有的数据中增加一个点云，需要指定机器人位置 
    void addInput(int index, const SE2& robot);
    
 protected:
    string _fileAddr;
    PointCloud::Ptr _data;  
    pcl::VoxelGrid<PointT> _grid;
};
