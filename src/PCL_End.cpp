#include "PCL_End.h"

#include <sstream>
#include <Eigen/Geometry>
using namespace std;

PCL_End::PCL_End(const string& data_addr)
    : _data(new PointCloud)
{
    _fileAddr = data_addr;
    string loadfile = data_addr + string("/pcd_new/1.pcd");
    _grid.setLeafSize(grid_size, grid_size, grid_size);

    cout<<"PCL End init ok"<<endl;
}

PCL_End::~PCL_End()
{
    
}

void PCL_End::save()
{
    string str = "./final.pcd";
    pcl::io::savePCDFileASCII(str, *_data);

    cout<<"PCD file saved to "<<str<<endl;
    
}

void PCL_End::addInput(int index, const SE2& robot)
{
    cout<<"calling PCL End"<<endl;
    PointCloud::Ptr p( new PointCloud() );
    //读取点云数据
    string str;
    stringstream ss;
    ss<<_fileAddr<<"/pcd_new/"<<index<<".pcd";
    ss>>str;
    cout<<"PCL loading "<<str<<endl;
    
    pcl::io::loadPCDFile(str, *p);

    _grid.setInputCloud(p);
    PointCloud::Ptr pf( new PointCloud() );
    
    _grid.filter(*pf);
    
    //将读取的点云加到原有的点云上，偏转量为机器人的位置信息
    Eigen::Transform<float, 3, Eigen::Affine> t = Eigen::AngleAxis<float>(robot[2], Eigen::Vector3f(0,0,1)) * Eigen::Translation<float,3>(robot[0],robot[1], 0);

    
    PointCloud::Ptr temp(new PointCloud);
    pcl::transformPointCloud(*pf, *temp, t);

    //将旋转后的点云加至全局数据并过滤
    _grid.setInputCloud(temp);
    _grid.filter(*temp);

    *_data += *temp;

}
