#pragma once
#include <string>
#include <opencv/cv.h>

using namespace cv;
using namespace std;

/*************************************************************
 * Image Reader: 读取参数文件与图片的类，从属于GraphicEnd
 * 因为将来可能写其他的数据库读取方式，所以定义一个基类
 *************************************************************/
class ImageReaderBase
{
 public:
    virtual int Next( )=0;
    virtual bool IsEnd() const =0;
    virtual Mat GetRGB() {
        return Mat();
    }
    virtual Mat GetDep() {
        return Mat();
    }

    virtual string GetParameters(string para_name) const {
        return string("");
    }
};


class ImageReader: public ImageReaderBase
{
 public:
    ImageReader(string parameter_file_addr);

    ~ImageReader();

    virtual int Next();  //Read _index_curr Image, then _index_curr++, return no zero if exist
    virtual bool IsEnd() const {
        return _index_curr > _end_index;
    }

    virtual Mat GetRGB() {
        return _rgb;
    }
    virtual Mat GetDep() {
        return _depth;
    }
    virtual string GetParameters(string para_name) const;
 protected:

    // image
    Mat _rgb;
    Mat _depth;

    int _index_curr;  //当前所在的索引位置
    
    // configure parameters
    string _data_source; //数据来源
    string _feature_name;//特征名
    int _start_index;    //起始索引
    int _end_index;      //终止索引
    int _grayscale;      //是否以灰度图读取
    
};
