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

    virtual int GetCurrentIndex() const =0;
    virtual string GetCurrentFileName() const =0;
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
    virtual int GetCurrentIndex() const {
        return _index_curr-1; //因为每次都是先调Next，然后再调这个函数，所以要减1
    }
    virtual string GetCurrentFileName() const {
        return _filename;
    }
    virtual Mat GetRGB() {
        return _rgb;
    }
    virtual Mat GetDep() {
        return _depth;
    }
 protected:
    // image
    Mat _rgb;
    Mat _depth;

    int _index_curr;  //当前所在的索引位置
    int _start_index, _end_index;

    string _data_source;
    bool _grayscale;

    string _filename;
};
