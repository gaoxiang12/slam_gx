#include "ImageReader.h"
#include "yaml-cpp/yaml.h"
#include "const.h"
#include "ParameterReader.h"
#include <iostream>
#include <fstream>
#include <sstream>
#include <opencv/cv.h>
#include <opencv/highgui.h>

using namespace cv;

ImageReader::ImageReader(string parameter_file_addr)
{
    _start_index = atoi(g_pParaReader->GetPara("start_index").c_str());
    _end_index = atoi( g_pParaReader->GetPara("end_index").c_str());
    _data_source = g_pParaReader->GetPara("data_source");
    _grayscale = g_pParaReader->GetPara("grayscale") == string("yes");
    _index_curr = _start_index;
}

int ImageReader::Next()
{
    if (_index_curr > _end_index)
        return 0;
    stringstream ss;
    ss <<_data_source << "/rgb_index/" << _index_curr << ".png";
    string cmd;
    ss >> cmd;
    if (_grayscale)
    {
        _rgb = imread( cmd, CV_LOAD_IMAGE_GRAYSCALE);
    }
    else
    {
        _rgb = imread( cmd );
    }
    ss.clear();
    ss <<_data_source << "/dep_index/" << _index_curr << ".png";
    ss >> cmd;
    //注意读深度图的时候不能用CV_LOAD_IMAGE_GRAYSCALE,不然会把深度值归一化到（0，255）
    if (_grayscale)
    {
        _depth = imread( cmd, IMREAD_ANYDEPTH );
    }
    else
    {
        _depth = imread( cmd );
    }
    
    if ( !_rgb.data || !_depth.data )
    {
        cerr<< _index_curr << " image not found."<<endl;
        return 0;
    }

    _index_curr++;
    return 1;
}

