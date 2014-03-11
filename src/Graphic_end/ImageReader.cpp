#include "ImageReader.h"
#include "yaml-cpp/yaml.h"
#include "const.h"
#include <iostream>
#include <fstream>
#include <sstream>
#include <opencv/cv.h>
#include <opencv/highgui.h>

using namespace cv;

ImageReader::ImageReader(string parameter_file_addr)
{
    if (debug_info)
    {
        cout<<"init image reader, parameter file is in "
            <<parameter_file_addr<<endl;
    }

    ifstream fin(parameter_file_addr.c_str());
    YAML::Parser parser(fin);
    YAML::Node config;
    try {
        parser.GetNextDocument(config);
    } catch (YAML::ParserException& e)
    {
        cerr<<e.what()<<"\n";
        return ;
    }

    //直接从config中读信息即可
    config[ "data_source" ] >> _data_source;
    config[ "detector_name" ] >> _detector_name;
    config[ "descriptor_name" ] >> _descriptor_name;
    config[ "start_index" ] >> _start_index;
    config[ "end_index" ] >> _end_index;
    config[ "grayscale" ] >> _grayscale;
    config[ "step_time" ] >> _step_time;
    config[ "save_if_seen" ] >> _save_if_seen;
    config[ "del_not_seen" ] >> _del_not_seen;
    config[ "optimize_step" ] >> _optimize_step;
    config[ "robust_kernel" ] >> _robust_kernel;
    if (_end_index < _start_index)
    {
        cerr<<"end index should be larger than start index."<<endl;
        return;
    }

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

string ImageReader::GetParameters(string para_name) const
{
    stringstream ss;
    string s;
    if (para_name == string("detector_name"))
        return _detector_name;
    if (para_name == string("descriptor_name"))
        return _descriptor_name;
    if (para_name == string("data_source"))
        return _data_source;
    if (para_name == string("step_time"))
    {
        ss<<_step_time;
        ss>>s;
        return s;
    }
    if (para_name == string("save_if_seen"))
    {
        ss<<_save_if_seen;
        ss>>s;
        return s;
    }
    if (para_name == string("del_not_seen"))
    {
        ss<<_del_not_seen;
        ss>>s;
        return s;
    }
    if (para_name == string("optimize_step"))
    {
        ss<<_optimize_step;
        ss>>s;
        return s;
    }
    if (para_name == string("robust_kernel"))
    {
        return _robust_kernel;
    }
    
    return string("unknown_para_name");
}
