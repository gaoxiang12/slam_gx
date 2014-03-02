#pragma once

#include <iostream>
#include <string>
#include "ImageReader.h"
#include "FeatureGrabber.h"
#include "FeatureManager.h"
#include <g2o/types/slam2d/se2.h>

using namespace g2o;
using namespace std;

struct FEATURE
{
    
};

class GraphicEnd
{
public:
    GraphicEnd();
    ~GraphicEnd();

    int run();
    int run_once();
protected:

    ImageReaderBase* pImageReader;
    FeatureGrabberBase* pFeatureGrabber;
    FeatureManager* pFeatureManager;
    
    SE2 _robot_curr;      //当前机器人所在位置
};

