#pragma once

#include <iostream>
#include <string>
#include "ImageReader.h"
#include "FeatureGrabber.h"

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
};

