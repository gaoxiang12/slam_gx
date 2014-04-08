#pragma once
#include "FABMAP.h"
#include <string>
#include <map>

using namespace std;

struct PLACE_RECOG
{
    int id;
    double prob;
    bool isNewPlace;
};

class FABMAP_End
{
 public:
    FABMAP_End();
    //input image and its frame id, return the recognition result
    PLACE_RECOG process(string filename, int frame); 
    
 protected:
    CFABMAPClient _engine;
    map<int, int> _frame_place; //each frame must have a place id
};
