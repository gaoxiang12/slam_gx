#pragma once
#include "FABMAP.h"
#include <string>
#include <map>

using namespace std;

struct PLACE_RECOG
{
    PLACE_RECOG() {
        id = 0; similar = 0; prob = 0.0; isNewPlace=true;
    }
    int id;
    int similar;
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
    map<int, int> _frame_place; //二元组，第一个元素为帧id，第二个为place的id
};

extern FABMAP_End* g_pFAB;
