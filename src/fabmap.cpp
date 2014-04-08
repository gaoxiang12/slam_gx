#include "FABMAP.h"
#include "FABMAP_End.h"

FABMAP_End::FABMAP_End()
{
    //Note: before run fabmap client, you should run MOOSDB, wordmaker and Fabmap first.
    
    _engine.Open("localhost", 9000);
    
}

PLACE_RECOG FABMAP_End::process( string filename, int frame )
{
    PLACE_RECOG p;
    p.id = _engine.ProcessImage( filename );
    //insert it to map
    _frame_place.insert( pair<int,int>(frame, p.id) );
    double prob;
    unsigned int nPlace;
    string sFileName;
    if (_engine.GetMostProbablePlace(nPlace, sFileName, prob))
    {
        cout<<"image "<<frame<<" is most similar to "<<nPlace<<endl;
        p.similar = nPlace;
        p.isNewPlace = (p.id==nPlace);
        p.prob = prob;
    }

    return p;
}

FABMAP_End* g_pFAB;
