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
}
