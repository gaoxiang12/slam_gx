
#include "GraphicEnd.h"
#include "Slam_end.h"
#include <sstream>

using namespace std;

void usage()
{
    cout<<"usage: run_graphic_and_slam [times=1]"<<endl;
}

int main(int argc, char** argv)
{
    int times = 1;

    if (argc == 2)
    {
        stringstream ss;
        ss<<argv[1];
        ss>>times;
    }
    else if (argc > 2)
    {
        usage();
        return -1;
    }

    GraphicEnd graphic_end;
    SLAMEnd slam_end(&graphic_end);
    
    for (int i=0; i<times; i++)
    {
        graphic_end.run_once();
        slam_end.optimize_once();
    }
}
