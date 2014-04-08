/*****************************************
 * 测试Graphic_end的程序
 *****************************************/

#include "GraphicEnd.h"
#include <sstream>

using namespace std;

void printhelp();

int main(int argc, char** argv)
{
    GraphicEnd graphic_end;
    int times = 1;

    if (argc == 2)
    {
        stringstream ss;
        ss<<argv[ 1 ];
        ss>>times;
    }

    for (int i=0; i<times; i++)
    {
        graphic_end.run_once();
    }

    
    return 0;
}

void printhelp()
{
    cout<<"usage: run_graphic [ times ]"<<endl;
}
