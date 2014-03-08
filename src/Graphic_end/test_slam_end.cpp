// 测试slam端的程序

#include "Slam_end.h"
#include <string>
#include <iostream>
using namespace std;
using namespace g2o;


void Usage()
{
    cout<<"test_slam [g2o file]"<<endl;
}

int main( int argc, char** argv )
{
    if (argc != 2)
    {
        Usage();
        return -1;
    }

    SLAMEnd slam_end;

    slam_end.test_slam(string(argv[1]));
    return 0;
}
