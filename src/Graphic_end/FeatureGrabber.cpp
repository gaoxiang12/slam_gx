#include "FeatureGrabber.h"
#include "const.h"

#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <iostream>

using namespace std;
using namespace cv;

vector<KeyPoint> FeatureGrabber::GetKeyPoints()
{
    if (debug_info)
    {
        cout<<"grabbing features, feature name is "<<_featureName<<endl;
    }
    initModule_nonfree();
    Ptr<FeatureDetector> detector = FeatureDetector::create("SIFT");

    if (detector.empty())
    {
        cerr<<"fail to create detector!"<<endl;
        return _keypoints;
    }
    _keypoints.clear();

    detector->detect(_rgb, _keypoints);

    if (debug_info)
    {
        cout<<"detect "<<_keypoints.size()<<" keypoints."<<endl;
    }
    return _keypoints;
}

Mat FeatureGrabber::GetDescriptors()
{
    if (debug_info)
    {
        cout<<"computing descriptors ... "<<endl;
    }

    Ptr<DescriptorExtractor> descriptor_extractor = DescriptorExtractor::create( _featureName );
    Mat descriptors;
    descriptor_extractor->compute(_rgb, _keypoints, descriptors);

    return descriptors;
}
