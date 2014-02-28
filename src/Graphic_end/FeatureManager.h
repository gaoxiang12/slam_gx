/*****************************************
 * Feature Manager 管理得到的特征并进行帧间配准
 ****************************************/
#include <opencv2/core/core.hpp>
#include <Eigen/Core>
#include <g2o/types/slam2d/se2.h>
#include <opencv2/features2d/features2d.hpp>


using namespace std;
using namespace cv;
using namespace g2o;

class FeatureManager
{
 public:
    FeatureManager(int save_if_seen = 10 );
    Mat GetLandmarks() const {
        return _landmark_desp;
    }

    void Input(vector<KeyPoint>& keypoints, Mat feature_descriptor, SE2 robot_curr);  //输入当前帧的特征点描述子与机器人位置

 protected:
    Mat _landmark_desp;  //存储路标的描述子,每一行代表一个特征
    Mat _landmark_pos;   //路标的位置
    Mat _temp;           //缓存

    // 参数定义
    int _save_if_seen;  //多少帧连续看见该特征，则存储之
    
};
