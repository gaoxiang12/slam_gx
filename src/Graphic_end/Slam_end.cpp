/* ****************************************
 * SLAM 端的实现
 ****************************************/
#include "Slam_end.h"


#include "Slam_end.h"
#include <string>

#include <g2o/types/slam2d/vertex_se2.h>
#include <g2o/types/slam2d/vertex_point_xy.h>
#include <g2o/types/slam2d/edge_se2.h>
#include <g2o/types/slam2d/edge_se2_pointxy.h>
#include <g2o/types/slam2d/types_slam2d.h>

#include <sstream>

using namespace std;
using namespace g2o;

SLAMEnd::SLAMEnd(GraphicEnd* pGraphicEnd)
{
    SlamLinearSolver* linearSolver = new SlamLinearSolver();
    linearSolver->setBlockOrdering(false);
    SlamBlockSolver* blockSolver = new SlamBlockSolver(linearSolver);
    OptimizationAlgorithmGaussNewton* solver = new OptimizationAlgorithmGaussNewton(blockSolver);

    optimizer.setAlgorithm(solver);

    _pGraphicEnd = pGraphicEnd;
    _pFeatureManager = pGraphicEnd->pFeatureManager;
    _pImageReader = pGraphicEnd->pImageReader;
    _pFeatureGrabber = pGraphicEnd->pFeatureGrabber;
    
    _optimize_step = atoi(_pImageReader->GetParameters("optimize_step").c_str());

    _robot_id = 0;
    _landmark_id = 0;

    _robust_kernel_ptr = RobustKernelFactory::instance()->construct(_pImageReader->GetParameters("robust_kernel"));
    if (debug_info)
    {
        cout<<"Slam end init over."<<endl;
    }
}

SLAMEnd::~SLAMEnd()
{
    optimizer.clear();
    RobustKernelFactory::destroy();
    //    Factory::destory();
    //    OptimizationAlgorithmFactory::destory();
    //    HyperGraphActionLibrary::destroy();
}

int SLAMEnd::optimize()
{
    return 1;
}

//重要！
//这一步会向图中增加新的结点与边
int SLAMEnd::optimize_once()
{
    if (_pFeatureManager->_success == false)
    {
        cerr<<"SLAMEnd: RANSAC failed. Skip optimization."<<endl;
        return -1;
    }
    //Step 1. 把当前机器人位置增加到pose图中
    AddRobotPose();
    //Step 2. 把与当前位置相关的路标，以及相连的边也加至图中
    AddLandmark();
    //Step 3. 优化
    solve();
    //Step 4. 反馈
    feedback();

    //保存
    save();
    return 1;
}

void SLAMEnd::feedback()
{
    //将优化的结果返回至Graphic end的特征仓库中
    //Step 1. 更新机器人位置
    
    VertexSE2* curr = dynamic_cast<VertexSE2*> (optimizer.vertex(_robot_id - 1));
    Eigen::Vector3d* esti = new Eigen::Vector3d();
    curr->getEstimateData((double*)esti);
    if (debug_info)
    {
        cout<<"robot pos changed to:"<<(*esti)[0]<<", "<<(*esti)[1]<<", "<<(*esti)[2]<<endl;
    }
    _pGraphicEnd->_robot_curr.fromVector(*esti);
    delete esti;

    //Step 2. 更新路标位置
    list<LANDMARK>& landmarks = _pFeatureManager->_landmark_library;
    int i=0; 
    for (list<LANDMARK>::iterator iter = landmarks.begin();
         iter != landmarks.end(); iter++)
    {
        VertexPointXY* t = dynamic_cast<VertexPointXY*>(optimizer.vertex(i + LANDMARK_START_ID));
        if (t)
        {
            //在图中存在该路标，则更新它的位置信息
            Eigen::Vector2d *d = new Eigen::Vector2d();
            t->getEstimateData((double*)d);
            Eigen::Vector3d new_pos((*d)[0], (*d)[1], iter->_pos_g2o[2]);
            iter->_pos_g2o = new_pos;
            iter->_pos_cv = g2o2cv(new_pos);
            delete d;
        }
        i++;
        //不存在就不用鸟这破玩意了
    }
}

void SLAMEnd::testOptimization(string fileAddr)
{
    ifstream fin(fileAddr.c_str());
    optimizer.load(fin);
    optimizer.initializeOptimization();
    optimizer.optimize(_optimize_step);

    optimizer.save("log/debug_after.g2o");
    return;
}

void SLAMEnd::AddRobotPose()
{
    if (debug_info)
    {
        cout<<"Opt: add robot pose."<<endl;
    }

    //将当前的机器人位置加入图中
    SE2 pr = _pGraphicEnd->_robot_curr;
    VertexSE2* robot = new VertexSE2;
    robot->setId( ROBOT_ID(_robot_id) );
    robot->setEstimate(pr);
    optimizer.addVertex(robot);

    
    if (add_ransac_odo && _robot_id > 1)
    {
        //非初始化，加入一条边，连接新顶点与前顶点
        VertexSE2* prev = dynamic_cast<VertexSE2*> (optimizer.vertex(_robot_id-2));
        if (prev)
        {
            EdgeSE2* odo = new EdgeSE2;
            odo->vertices()[0] = optimizer.vertex( _robot_id - 2 );
            odo->vertices()[1] = optimizer.vertex( _robot_id - 1 );
            odo->setMeasurement( SE2(0,0,0) ); //因为没有惯性测量设备，我们默认机器人没有移动
            //odo->setMeasurement( _prev.inverse()*pr );  //或者，使用了RANSAC得到的结果，作为惯性测量
            Matrix3d information, cov;
            cov.fill(0.);
            cov(0,0) = transNoiseX * transNoiseX;
            cov(1,1) = transNoiseY * transNoiseY;
            cov(2,2) = rotationNoise * rotationNoise;
            information = cov.inverse();
            odo->setInformation( information );
            optimizer.addEdge(odo);
        }
    }

    _prev = pr;
}

void SLAMEnd::AddLandmark()
{
    //将与当前机器人位置有关的路标加到图中
    //首先加顶点，即在此帧中观察到的新路标
    vector<int>& match = _pFeatureManager->_match_idx;
    cout<<"g2o: match size = "<<match.size()<<endl;
    
    //由于边集可能增长的太快，考虑限制一下它的增长速度
    int add_edge = 0, add_vertex = 0;
    for (size_t i=0; i<match.size(); i++)
    {
        if (add_edge > max_match_per_loop)
        {
            break;
        }
            
        //if (_pFeatureManager->_inlier[i] == false)
        {
            //            continue;
        }
        int id = match[i] + LANDMARK_START_ID;
        VertexPointXY* landmark = dynamic_cast<VertexPointXY*>(optimizer.vertex(id));
        LANDMARK l = _pFeatureManager->GetLandmark(match[i]);

        add_edge++;
        if (landmark == NULL)
        {
            add_vertex++;
            //该路标未在图中出现，新增一个节点
            landmark = new VertexPointXY;
            landmark->setId(match[i] + LANDMARK_START_ID);
            landmark->setEstimate( l.Pose2d() );
            optimizer.addVertex( landmark );
        }
        else
        {
        }

        //增加current_state与landmark相连的那条边

        EdgeSE2PointXY* landmarkObservation = new EdgeSE2PointXY;

        //机器人位置所在的顶点，因为前面加过1了，现在就要减1
        landmarkObservation->vertices()[0] = optimizer.vertex( _robot_id - 1 + ROBOT_START_ID );
        landmarkObservation->vertices()[1] = optimizer.vertex( id );
        landmarkObservation->setMeasurement( _pFeatureGrabber->GetObservation2d(_pFeatureManager->_match_keypoints[i]) );
        Matrix2d information, cov;
        cov.fill(0.);
        if ( l._pos_g2o[0] > 15 )
        {
            //x 方向很远，说明这个值超过了深度图的最大测量距离，它的值是不确定的
            cov(0,0) = landmarkNoiseXL * landmarkNoiseXL;
        }
        else
        {
            cov(0,0) = landmarkNoiseX * landmarkNoiseX;
        }
        cov(1,1) = landmarkNoiseY * landmarkNoiseY;
        information = cov.inverse();
        landmarkObservation->setInformation(information);
        if (robust_kernel)
        {
            landmarkObservation->setRobustKernel( _robust_kernel_ptr );
        }
        optimizer.addEdge(landmarkObservation);
    }

    if (debug_info)
    {
        cout<<"g2o: add "<<add_vertex<<" new landmark, "<<add_edge<<" observations."<<endl;
    }
}
