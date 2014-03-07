#include <iostream>
#include <cmath>

#include "simulator.h"

#include "vertex_se2.h" //机器人状态顶点
#include "vertex_point_xy.h"  //路标顶点
#include "edge_se2.h"        //惯性测量装置约束
#include "edge_se2_pointxy.h"  //外部传感器约束
#include "types_tutorial_slam2d.h" //注册自定义顶点

#include "g2o/core/sparse_optimizer.h"
#include "g2o/core/block_solver.h"
#include "g2o/core/factory.h"
#include "g2o/core/optimization_algorithm_factory.h"
#include "g2o/core/optimization_algorithm_gauss_newton.h"
#include "g2o/solvers/csparse/linear_solver_csparse.h"

using namespace std;
using namespace g2o;
using namespace g2o::tutorial;

int main()
{
  init_tutorial_slam2d_types();

  // TODO simulate different sensor offset
  // simulate a robot observing landmarks while travelling on a grid
  SE2 sensorOffsetTransf(0.2, 0.1, -0.1);
  int numNodes = 300;
  Simulator simulator;
  simulator.simulate(numNodes, sensorOffsetTransf);//300个姿态，注意是一起产生的，不是边产生边优化的

  /*********************************************************************************
   * creating the optimization problem
   ********************************************************************************/

  typedef BlockSolver< BlockSolverTraits<-1, -1> >  SlamBlockSolver;
  typedef LinearSolverCSparse<SlamBlockSolver::PoseMatrixType> SlamLinearSolver;

  // allocating the optimizer
  SparseOptimizer optimizer;
  SlamLinearSolver* linearSolver = new SlamLinearSolver();
  linearSolver->setBlockOrdering(false);
  SlamBlockSolver* blockSolver = new SlamBlockSolver(linearSolver);
  OptimizationAlgorithmGaussNewton* solver = new OptimizationAlgorithmGaussNewton(blockSolver);

  optimizer.setAlgorithm(solver);

  // add the parameter representing the sensor offset
  ParameterSE2Offset* sensorOffset = new ParameterSE2Offset;
  sensorOffset->setOffset(sensorOffsetTransf);
  sensorOffset->setId(0);
  optimizer.addParameter(sensorOffset);

  // adding the odometry to the optimizer
  // first adding all the vertices
  // 可能考虑到算的是一个full SLAM，所以把所有东西都搁进去了，但在实际上只需要关键帧即可
  cerr << "Optimization: Adding robot poses ... ";
  for (size_t i = 0; i < simulator.poses().size(); ++i) {
    const Simulator::GridPose& p = simulator.poses()[i];
    const SE2& t = p.simulatorPose; 
    VertexSE2* robot =  new VertexSE2;
    robot->setId(p.id);
    robot->setEstimate(t);
    optimizer.addVertex(robot);
  }
  cerr << "done." << endl;

  // second add the odometry constraints
  // 这一步是在加边，误差由 EdgeSE2::computeError()计算
  cerr << "Optimization: Adding odometry measurements ... ";
  for (size_t i = 0; i < simulator.odometry().size(); ++i) {
    const Simulator::GridEdge& simEdge = simulator.odometry()[i];

    EdgeSE2* odometry = new EdgeSE2;
    odometry->vertices()[0] = optimizer.vertex(simEdge.from);//这里的from 和to 能指代顶点的id
    odometry->vertices()[1] = optimizer.vertex(simEdge.to);
    odometry->setMeasurement(simEdge.simulatorTransf);
    odometry->setInformation(simEdge.information);
    optimizer.addEdge(odometry);
  }
  cerr << "done." << endl;

  // add the landmark observations
  // 这一步是在加顶点哦，就是把观察到的landmark都加到图中去
  cerr << "Optimization: add landmark vertices ... ";
  for (size_t i = 0; i < simulator.landmarks().size(); ++i) {
    const Simulator::Landmark& l = simulator.landmarks()[i];
    VertexPointXY* landmark = new VertexPointXY;
    landmark->setId(l.id);
    landmark->setEstimate(l.simulatedPose);
    optimizer.addVertex(landmark);
  }
  cerr << "done." << endl;

  //增加边，一个帧中观察到了n个路标，就要加n个边
  cerr << "Optimization: add landmark observations ... ";
  for (size_t i = 0; i < simulator.landmarkObservations().size(); ++i) {
    const Simulator::LandmarkEdge& simEdge = simulator.landmarkObservations()[i];
    EdgeSE2PointXY* landmarkObservation =  new EdgeSE2PointXY;
    landmarkObservation->vertices()[0] = optimizer.vertex(simEdge.from);
    landmarkObservation->vertices()[1] = optimizer.vertex(simEdge.to);
    
    landmarkObservation->setMeasurement(simEdge.simulatorMeas); //添加一个观察
    landmarkObservation->setInformation(simEdge.information);   //添加了一个信息矩阵，个人觉得关系不大
    landmarkObservation->setParameterId(0, sensorOffset->id());
    optimizer.addEdge(landmarkObservation);
  }
  cerr << "done." << endl;


  /*********************************************************************************
   * optimization
   ********************************************************************************/

  // dump initial state to the disk
  optimizer.save("tutorial_before.g2o");

  // prepare and run the optimization
  // fix the first robot pose to account for gauge freedom
  VertexSE2* firstRobotPose = dynamic_cast<VertexSE2*>(optimizer.vertex(0));
  firstRobotPose->setFixed(true);
  optimizer.setVerbose(true);

  cerr << "Optimizing" << endl;
  optimizer.initializeOptimization();
  optimizer.optimize(10);
  cerr << "done." << endl;

  //因为毕竟有300个结点同时在优化，所以比较影响速度
  
  optimizer.save("tutorial_after.g2o");

  // freeing the graph memory
  optimizer.clear();

  // destroy all the singletons
  Factory::destroy();
  OptimizationAlgorithmFactory::destroy();
  HyperGraphActionLibrary::destroy();

  return 0;
}
