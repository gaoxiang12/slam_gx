/* ****************************************
 * SLAM end: 基于图优化的SLAM端，
 * 从Graphic end中读取数据并估计机器人位置与路标
 ****************************************/
#pragma once

#include "GraphicEnd.h"
#include <g2o/types/slam2d/vertex_se2.h>
#include <g2o/types/slam2d/vertex_point_xy.h>
#include <g2o/types/slam2d/edge_se2.h>
#include <g2o/types/slam2d/edge_se2_pointxy.h>
#include <g2o/types/slam2d/types_slam2d.h>

#include <g2o/core/sparse_optimizer.h>
#include <g2o/core/block_solver.h>
#include <g2o/core/factory.h>
#include <g2o/core/optimization_algorithm_factory.h>
#include <g2o/core/optimization_algorithm_gauss_newton.h>
#include <g2o/solvers/csparse/linear_solver_csparse.h>

using namespace std;
using namespace g2o;

typedef BlockSolver< BlockSolverTraits<-1, -1>> SlamBlockSolver;
typedef LinearSolverCSparse<SlamBlockSolver::PoseMatrixType> SlamLinearSolver;



class SLAMEnd
{
 public:
    SLAMEnd();
    ~SLAMEnd();

 public:
    int optimize();
    int optimize_once();

 protected:
    SparseOptimizer optimizer;
    
};
