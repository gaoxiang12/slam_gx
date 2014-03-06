/* ****************************************
 * SLAM 端的实现
 ****************************************/
#include "Slam_end.h"


SLAMEnd::SLAMEnd()
{
    SlamLinearSolver* linearSolver = new SlamLinearSolver();
    linearSolver->setBlockOrdering(false);
    SlamBlockSolver* blockSolver = new SlamBlockSolver(linearSolver);
    OptimizationAlgorithmGaussNewton* solver = new OptimizationAlgorithmGaussNewton(blockSolver);

    optimizer.setAlgorithm(solver);
    
}
