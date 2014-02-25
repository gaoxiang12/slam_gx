
#ifndef G2O_TUTORIAL_VERTEX_SE2_H
#define G2O_TUTORIAL_VERTEX_SE2_H

#include "g2o/core/base_vertex.h"
#include "g2o/core/hyper_graph_action.h"
#include "se2.h"
#include "g2o_tutorial_slam2d_api.h"

namespace g2o {
  namespace tutorial {

    /**
     * \brief 2D pose Vertex, (x,y,theta)
     */
      class G2O_TUTORIAL_SLAM2D_API VertexSE2 : public BaseVertex<3, SE2> //中间那个G2O的宏没有什么实际意义，只是区别一下编译器
          //SE2的定义见se2.h，它包含了机器人的位置与旋转角
    {
      public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
        VertexSE2();

        virtual void setToOriginImpl() {
            //_estimate是第二个类型的一个实例，所以可以这样写
          _estimate=SE2();
        }

        virtual void oplusImpl(const double* update)
        {
          SE2 up(update[0], update[1], update[2]);
          _estimate *= up;
        }

        virtual bool read(std::istream& is);
        virtual bool write(std::ostream& os) const;

    };

  } // end namespace
} // end namespace

#endif
