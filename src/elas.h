/*
版权所有 2011，保留所有权利。
Institute of Measurement and Control Systems
Karlsruhe Institute of Technology, Germany

本文件是 libelas 的一部分。
Authors: Andreas Geiger

libelas 是自由软件；你可以根据自由软件基金会发布的 GNU 通用公共许可证
（GNU General Public License）第 3 版，或（由你选择的）任何更高版本的条款
对其进行再发布和/或修改。

发布 libelas 的目的是希望它能发挥作用，但**不提供任何担保**；甚至不包含
对适销性或特定用途适用性的默示担保。更多细节请参阅 GNU 通用公共许可证。

你应该已经随同 libelas 一起收到了 GNU 通用公共许可证的副本；
如果没有，请写信至 Free Software Foundation, Inc.,
51 Franklin Street, Fifth Floor, Boston, MA 02110-1301, USA。
*/

// 主头文件：在你的代码中包含本文件即可使用 libelas。

#ifndef __ELAS_H__
#define __ELAS_H__

#include <iostream>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <vector>
#include <emmintrin.h>
 
// 为兼容新版本 Visual Studio：统一使用标准头 <stdint.h> 中定义的定长整数类型
// （int8_t、uint8_t、int32_t 等），避免旧代码中自行 typedef 与 VS 自带定义重复，
// 导致 "int8_t: 重定义; 不同的基类型" 之类的编译错误
#include <stdint.h>

#ifdef PROFILE
#include "timer.h"
#endif

class Elas {
  
public:
  
  enum setting {ROBOTICS,MIDDLEBURY};
  
  // 参数设置
  struct parameters {
    int32_t disp_min;               // 最小视差
    int32_t disp_max;               // 最大视差
    float   support_threshold;      // 支持点唯一性阈值（最佳匹配与次优匹配的能量比）
    int32_t support_texture;        // 生成支持点所需的最小纹理
    int32_t candidate_stepsize;     // 支持点候选的规则网格步长
    int32_t incon_window_size;      // 用于检测不一致支持点的窗口大小
    int32_t incon_threshold;        // 判断支持点一致性的视差相似度阈值
    int32_t incon_min_support;      // 认为有效所需的最小一致支持点个数
    bool    add_corners;            // 是否在图像四角添加附加支持点（使用最近邻视差）
    int32_t grid_size;              // 额外支持点外推所使用的网格邻域尺寸
    float   beta;                   // 图像似然参数
    float   gamma;                  // 先验常数
    float   sigma;                  // 先验高斯的 sigma
    float   sradius;                // 先验高斯的空间半径
    int32_t match_texture;          // 稠密匹配所需的最小纹理
    int32_t lr_threshold;           // 左右一致性检查的视差阈值
    float   speckle_sim_threshold;  // 斑点分割的相似度阈值
    int32_t speckle_size;           // 斑点的最大尺寸（过小斑点会被移除）
    int32_t ipol_gap_width;         // 插值时允许的最大空洞宽度（左右、上下方向）
    bool    filter_median;          // 是否使用近似中值滤波
    bool    filter_adaptive_mean;   // 是否使用近似自适应均值滤波
    bool    postprocess_only_left;  // 是否仅对左视图进行后处理以节省时间
    bool    subsampling;            // 是否只在每隔一个像素上计算视差以加快速度
                                    // 注意：启用该选项时，D1 和 D2 的尺寸应为
                                    //       width/2 x height/2（向零取整）
    
    // 构造函数：根据不同场景预设参数
    parameters (setting s=ROBOTICS) {
      
      // 机器人环境下的默认参数设置
      // （在半遮挡区域不生成视差，对光照变化等情况更鲁棒）
      if (s==ROBOTICS) {
        disp_min              = 0;
        disp_max              = 255;
        support_threshold     = 0.85;
        support_texture       = 10;
        candidate_stepsize    = 5;
        incon_window_size     = 5;
        incon_threshold       = 5;
        incon_min_support     = 5;
        add_corners           = 0;
        grid_size             = 20;
        beta                  = 0.02;
        gamma                 = 3;
        sigma                 = 1;
        sradius               = 2;
        match_texture         = 1;
        lr_threshold          = 2;
        speckle_sim_threshold = 1;
        speckle_size          = 200;
        ipol_gap_width        = 3;
        filter_median         = 0;
        filter_adaptive_mean  = 1;
        postprocess_only_left = 1;
        subsampling           = 0;
        
      // Middlebury 基准测试的默认参数设置
      // （对所有缺失视差进行插值）
      } else {
        disp_min              = 0;
        disp_max              = 255;
        support_threshold     = 0.95;
        support_texture       = 10;
        candidate_stepsize    = 5;
        incon_window_size     = 5;
        incon_threshold       = 5;
        incon_min_support     = 5;
        add_corners           = 1;
        grid_size             = 20;
        beta                  = 0.02;
        gamma                 = 5;
        sigma                 = 1;
        sradius               = 3;
        match_texture         = 0;
        lr_threshold          = 2;
        speckle_sim_threshold = 1;
        speckle_size          = 200;
        ipol_gap_width        = 5000;
        filter_median         = 1;
        filter_adaptive_mean  = 0;
        postprocess_only_left = 0;
        subsampling           = 0;
      }
    }
  };

  // 构造函数，输入：参数集合
  Elas (parameters param) : param(param) {}

  // 析构函数
  ~Elas () {}
  
  // 主匹配函数
  // 输入：左图（I1）和右图（I2）的灰度图指针（uint8，输入）
  //       左视差图（D1）和右视差图（D2）的指针（float，输出）
  //       dims[0] = I1 与 I2 的宽度
  //       dims[1] = I1 与 I2 的高度
  //       dims[2] = 每行字节数（通常等于宽度，但也可以不同）
  // 说明：调用前必须为 D1 与 D2 分配好内存（每行字节数 = 宽度）；
  //       若未启用 subsampling，则尺寸为 width x height；
  //       若启用了 subsampling，则为 width/2 x height/2（向零取整）。
  void process (uint8_t* I1,uint8_t* I2,float* D1,float* D2,const int32_t* dims);
  
private:
  
  struct support_pt {
    int32_t u;
    int32_t v;
    int32_t d;
    support_pt(int32_t u,int32_t v,int32_t d):u(u),v(v),d(d){}
  };

  struct triangle {
    int32_t c1,c2,c3;
    float   t1a,t1b,t1c;
    float   t2a,t2b,t2c;
    triangle(int32_t c1,int32_t c2,int32_t c3):c1(c1),c2(c2),c3(c3){}
  };

  inline uint32_t getAddressOffsetImage (const int32_t& u,const int32_t& v,const int32_t& width) {
    return v*width+u;
  }

  inline uint32_t getAddressOffsetGrid (const int32_t& x,const int32_t& y,const int32_t& d,const int32_t& width,const int32_t& disp_num) {
    return (y*width+x)*disp_num+d;
  }

  // 支持点相关函数
  void removeInconsistentSupportPoints (int16_t* D_can,int32_t D_can_width,int32_t D_can_height);
  void removeRedundantSupportPoints (int16_t* D_can,int32_t D_can_width,int32_t D_can_height,
                                     int32_t redun_max_dist, int32_t redun_threshold, bool vertical);
  void addCornerSupportPoints (std::vector<support_pt> &p_support);
  inline int16_t computeMatchingDisparity (const int32_t &u,const int32_t &v,uint8_t* I1_desc,uint8_t* I2_desc,const bool &right_image);
  std::vector<support_pt> computeSupportMatches (uint8_t* I1_desc,uint8_t* I2_desc);

  // 三角剖分与离散视差网格
  std::vector<triangle> computeDelaunayTriangulation (std::vector<support_pt> p_support,int32_t right_image);
  void computeDisparityPlanes (std::vector<support_pt> p_support,std::vector<triangle> &tri,int32_t right_image);
  void createGrid (std::vector<support_pt> p_support,int32_t* disparity_grid,int32_t* grid_dims,bool right_image);

  // 视差匹配
  inline void updatePosteriorMinimum (__m128i* I2_block_addr,const int32_t &d,const int32_t &w,
                                      const __m128i &xmm1,__m128i &xmm2,int32_t &val,int32_t &min_val,int32_t &min_d);
  inline void updatePosteriorMinimum (__m128i* I2_block_addr,const int32_t &d,
                                      const __m128i &xmm1,__m128i &xmm2,int32_t &val,int32_t &min_val,int32_t &min_d);
  inline void findMatch (int32_t &u,int32_t &v,float &plane_a,float &plane_b,float &plane_c,
                         int32_t* disparity_grid,int32_t *grid_dims,uint8_t* I1_desc,uint8_t* I2_desc,
                         int32_t *P,int32_t &plane_radius,bool &valid,bool &right_image,float* D);
  void computeDisparity (std::vector<support_pt> p_support,std::vector<triangle> tri,int32_t* disparity_grid,int32_t* grid_dims,
                         uint8_t* I1_desc,uint8_t* I2_desc,bool right_image,float* D);

  // 左右视差一致性检查
  void leftRightConsistencyCheck (float* D1,float* D2);
  
  // 后处理
  void removeSmallSegments (float* D);
  void gapInterpolation (float* D);

  // 可选后处理
  void adaptiveMean (float* D);
  void median (float* D);
  
  // 参数集合
  parameters param;
  
  // 内存按对齐方式存放的输入图像及其尺寸
  uint8_t *I1,*I2;
  int32_t width,height,bpl;
  
  // 性能分析计时器
#ifdef PROFILE
  Timer timer;
#endif
};

#endif
