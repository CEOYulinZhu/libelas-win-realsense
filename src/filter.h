/*
版权所有 2011，保留所有权利。
Institute of Measurement and Control Systems
Karlsruhe Institute of Technology, Germany

本文件是 libelas 的一部分。
Authors: Julius Ziegler, Andreas Geiger

libelas 是自由软件；你可以根据自由软件基金会发布的 GNU 通用公共许可证
（GNU General Public License）第 3 版，或（由你选择的）任何更高版本的条款
对其进行再发布和/或修改。

发布 libelas 的目的是希望它能发挥作用，但**不提供任何担保**；甚至不包含
对适销性或特定用途适用性的默示担保。更多细节请参阅 GNU 通用公共许可证。

你应该已经随同 libelas 一起收到了 GNU 通用公共许可证的副本；
如果没有，请写信至 Free Software Foundation, Inc.,
51 Franklin Street, Fifth Floor, Boston, MA 02110-1301, USA。
*/

#ifndef __FILTER_H__
#define __FILTER_H__

#include <emmintrin.h>
#include <pmmintrin.h>
 
// 为兼容 Visual Studio：这里不再区分编译器手动 typedef，
// 而是统一包含 <stdint.h>，使用其中定义的 uint8_t / int16_t / int32_t 等定长整数类型，
// 避免与 VS 自带 <stdint.h> 中的定义重复
#include <stdint.h>

// 快速滤波器：实现 3x3 和 5x5 Sobel 滤波，以及基于 SSE2/3 指令的
//             5x5 blob 与 corner 滤波器
namespace filter {
  
  // 私有命名空间，供内部实现使用；对外接口在文件底部给出
  namespace detail {
    void integral_image( const uint8_t* in, int32_t* out, int w, int h );
    void unpack_8bit_to_16bit( const __m128i a, __m128i& b0, __m128i& b1 );
    void pack_16bit_to_8bit_saturate( const __m128i a0, const __m128i a1, __m128i& b );
    
    // 使用 (1,4,6,4,1) 行向量对图像做卷积，结果累加到输出中。
    // 输出按 1/128 进行缩放，再截断到 [-128,128]，最后平移到 [0,255]。
    void convolve_14641_row_5x5_16bit( const int16_t* in, uint8_t* out, int w, int h );
    
    // 使用 (1,2,0,-2,-1) 行向量对图像做卷积，结果累加到输出中。
    // 该函数以 16 位输入、8 位输出形式工作。
    // 输出按 1/128 进行缩放，再截断到 [-128,128]，最后平移到 [0,255]。
    void convolve_12021_row_5x5_16bit( const int16_t* in, uint8_t* out, int w, int h );

    // 使用 (1,2,1) 行向量对图像做卷积，结果累加到输出中。
    // 该函数以 16 位输入、8 位输出形式工作。
    // 输出按 1/4 进行缩放，再截断到 [-128,128]，最后平移到 [0,255]。
    void convolve_121_row_3x3_16bit( const int16_t* in, uint8_t* out, int w, int h );
    
    // 使用 (1,0,-1) 行向量对图像做卷积，结果累加到输出中。
    // 该函数以 16 位输入、8 位输出形式工作。
    // 输出按 1/4 进行缩放，再截断到 [-128,128]，最后平移到 [0,255]。
    void convolve_101_row_3x3_16bit( const int16_t* in, uint8_t* out, int w, int h );
    
    void convolve_cols_5x5( const unsigned char* in, int16_t* out_v, int16_t* out_h, int w, int h );
    
    void convolve_col_p1p1p0m1m1_5x5( const unsigned char* in, int16_t* out, int w, int h );
    
    void convolve_row_p1p1p0m1m1_5x5( const int16_t* in, int16_t* out, int w, int h );
    
    void convolve_cols_3x3( const unsigned char* in, int16_t* out_v, int16_t* out_h, int w, int h );
  }
  
  void sobel3x3( const uint8_t* in, uint8_t* out_v, uint8_t* out_h, int w, int h );  // 3x3 Sobel 滤波
  
  void sobel5x5( const uint8_t* in, uint8_t* out_v, uint8_t* out_h, int w, int h );  // 5x5 Sobel 滤波
  
  // -1 -1  0  1  1
  // -1 -1  0  1  1
  //  0  0  0  0  0
  //  1  1  0 -1 -1
  //  1  1  0 -1 -1
  void checkerboard5x5( const uint8_t* in, int16_t* out, int w, int h );
  
  // -1 -1 -1 -1 -1
  // -1  1  1  1 -1
  // -1  1  8  1 -1
  // -1  1  1  1 -1
  // -1 -1 -1 -1 -1
  void blob5x5( const uint8_t* in, int16_t* out, int w, int h );
};

#endif
