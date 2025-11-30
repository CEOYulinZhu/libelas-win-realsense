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

#include "descriptor.h"
#include "filter.h"
#include <emmintrin.h>

using namespace std;

Descriptor::Descriptor(uint8_t* I,int32_t width,int32_t height,int32_t bpl,bool half_resolution) {
  I_desc        = (uint8_t*)_mm_malloc(16*width*height*sizeof(uint8_t),16);
  uint8_t* I_du = (uint8_t*)_mm_malloc(bpl*height*sizeof(uint8_t),16);
  uint8_t* I_dv = (uint8_t*)_mm_malloc(bpl*height*sizeof(uint8_t),16);
  filter::sobel3x3(I,I_du,I_dv,bpl,height);
  createDescriptor(I_du,I_dv,width,height,bpl,half_resolution);
  _mm_free(I_du);
  _mm_free(I_dv);
}

Descriptor::~Descriptor() {
  _mm_free(I_desc);
}

void Descriptor::createDescriptor (uint8_t* I_du,uint8_t* I_dv,int32_t width,int32_t height,int32_t bpl,bool half_resolution) {

  uint8_t *I_desc_curr;  
  uint32_t addr_v0,addr_v1,addr_v2,addr_v3,addr_v4;
  
  // 在 half_resolution 模式下：只在隔行位置计算描述子
  if (half_resolution) {
  
    // 创建用于滤波的条带
    for (int32_t v=4; v<height-3; v+=2) {

      addr_v2 = v*bpl;
      addr_v0 = addr_v2-2*bpl;
      addr_v1 = addr_v2-1*bpl;
      addr_v3 = addr_v2+1*bpl;
      addr_v4 = addr_v2+2*bpl;

      for (int32_t u=3; u<width-3; u++) {
        I_desc_curr = I_desc+(v*width+u)*16;
        *(I_desc_curr++) = *(I_du+addr_v0+u+0);
        *(I_desc_curr++) = *(I_du+addr_v1+u-2);
        *(I_desc_curr++) = *(I_du+addr_v1+u+0);
        *(I_desc_curr++) = *(I_du+addr_v1+u+2);
        *(I_desc_curr++) = *(I_du+addr_v2+u-1);
        *(I_desc_curr++) = *(I_du+addr_v2+u+0);
        *(I_desc_curr++) = *(I_du+addr_v2+u+0);
        *(I_desc_curr++) = *(I_du+addr_v2+u+1);
        *(I_desc_curr++) = *(I_du+addr_v3+u-2);
        *(I_desc_curr++) = *(I_du+addr_v3+u+0);
        *(I_desc_curr++) = *(I_du+addr_v3+u+2);
        *(I_desc_curr++) = *(I_du+addr_v4+u+0);
        *(I_desc_curr++) = *(I_dv+addr_v1+u+0);
        *(I_desc_curr++) = *(I_dv+addr_v2+u-1);
        *(I_desc_curr++) = *(I_dv+addr_v2+u+1);
        *(I_desc_curr++) = *(I_dv+addr_v3+u+0);
      }
    }
    
  // 否则：计算完整分辨率的描述子图像
  } else {
    
    // 创建用于滤波的条带
    for (int32_t v=3; v<height-3; v++) {

      addr_v2 = v*bpl;
      addr_v0 = addr_v2-2*bpl;
      addr_v1 = addr_v2-1*bpl;
      addr_v3 = addr_v2+1*bpl;
      addr_v4 = addr_v2+2*bpl;

      for (int32_t u=3; u<width-3; u++) {
        I_desc_curr = I_desc+(v*width+u)*16;
        *(I_desc_curr++) = *(I_du+addr_v0+u+0);
        *(I_desc_curr++) = *(I_du+addr_v1+u-2);
        *(I_desc_curr++) = *(I_du+addr_v1+u+0);
        *(I_desc_curr++) = *(I_du+addr_v1+u+2);
        *(I_desc_curr++) = *(I_du+addr_v2+u-1);
        *(I_desc_curr++) = *(I_du+addr_v2+u+0);
        *(I_desc_curr++) = *(I_du+addr_v2+u+0);
        *(I_desc_curr++) = *(I_du+addr_v2+u+1);
        *(I_desc_curr++) = *(I_du+addr_v3+u-2);
        *(I_desc_curr++) = *(I_du+addr_v3+u+0);
        *(I_desc_curr++) = *(I_du+addr_v3+u+2);
        *(I_desc_curr++) = *(I_du+addr_v4+u+0);
        *(I_desc_curr++) = *(I_dv+addr_v1+u+0);
        *(I_desc_curr++) = *(I_dv+addr_v2+u-1);
        *(I_desc_curr++) = *(I_dv+addr_v2+u+1);
        *(I_desc_curr++) = *(I_dv+addr_v3+u+0);
      }
    }
  }
  
}
