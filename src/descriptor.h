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

// 说明：该描述子是论文中 50 维描述子的稀疏近似，
// 产生的结果与原描述子类似，
// 但计算速度更快。

#ifndef __DESCRIPTOR_H__
#define __DESCRIPTOR_H__

#include <iostream>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <math.h>
 
// 为兼容 Visual Studio：直接包含标准头 <stdint.h>，使用标准的 int8_t / uint8_t 等类型，
// 替代原来在 MSVC 分支里手动 typedef 的做法，从而避免与 VS 自带定义发生重定义冲突
#include <stdint.h>

class Descriptor {
  
public:
  
  // 构造函数：根据输入图像创建描述子
  Descriptor(uint8_t* I,int32_t width,int32_t height,int32_t bpl,bool half_resolution);
  
  // 析构函数：释放内部申请的内存
  ~Descriptor();
  
  // 外部可访问的描述子数据
  uint8_t* I_desc;
  
private:

  // 根据 I_du 和 I_dv 构建 I_desc 描述子
  void createDescriptor(uint8_t* I_du,uint8_t* I_dv,int32_t width,int32_t height,int32_t bpl,bool half_resolution);

};

#endif
