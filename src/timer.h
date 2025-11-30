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

#ifndef __TIMER_H__
#define __TIMER_H__

#include <iostream>
#include <iomanip>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <vector>
#include <string>
#include <sys/time.h>

// 为 Visual Studio 工程定义定长整数类型
#ifndef _MSC_VER
  #include <stdint.h>
#else
  typedef __int8            int8_t;
  typedef __int16           int16_t;
  typedef __int32           int32_t;
  typedef __int64           int64_t;
  typedef unsigned __int8   uint8_t;
  typedef unsigned __int16  uint16_t;
  typedef unsigned __int32  uint32_t;
  typedef unsigned __int64  uint64_t;
#endif

class Timer {
  
public:
  
  Timer() {}
  
  ~Timer() {}
  
  void start (std::string title) {
    desc.push_back(title);
    push_back_time();
  }
  
  void stop () {
    if (time.size()<=desc.size())
      push_back_time();
  }
  
  void plot () {
    stop();
    float total_time = 0;
    for (int32_t i=0; i<desc.size(); i++) {
      float curr_time = getTimeDifferenceMilliseconds(time[i],time[i+1]);
      total_time += curr_time;
      std::cout.width(30);
      std::cout << desc[i] << " ";
      std::cout << std::fixed << std::setprecision(1) << std::setw(6);
      std::cout << curr_time;
      std::cout << " ms" << std::endl;
    }
    std::cout << "========================================" << std::endl;
    std::cout << "                    Total time ";
    std::cout << std::fixed << std::setprecision(1) << std::setw(6);
    std::cout << total_time;
    std::cout << " ms" << std::endl << std::endl;
  }
  
  void reset () {
    desc.clear();
    time.clear();
  }
  
private:
  
  std::vector<std::string>  desc;
  std::vector<timeval>      time;
  
  void push_back_time () {
    timeval curr_time;
    gettimeofday(&curr_time,0);
    time.push_back(curr_time);
  }
  
  float getTimeDifferenceMilliseconds(timeval a,timeval b) {
    return ((float)(b.tv_sec -a.tv_sec ))*1e+3 +
           ((float)(b.tv_usec-a.tv_usec))*1e-3;
  }
};

#endif
