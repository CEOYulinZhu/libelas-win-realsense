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

#include "elas.h"

#include <algorithm>
#include <math.h>
#include "descriptor.h"
#include "triangle.h"
#include "matrix.h"

using namespace std;

void Elas::process (uint8_t* I1_,uint8_t* I2_,float* D1,float* D2,const int32_t* dims){
  
  // 获取图像宽度、高度以及每行字节数
  width  = dims[0];
  height = dims[1];
  bpl    = width + 15-(width-1)%16;
  
  // 将图像拷贝到按 16 字节对齐的缓冲区
  I1 = (uint8_t*)_mm_malloc(bpl*height*sizeof(uint8_t),16);
  I2 = (uint8_t*)_mm_malloc(bpl*height*sizeof(uint8_t),16);
  memset (I1,0,bpl*height*sizeof(uint8_t));
  memset (I2,0,bpl*height*sizeof(uint8_t));
  if (bpl==dims[2]) {
    memcpy(I1,I1_,bpl*height*sizeof(uint8_t));
    memcpy(I2,I2_,bpl*height*sizeof(uint8_t));
  } else {
    for (int32_t v=0; v<height; v++) {
      memcpy(I1+v*bpl,I1_+v*dims[2],width*sizeof(uint8_t));
      memcpy(I2+v*bpl,I2_+v*dims[2],width*sizeof(uint8_t));
    }
  }

#ifdef PROFILE
  timer.start("Descriptor");  
#endif
  Descriptor desc1(I1,width,height,bpl,param.subsampling);
  Descriptor desc2(I2,width,height,bpl,param.subsampling);

#ifdef PROFILE
  timer.start("Support Matches");
#endif
  vector<support_pt> p_support = computeSupportMatches(desc1.I_desc,desc2.I_desc);
  
  // 如果支持点数量不足以进行三角剖分
  if (p_support.size()<3) {
    cout << "ERROR: Need at least 3 support points!" << endl;
    _mm_free(I1);
    _mm_free(I2);
    return;
  }

#ifdef PROFILE
  timer.start("Delaunay Triangulation");
#endif
  vector<triangle> tri_1 = computeDelaunayTriangulation(p_support,0);
  vector<triangle> tri_2 = computeDelaunayTriangulation(p_support,1);

#ifdef PROFILE
  timer.start("Disparity Planes");
#endif
  computeDisparityPlanes(p_support,tri_1,0);
  computeDisparityPlanes(p_support,tri_2,1);

#ifdef PROFILE
  timer.start("Grid");
#endif

  // 为视差网格分配内存
  int32_t grid_width   = (int32_t)ceil((float)width/(float)param.grid_size);
  int32_t grid_height  = (int32_t)ceil((float)height/(float)param.grid_size);
  int32_t grid_dims[3] = {param.disp_max+2,grid_width,grid_height};
  int32_t* disparity_grid_1 = (int32_t*)calloc((param.disp_max+2)*grid_height*grid_width,sizeof(int32_t));
  int32_t* disparity_grid_2 = (int32_t*)calloc((param.disp_max+2)*grid_height*grid_width,sizeof(int32_t));
  
  createGrid(p_support,disparity_grid_1,grid_dims,0);
  createGrid(p_support,disparity_grid_2,grid_dims,1);

#ifdef PROFILE
  timer.start("Matching");
#endif
  computeDisparity(p_support,tri_1,disparity_grid_1,grid_dims,desc1.I_desc,desc2.I_desc,0,D1);
  computeDisparity(p_support,tri_2,disparity_grid_2,grid_dims,desc1.I_desc,desc2.I_desc,1,D2);

#ifdef PROFILE
  timer.start("L/R Consistency Check");
#endif
  leftRightConsistencyCheck(D1,D2);

#ifdef PROFILE
  timer.start("Remove Small Segments");
#endif
  removeSmallSegments(D1);
  if (!param.postprocess_only_left)
    removeSmallSegments(D2);

#ifdef PROFILE
  timer.start("Gap Interpolation");
#endif
  gapInterpolation(D1);
  if (!param.postprocess_only_left)
    gapInterpolation(D2);

  if (param.filter_adaptive_mean) {
#ifdef PROFILE
    timer.start("Adaptive Mean");
#endif
    adaptiveMean(D1);
    if (!param.postprocess_only_left)
      adaptiveMean(D2);
  }

  if (param.filter_median) {
#ifdef PROFILE
    timer.start("Median");
#endif
    median(D1);
    if (!param.postprocess_only_left)
      median(D2);
  }

#ifdef PROFILE
  timer.plot();
#endif

  // 释放内存
  free(disparity_grid_1);
  free(disparity_grid_2);
  _mm_free(I1);
  _mm_free(I2);
}

void Elas::removeInconsistentSupportPoints (int16_t* D_can,int32_t D_can_width,int32_t D_can_height) {
  
  // 遍历所有有效的支持点
  for (int32_t u_can=0; u_can<D_can_width; u_can++) {
    for (int32_t v_can=0; v_can<D_can_height; v_can++) {
      int16_t d_can = *(D_can+getAddressOffsetImage(u_can,v_can,D_can_width));
      if (d_can>=0) {
        
        // 统计在邻域内与当前支持点视差接近的点的数量
        int32_t support = 0;
        for (int32_t u_can_2=u_can-param.incon_window_size; u_can_2<=u_can+param.incon_window_size; u_can_2++) {
          for (int32_t v_can_2=v_can-param.incon_window_size; v_can_2<=v_can+param.incon_window_size; v_can_2++) {
            if (u_can_2>=0 && v_can_2>=0 && u_can_2<D_can_width && v_can_2<D_can_height) {
              int16_t d_can_2 = *(D_can+getAddressOffsetImage(u_can_2,v_can_2,D_can_width));
              if (d_can_2>=0 && abs(d_can-d_can_2)<=param.incon_threshold)
                support++;
            }
          }
        }
        
        // 若邻域内支持当前点的数量过少，则将该支持点标记为无效
        if (support<param.incon_min_support)
          *(D_can+getAddressOffsetImage(u_can,v_can,D_can_width)) = -1;
      }
    }
  }
}

void Elas::removeRedundantSupportPoints(int16_t* D_can,int32_t D_can_width,int32_t D_can_height,
                                        int32_t redun_max_dist, int32_t redun_threshold, bool vertical) {
  
  // 冗余检测方向相关的参数
  int32_t redun_dir_u[2] = {0,0};
  int32_t redun_dir_v[2] = {0,0};
  if (vertical) {
    redun_dir_v[0] = -1;
    redun_dir_v[1] = +1;
  } else {
    redun_dir_u[0] = -1;
    redun_dir_u[1] = +1;
  }
    
  // 遍历所有有效的支持点
  for (int32_t u_can=0; u_can<D_can_width; u_can++) {
    for (int32_t v_can=0; v_can<D_can_height; v_can++) {
      int16_t d_can = *(D_can+getAddressOffsetImage(u_can,v_can,D_can_width));
      if (d_can>=0) {
        
        // 在两个方向上检查该支持点是否存在冗余
        bool redundant = true;
        for (int32_t i=0; i<2; i++) {
          
          // 沿当前方向搜索是否存在与当前点视差足够接近的支持点
          int32_t u_can_2 = u_can;
          int32_t v_can_2 = v_can;
          int16_t d_can_2;
          bool support = false;
          for (int32_t j=0; j<redun_max_dist; j++) {
            u_can_2 += redun_dir_u[i];
            v_can_2 += redun_dir_v[i];
            if (u_can_2<0 || v_can_2<0 || u_can_2>=D_can_width || v_can_2>=D_can_height)
              break;
            d_can_2 = *(D_can+getAddressOffsetImage(u_can_2,v_can_2,D_can_width));
            if (d_can_2>=0 && abs(d_can-d_can_2)<=redun_threshold) {
              support = true;
              break;
            }
          }
          
          // 如果在该方向上没有找到满足条件的支持点，则当前点不再视为冗余
          if (!support) {
            redundant = false;
            break;
          }
        }
               
        // 若确认该点冗余，则将其标记为无效
        if (redundant)
          *(D_can+getAddressOffsetImage(u_can,v_can,D_can_width)) = -1;
      }
    }
  }
}

void Elas::addCornerSupportPoints(vector<support_pt> &p_support) {
  
  // 图像边界上的四个角点
  vector<support_pt> p_border;
  p_border.push_back(support_pt(0,0,0));
  p_border.push_back(support_pt(0,height-1,0));
  p_border.push_back(support_pt(width-1,0,0));
  p_border.push_back(support_pt(width-1,height-1,0));
  
  // 为每个角点找到最近的支持点视差
  for (int32_t i=0; i<p_border.size(); i++) {
    int32_t best_dist = 10000000;
    for (int32_t j=0; j<p_support.size(); j++) {
      int32_t du = p_border[i].u-p_support[j].u;
      int32_t dv = p_border[i].v-p_support[j].v;
      int32_t curr_dist = du*du+dv*dv;
      if (curr_dist<best_dist) {
        best_dist = curr_dist;
        p_border[i].d = p_support[j].d;
      }
    }
  }
  
  // 右图中的角点（u 需要加上视差）
  p_border.push_back(support_pt(p_border[2].u+p_border[2].d,p_border[2].v,p_border[2].d));
  p_border.push_back(support_pt(p_border[3].u+p_border[3].d,p_border[3].v,p_border[3].d));
  
  // 将角点加入支持点集合
  for (int32_t i=0; i<p_border.size(); i++)
    p_support.push_back(p_border[i]);
}

inline int16_t Elas::computeMatchingDisparity (const int32_t &u,const int32_t &v,uint8_t* I1_desc,uint8_t* I2_desc,const bool &right_image) {
  
  const int32_t u_step      = 2;
  const int32_t v_step      = 2;
  const int32_t window_size = 3;
  
  int32_t desc_offset_1 = -16*u_step-16*width*v_step;
  int32_t desc_offset_2 = +16*u_step-16*width*v_step;
  int32_t desc_offset_3 = -16*u_step+16*width*v_step;
  int32_t desc_offset_4 = +16*u_step+16*width*v_step;
  
  __m128i xmm1,xmm2,xmm3,xmm4,xmm5,xmm6;

  // 检查当前 (u, v) 是否在可匹配的图像区域内
  if (u>=window_size+u_step && u<=width-window_size-1-u_step && v>=window_size+v_step && v<=height-window_size-1-v_step) {
    
    // 计算描述子数据和行起始地址
    int32_t  line_offset = 16*width*v;
    uint8_t *I1_line_addr,*I2_line_addr;
    if (!right_image) {
      I1_line_addr = I1_desc+line_offset;
      I2_line_addr = I2_desc+line_offset;
    } else {
      I1_line_addr = I2_desc+line_offset;
      I2_line_addr = I1_desc+line_offset;
    }

    // 计算 I1 中当前块的起始地址
    uint8_t* I1_block_addr = I1_line_addr+16*u;
    uint8_t* I2_block_addr;
    
    // 要求该块具有一定的纹理强度
    int32_t sum = 0;
    for (int32_t i=0; i<16; i++)
      sum += abs((int32_t)(*(I1_block_addr+i))-128);
    if (sum<param.support_texture)
      return -1;
    
    // 将 I1 中前四个 4×4 描述子块加载到寄存器
    xmm1 = _mm_load_si128((__m128i*)(I1_block_addr+desc_offset_1));
    xmm2 = _mm_load_si128((__m128i*)(I1_block_addr+desc_offset_2));
    xmm3 = _mm_load_si128((__m128i*)(I1_block_addr+desc_offset_3));
    xmm4 = _mm_load_si128((__m128i*)(I1_block_addr+desc_offset_4));
    
    // 为每个候选视差准备匹配代价值
    int32_t u_warp;
    
    // 记录最佳与次佳匹配
    int16_t min_1_E = 32767;
    int16_t min_1_d = -1;
    int16_t min_2_E = 32767;
    int16_t min_2_d = -1;

    // 计算当前像素可用的视差范围
    int32_t disp_min_valid = max(param.disp_min,0);
    int32_t disp_max_valid = param.disp_max;
    if (!right_image) disp_max_valid = min(param.disp_max,u-window_size-u_step);
    else              disp_max_valid = min(param.disp_max,width-u-window_size-u_step);
    
    // 要求该像素至少可以评估 10 个不同视差，否则视为无效
    if (disp_max_valid-disp_min_valid<10)
      return -1;

    // 遍历所有候选视差
    for (int16_t d=disp_min_valid; d<=disp_max_valid; d++) {

      // 对应左/右图时，根据视差对 u 坐标做平移
      if (!right_image) u_warp = u-d;
      else              u_warp = u+d;

      // 计算右图中匹配块的起始地址
      I2_block_addr = I2_line_addr+16*u_warp;

      // 在该视差下计算匹配代价
      xmm6 = _mm_load_si128((__m128i*)(I2_block_addr+desc_offset_1));
      xmm6 = _mm_sad_epu8(xmm1,xmm6);
      xmm5 = _mm_load_si128((__m128i*)(I2_block_addr+desc_offset_2));
      xmm6 = _mm_add_epi16(_mm_sad_epu8(xmm2,xmm5),xmm6);
      xmm5 = _mm_load_si128((__m128i*)(I2_block_addr+desc_offset_3));
      xmm6 = _mm_add_epi16(_mm_sad_epu8(xmm3,xmm5),xmm6);
      xmm5 = _mm_load_si128((__m128i*)(I2_block_addr+desc_offset_4));
      xmm6 = _mm_add_epi16(_mm_sad_epu8(xmm4,xmm5),xmm6);
      sum  = _mm_extract_epi16(xmm6,0)+_mm_extract_epi16(xmm6,4);

      // 维护当前最佳与次佳匹配
      if (sum<min_1_E) {
        min_2_E = min_1_E;   
        min_2_d = min_1_d;
        min_1_E = sum;
        min_1_d = d;
      } else if (sum<min_2_E) {
        min_2_E = sum;
        min_2_d = d;
      }
    }

    // 检查是否同时存在最佳与次佳匹配，且代价比满足唯一性约束
    if (min_1_d>=0 && min_2_d>=0 && (float)min_1_E<param.support_threshold*(float)min_2_E)
      return min_1_d;
    else
      return -1;
    
  } else
    return -1;
}

vector<Elas::support_pt> Elas::computeSupportMatches (uint8_t* I1_desc,uint8_t* I2_desc) {
  
  // 注意：在半分辨率模式下，只需要使用每隔一行的数据
  int32_t D_candidate_stepsize = param.candidate_stepsize;
  if (param.subsampling)
    D_candidate_stepsize += D_candidate_stepsize%2;

  // 为视差候选结果创建矩阵
  int32_t D_can_width  = 0;
  int32_t D_can_height = 0;
  for (int32_t u=0; u<width;  u+=D_candidate_stepsize) D_can_width++;
  for (int32_t v=0; v<height; v+=D_candidate_stepsize) D_can_height++;
  int16_t* D_can = (int16_t*)calloc(D_can_width*D_can_height,sizeof(int16_t));

  // 循环变量
  int32_t u,v;
  int16_t d,d2;
   
  // 遍历图像 1 中所有候选点
  for (int32_t u_can=1; u_can<D_can_width; u_can++) {
    u = u_can*D_candidate_stepsize;
    for (int32_t v_can=1; v_can<D_can_height; v_can++) {
      v = v_can*D_candidate_stepsize;
      
      // 将该候选点的视差初始化为无效
      *(D_can+getAddressOffsetImage(u_can,v_can,D_can_width)) = -1;
      
      // 先在左→右方向上匹配
      d = computeMatchingDisparity(u,v,I1_desc,I2_desc,false);
      if (d>=0) {
        
        // 再在右→左方向验证匹配
        d2 = computeMatchingDisparity(u-d,v,I1_desc,I2_desc,true);
        if (d2>=0 && abs(d-d2)<=param.lr_threshold)
          *(D_can+getAddressOffsetImage(u_can,v_can,D_can_width)) = d;
      }
    }
  }
  
  // 移除不一致的支持点
  removeInconsistentSupportPoints(D_can,D_can_width,D_can_height);
  
  // 移除位于直线区域上的支持点（这类点通常是冗余的），
  // 可以减小三角形数量，从而加快后续三角剖分过程
  removeRedundantSupportPoints(D_can,D_can_width,D_can_height,5,1,true);
  removeRedundantSupportPoints(D_can,D_can_width,D_can_height,5,1,false);
  
  // 将图像坐标中的支持点转换为向量表示
  vector<support_pt> p_support;
  for (int32_t u_can=1; u_can<D_can_width; u_can++)
    for (int32_t v_can=1; v_can<D_can_height; v_can++)
      if (*(D_can+getAddressOffsetImage(u_can,v_can,D_can_width))>=0)
        p_support.push_back(support_pt(u_can*D_candidate_stepsize,
                                       v_can*D_candidate_stepsize,
                                       *(D_can+getAddressOffsetImage(u_can,v_can,D_can_width))));
  
  // 若标志位开启，则在图像四角补充支持点，
  // 其视差取自最近的已有支持点
  if (param.add_corners)
    addCornerSupportPoints(p_support);

  // 释放临时内存
  free(D_can);
  
  // 返回支持点向量
  return p_support; 
}

vector<Elas::triangle> Elas::computeDelaunayTriangulation (vector<support_pt> p_support,int32_t right_image) {

  // 三角剖分的输入 / 输出结构体
  struct triangulateio in, out;
  int32_t k;

  // 输入部分
  in.numberofpoints = p_support.size();
  in.pointlist = (float*)malloc(in.numberofpoints*2*sizeof(float));
  k=0;
  if (!right_image) {
    for (int32_t i=0; i<p_support.size(); i++) {
      in.pointlist[k++] = p_support[i].u;
      in.pointlist[k++] = p_support[i].v;
    }
  } else {
    for (int32_t i=0; i<p_support.size(); i++) {
      in.pointlist[k++] = p_support[i].u-p_support[i].d;
      in.pointlist[k++] = p_support[i].v;
    }
  }
  in.numberofpointattributes = 0;
  in.pointattributelist      = NULL;
  in.pointmarkerlist         = NULL;
  in.numberofsegments        = 0;
  in.numberofholes           = 0;
  in.numberofregions         = 0;
  in.regionlist              = NULL;
  
  // 输出部分
  out.pointlist              = NULL;
  out.pointattributelist     = NULL;
  out.pointmarkerlist        = NULL;
  out.trianglelist           = NULL;
  out.triangleattributelist  = NULL;
  out.neighborlist           = NULL;
  out.segmentlist            = NULL;
  out.segmentmarkerlist      = NULL;
  out.edgelist               = NULL;
  out.edgemarkerlist         = NULL;

  // 进行三角剖分（z=从 0 开始编号，n=输出邻接关系，Q=安静模式，B=不输出边界标记）
  char parameters[] = "zQB";
  triangulate(parameters, &in, &out, NULL);
  
  // 将三角形结果写入 tri 向量
  vector<triangle> tri;
  k=0;
  for (int32_t i=0; i<out.numberoftriangles; i++) {
    tri.push_back(triangle(out.trianglelist[k],out.trianglelist[k+1],out.trianglelist[k+2]));
    k+=3;
  }
  
  // 释放三角剖分过程中申请的内存
  free(in.pointlist);
  free(out.pointlist);
  free(out.trianglelist);
  
  // 返回三角形列表
  return tri;
}

void Elas::computeDisparityPlanes (vector<support_pt> p_support,vector<triangle> &tri,int32_t right_image) {

  // 初始化线性方程组所需的矩阵
  Matrix A(3,3);
  Matrix b(3,1);
  
  // 对每个三角形计算其对应的视差平面
  for (int32_t i=0; i<tri.size(); i++) {
    
    // 取得三角形三个顶点在支持点向量中的索引
    int32_t c1 = tri[i].c1;
    int32_t c2 = tri[i].c2;
    int32_t c3 = tri[i].c3;
    
    // 构建左图三角形对应线性系统的系数矩阵 A
    A.val[0][0] = p_support[c1].u;
    A.val[1][0] = p_support[c2].u;
    A.val[2][0] = p_support[c3].u;
    A.val[0][1] = p_support[c1].v; A.val[0][2] = 1;
    A.val[1][1] = p_support[c2].v; A.val[1][2] = 1;
    A.val[2][1] = p_support[c3].v; A.val[2][2] = 1;
    
    // 构建右端向量 b（包含三个顶点的视差）
    b.val[0][0] = p_support[c1].d;
    b.val[1][0] = p_support[c2].d;
    b.val[2][0] = p_support[c3].d;
    
    // 若高斯-约旦消元成功
    if (b.solve(A)) {
      
      // 从 b 中读取平面参数
      tri[i].t1a = b.val[0][0];
      tri[i].t1b = b.val[1][0];
      tri[i].t1c = b.val[2][0];
      
    // 否则标记为无效平面
    } else {
      tri[i].t1a = 0;
      tri[i].t1b = 0;
      tri[i].t1c = 0;
    }

    // 为右图中的对应三角形重新构建系数矩阵 A
    A.val[0][0] = p_support[c1].u-p_support[c1].d;
    A.val[1][0] = p_support[c2].u-p_support[c2].d;
    A.val[2][0] = p_support[c3].u-p_support[c3].d;
    A.val[0][1] = p_support[c1].v; A.val[0][2] = 1;
    A.val[1][1] = p_support[c2].v; A.val[1][2] = 1;
    A.val[2][1] = p_support[c3].v; A.val[2][2] = 1;
    
    // 构建右图线性系统的右端向量 b（包含三个顶点的视差）
    b.val[0][0] = p_support[c1].d;
    b.val[1][0] = p_support[c2].d;
    b.val[2][0] = p_support[c3].d;
    
    // 若高斯-约旦消元成功
    if (b.solve(A)) {
      
      // 从 b 中读取右图平面参数
      tri[i].t2a = b.val[0][0];
      tri[i].t2b = b.val[1][0];
      tri[i].t2c = b.val[2][0];
      
    // 否则标记为无效平面
    } else {
      tri[i].t2a = 0;
      tri[i].t2b = 0;
      tri[i].t2c = 0;
    }
  }  
}

void Elas::createGrid(vector<support_pt> p_support,int32_t* disparity_grid,int32_t* grid_dims,bool right_image) {
  
  // 获取视差网格的尺寸
  int32_t grid_width  = grid_dims[1];
  int32_t grid_height = grid_dims[2];
  
  // 为辅助网格分配临时内存
  int32_t* temp1 = (int32_t*)calloc((param.disp_max+1)*grid_height*grid_width,sizeof(int32_t));
  int32_t* temp2 = (int32_t*)calloc((param.disp_max+1)*grid_height*grid_width,sizeof(int32_t));
  
  // 遍历所有支持点
  for (int32_t i=0; i<p_support.size(); i++) {
    
    // 计算此支持点在视差网格中需要填充的视差范围
    int32_t x_curr = p_support[i].u;
    int32_t y_curr = p_support[i].v;
    int32_t d_curr = p_support[i].d;
    int32_t d_min  = max(d_curr-1,0);
    int32_t d_max  = min(d_curr+1,param.disp_max);
    
    // 在临时网格 temp1 中标记该支持点影响到的视差位置
    for (int32_t d=d_min; d<=d_max; d++) {
      int32_t x;
      if (!right_image)
        x = floor((float)(x_curr/param.grid_size));
      else
        x = floor((float)(x_curr-d_curr)/(float)param.grid_size);
      int32_t y = floor((float)y_curr/(float)param.grid_size);
      
      // 角点等情况可能会落在网格边界之外
      if (x>=0 && x<grid_width &&y>=0 && y<grid_height) {
        int32_t addr = getAddressOffsetGrid(x,y,d,grid_width,param.disp_max+1);
        *(temp1+addr) = 1;
      }
    }
  }
  
  // 扩散操作的指针
  const int32_t* tl = temp1 + (0*grid_width+0)*(param.disp_max+1);
  const int32_t* tc = temp1 + (0*grid_width+1)*(param.disp_max+1);
  const int32_t* tr = temp1 + (0*grid_width+2)*(param.disp_max+1);
  const int32_t* cl = temp1 + (1*grid_width+0)*(param.disp_max+1);
  const int32_t* cc = temp1 + (1*grid_width+1)*(param.disp_max+1);
  const int32_t* cr = temp1 + (1*grid_width+2)*(param.disp_max+1);
  const int32_t* bl = temp1 + (2*grid_width+0)*(param.disp_max+1);
  const int32_t* bc = temp1 + (2*grid_width+1)*(param.disp_max+1);
  const int32_t* br = temp1 + (2*grid_width+2)*(param.disp_max+1);
  
  int32_t* result    = temp2 + (1*grid_width+1)*(param.disp_max+1); 
  int32_t* end_input = temp1 + grid_width*grid_height*(param.disp_max+1);
  
  // 对临时网格进行 3×3 邻域扩散
  for( ; br != end_input; tl++, tc++, tr++, cl++, cc++, cr++, bl++, bc++, br++, result++ )
    *result = *tl | *tc | *tr | *cl | *cc | *cr | *bl | *bc | *br;
  
  // 遍历所有网格单元，生成最终的视差候选列表
  for (int32_t x=0; x<grid_width; x++) {
    for (int32_t y=0; y<grid_height; y++) {
        
      // 从索引 1 开始，索引 0 保留用于存储视差数量
      int32_t curr_ind = 1;
      
      // 遍历全部可能的视差
      for (int32_t d=0; d<=param.disp_max; d++) {

        // 若该视差在扩散后的临时网格中被标记，则加入当前网格单元
        if (*(temp2+getAddressOffsetGrid(x,y,d,grid_width,param.disp_max+1))>0) {
          *(disparity_grid+getAddressOffsetGrid(x,y,curr_ind,grid_width,param.disp_max+2))=d;
          curr_ind++;
        }
      }
      
      // 最后在索引 0 处写入当前单元中的视差数量
      *(disparity_grid+getAddressOffsetGrid(x,y,0,grid_width,param.disp_max+2))=curr_ind-1;
    }
  }
  
  // 释放临时网格内存
  free(temp1);
  free(temp2);
}

inline void Elas::updatePosteriorMinimum(__m128i* I2_block_addr,const int32_t &d,const int32_t &w,
                                         const __m128i &xmm1,__m128i &xmm2,int32_t &val,int32_t &min_val,int32_t &min_d) {
  xmm2 = _mm_load_si128(I2_block_addr);
  xmm2 = _mm_sad_epu8(xmm1,xmm2);
  val  = _mm_extract_epi16(xmm2,0)+_mm_extract_epi16(xmm2,4)+w;
  if (val<min_val) {
    min_val = val;
    min_d   = d;
  }
}

inline void Elas::updatePosteriorMinimum(__m128i* I2_block_addr,const int32_t &d,
                                         const __m128i &xmm1,__m128i &xmm2,int32_t &val,int32_t &min_val,int32_t &min_d) {
  xmm2 = _mm_load_si128(I2_block_addr);
  xmm2 = _mm_sad_epu8(xmm1,xmm2);
  val  = _mm_extract_epi16(xmm2,0)+_mm_extract_epi16(xmm2,4);
  if (val<min_val) {
    min_val = val;
    min_d   = d;
  }
}

inline void Elas::findMatch(int32_t &u,int32_t &v,float &plane_a,float &plane_b,float &plane_c,
                            int32_t* disparity_grid,int32_t *grid_dims,uint8_t* I1_desc,uint8_t* I2_desc,
                            int32_t *P,int32_t &plane_radius,bool &valid,bool &right_image,float* D){
  
  // 获取与视差计算相关的参数（视差个数与窗口尺寸）
  const int32_t disp_num    = grid_dims[0]-1;
  const int32_t window_size = 2;

  // 目标视差在视差图中的地址
  uint32_t d_addr;
  if (param.subsampling) d_addr = getAddressOffsetImage(u/2,v/2,width/2);
  else                   d_addr = getAddressOffsetImage(u,v,width);
  
  // 检查 u 是否落在有效范围内
  if (u<window_size || u>=width-window_size)
    return;

  // 计算当前行在描述子缓冲中的起始偏移
  int32_t  line_offset = 16*width*max(min(v,height-3),2);
  uint8_t *I1_line_addr,*I2_line_addr;
  if (!right_image) {
    I1_line_addr = I1_desc+line_offset;
    I2_line_addr = I2_desc+line_offset;
  } else {
    I1_line_addr = I2_desc+line_offset;
    I2_line_addr = I1_desc+line_offset;
  }

  // 计算 I1 中当前块的起始地址
  uint8_t* I1_block_addr = I1_line_addr+16*u;
  
  // 检查该块是否具有足够的纹理
  int32_t sum = 0;
  for (int32_t i=0; i<16; i++)
    sum += abs((int32_t)(*(I1_block_addr+i))-128);
  if (sum<param.match_texture)
    return;

  // 根据当前平面先验计算视差、最小视差和最大视差
  int32_t d_plane     = (int32_t)(plane_a*(float)u+plane_b*(float)v+plane_c);
  int32_t d_plane_min = max(d_plane-plane_radius,0);
  int32_t d_plane_max = min(d_plane+plane_radius,disp_num-1);

  // 获取当前像素所在网格单元及其视差候选列表
  int32_t  grid_x    = (int32_t)floor((float)u/(float)param.grid_size);
  int32_t  grid_y    = (int32_t)floor((float)v/(float)param.grid_size);
  uint32_t grid_addr = getAddressOffsetGrid(grid_x,grid_y,0,grid_dims[1],grid_dims[0]);  
  int32_t  num_grid  = *(disparity_grid+grid_addr);
  int32_t* d_grid    = disparity_grid+grid_addr+1;
  
  // 循环变量
  int32_t d_curr, u_warp, val;
  int32_t min_val = 10000;
  int32_t min_d   = -1;
  __m128i xmm1    = _mm_load_si128((__m128i*)I1_block_addr);
  __m128i xmm2;

  // 左图
  if (!right_image) { 
    for (int32_t i=0; i<num_grid; i++) {
      d_curr = d_grid[i];
      if (d_curr<d_plane_min || d_curr>d_plane_max) {
        u_warp = u-d_curr;
        if (u_warp<window_size || u_warp>=width-window_size)
          continue;
        updatePosteriorMinimum((__m128i*)(I2_line_addr+16*u_warp),d_curr,xmm1,xmm2,val,min_val,min_d);
      }
    }
    for (d_curr=d_plane_min; d_curr<=d_plane_max; d_curr++) {
      u_warp = u-d_curr;
      if (u_warp<window_size || u_warp>=width-window_size)
        continue;
      updatePosteriorMinimum((__m128i*)(I2_line_addr+16*u_warp),d_curr,valid?*(P+abs(d_curr-d_plane)):0,xmm1,xmm2,val,min_val,min_d);
    }
    
  // 右图
  } else {
    for (int32_t i=0; i<num_grid; i++) {
      d_curr = d_grid[i];
      if (d_curr<d_plane_min || d_curr>d_plane_max) {
        u_warp = u+d_curr;
        if (u_warp<window_size || u_warp>=width-window_size)
          continue;
        updatePosteriorMinimum((__m128i*)(I2_line_addr+16*u_warp),d_curr,xmm1,xmm2,val,min_val,min_d);
      }
    }
    for (d_curr=d_plane_min; d_curr<=d_plane_max; d_curr++) {
      u_warp = u+d_curr;
      if (u_warp<window_size || u_warp>=width-window_size)
        continue;
      updatePosteriorMinimum((__m128i*)(I2_line_addr+16*u_warp),d_curr,valid?*(P+abs(d_curr-d_plane)):0,xmm1,xmm2,val,min_val,min_d);
    }
  }

  // 设置最终视差值
  if (min_d>=0) *(D+d_addr) = min_d; // MAP 值（对应负对数概率最小的视差）
  else          *(D+d_addr) = -1;    // 视为无效视差
}

// TODO: 以更优雅的方式处理 %2 这样的运算
void Elas::computeDisparity(vector<support_pt> p_support,vector<triangle> tri,int32_t* disparity_grid,int32_t *grid_dims,
                            uint8_t* I1_desc,uint8_t* I2_desc,bool right_image,float* D) {

  // 视差数
  const int32_t disp_num  = grid_dims[0]-1;
  
  // 描述子窗口大小
  int32_t window_size = 2;
  
  // 将视差图初始化为 -10（表示尚未赋值的状态）
  if (param.subsampling) {
    for (int32_t i=0; i<(width/2)*(height/2); i++)
      *(D+i) = -10;
  } else {
    for (int32_t i=0; i<width*height; i++)
      *(D+i) = -10;
  }
  
  // 预先计算视差差的先验代价
  float two_sigma_squared = 2*param.sigma*param.sigma;
  int32_t* P = new int32_t[disp_num];
  for (int32_t delta_d=0; delta_d<disp_num; delta_d++)
    P[delta_d] = (int32_t)((-log(param.gamma+exp(-delta_d*delta_d/two_sigma_squared))+log(param.gamma))/param.beta);
  int32_t plane_radius = (int32_t)max((float)ceil(param.sigma*param.sradius),(float)2.0);

  // 循环变量
  int32_t c1, c2, c3;
  float plane_a,plane_b,plane_c,plane_d;
  
  // 遍历所有三角形
  for (uint32_t i=0; i<tri.size(); i++) {
    
    // 获取当前三角形对应的视差平面参数
    uint32_t p_i = i*3;
    if (!right_image) {
      plane_a = tri[i].t1a;
      plane_b = tri[i].t1b;
      plane_c = tri[i].t1c;
      plane_d = tri[i].t2a;
    } else {
      plane_a = tri[i].t2a;
      plane_b = tri[i].t2b;
      plane_c = tri[i].t2c;
      plane_d = tri[i].t1a;
    }
    
    // 三角形的三个顶点索引
    c1 = tri[i].c1;
    c2 = tri[i].c2;
    c3 = tri[i].c3;

    // 按 u 坐标从小到大对三个顶点进行排序
    float tri_u[3];
    if (!right_image) {
      tri_u[0] = p_support[c1].u;
      tri_u[1] = p_support[c2].u;
      tri_u[2] = p_support[c3].u;
    } else {
      tri_u[0] = p_support[c1].u-p_support[c1].d;
      tri_u[1] = p_support[c2].u-p_support[c2].d;
      tri_u[2] = p_support[c3].u-p_support[c3].d;
    }
    float tri_v[3] = {p_support[c1].v,p_support[c2].v,p_support[c3].v};
    
    for (uint32_t j=0; j<3; j++) {
      for (uint32_t k=0; k<j; k++) {
        if (tri_u[k]>tri_u[j]) {
          float tri_u_temp = tri_u[j]; tri_u[j] = tri_u[k]; tri_u[k] = tri_u_temp;
          float tri_v_temp = tri_v[j]; tri_v[j] = tri_v[k]; tri_v[k] = tri_v_temp;
        }
      }
    }
    
    // 将排序后的三个顶点重命名为 A、B、C
    float A_u = tri_u[0]; float A_v = tri_v[0];
    float B_u = tri_u[1]; float B_v = tri_v[1];
    float C_u = tri_u[2]; float C_v = tri_v[2];
    
    // 计算连接三角形顶点的直线参数
    float AB_a = 0; float AC_a = 0; float BC_a = 0;
    if ((int32_t)(A_u)!=(int32_t)(B_u)) AB_a = (A_v-B_v)/(A_u-B_u);
    if ((int32_t)(A_u)!=(int32_t)(C_u)) AC_a = (A_v-C_v)/(A_u-C_u);
    if ((int32_t)(B_u)!=(int32_t)(C_u)) BC_a = (B_v-C_v)/(B_u-C_u);
    float AB_b = A_v-AB_a*A_u;
    float AC_b = A_v-AC_a*A_u;
    float BC_b = B_v-BC_a*B_u;
    
    // 只有当该平面自身及其在另一幅图像中的投影不太倾斜时，才认为该平面有效
    bool valid = fabs(plane_a)<0.7 && fabs(plane_d)<0.7;
        
    // 第一部分：沿 A→B 边扫描
    if ((int32_t)(A_u)!=(int32_t)(B_u)) {
      for (int32_t u=max((int32_t)A_u,0); u<min((int32_t)B_u,width); u++){
        if (!param.subsampling || u%2==0) {
          int32_t v_1 = (uint32_t)(AC_a*(float)u+AC_b);
          int32_t v_2 = (uint32_t)(AB_a*(float)u+AB_b);
          for (int32_t v=min(v_1,v_2); v<max(v_1,v_2); v++)
            if (!param.subsampling || v%2==0) {
              findMatch(u,v,plane_a,plane_b,plane_c,disparity_grid,grid_dims,
                        I1_desc,I2_desc,P,plane_radius,valid,right_image,D);
            }
        }
      }
    }

    // 第二部分：沿 B→C 边扫描
    if ((int32_t)(B_u)!=(int32_t)(C_u)) {
      for (int32_t u=max((int32_t)B_u,0); u<min((int32_t)C_u,width); u++){
        if (!param.subsampling || u%2==0) {
          int32_t v_1 = (uint32_t)(AC_a*(float)u+AC_b);
          int32_t v_2 = (uint32_t)(BC_a*(float)u+BC_b);
          for (int32_t v=min(v_1,v_2); v<max(v_1,v_2); v++)
            if (!param.subsampling || v%2==0) {
              findMatch(u,v,plane_a,plane_b,plane_c,disparity_grid,grid_dims,
                        I1_desc,I2_desc,P,plane_radius,valid,right_image,D);
            }
        }
      }
    }
    
  }

  delete[] P;
}

void Elas::leftRightConsistencyCheck(float* D1,float* D2) {
  
  // 获取视差图尺寸
  int32_t D_width  = width;
  int32_t D_height = height;
  if (param.subsampling) {
    D_width  = width/2;
    D_height = height/2;
  }
  
  // 复制左右视差图，供一致性检查使用
  float* D1_copy = (float*)malloc(D_width*D_height*sizeof(float));
  float* D2_copy = (float*)malloc(D_width*D_height*sizeof(float));
  memcpy(D1_copy,D1,D_width*D_height*sizeof(float));
  memcpy(D2_copy,D2,D_width*D_height*sizeof(float));

  // 循环变量
  uint32_t addr,addr_warp;
  float    u_warp_1,u_warp_2,d1,d2;
  
  // 遍历所有像素
  for (int32_t u=0; u<D_width; u++) {
    for (int32_t v=0; v<D_height; v++) {
      
      // 计算 (u,v) 的地址，并读取对应的左右视差值
      addr     = getAddressOffsetImage(u,v,D_width);
      d1       = *(D1_copy+addr);
      d2       = *(D2_copy+addr);
      if (param.subsampling) {
        u_warp_1 = (float)u-d1/2;
        u_warp_2 = (float)u+d2/2;
      } else {
        u_warp_1 = (float)u-d1;
        u_warp_2 = (float)u+d2;
      }
      
      
      // 检查左视差是否有效
      if (d1>=0 && u_warp_1>=0 && u_warp_1<D_width) {       
                  
        // 计算右视差图中对应的投影位置地址
        addr_warp = getAddressOffsetImage((int32_t)u_warp_1,v,D_width);

        // 若不满足左右一致性阈值，则将左视差置为无效
        if (fabs(*(D2_copy+addr_warp)-d1)>param.lr_threshold)
          *(D1+addr) = -10;
        
      // 左视差无效
      } else
        *(D1+addr) = -10;
      
      // 检查右视差是否有效
      if (d2>=0 && u_warp_2>=0 && u_warp_2<D_width) {       

        // 计算左视差图中对应的投影位置地址
        addr_warp = getAddressOffsetImage((int32_t)u_warp_2,v,D_width);

        // 若不满足左右一致性阈值，则将右视差置为无效
        if (fabs(*(D1_copy+addr_warp)-d2)>param.lr_threshold)
          *(D2+addr) = -10;
        
      // 右视差无效
      } else
        *(D2+addr) = -10;
    }
  }
  
  // 释放拷贝的视差图
  free(D1_copy);
  free(D2_copy);
}

void Elas::removeSmallSegments (float* D) {
  
  // 获取视差图尺寸
  int32_t D_width        = width;
  int32_t D_height       = height;
  int32_t D_speckle_size = param.speckle_size;
  if (param.subsampling) {
    D_width        = width/2;
    D_height       = height/2;
    D_speckle_size = sqrt((float)param.speckle_size)*2;
  }
  
  // 在堆上为动态规划相关数组分配内存
  int32_t *D_done     = (int32_t*)calloc(D_width*D_height,sizeof(int32_t));
  int32_t *seg_list_u = (int32_t*)calloc(D_width*D_height,sizeof(int32_t));
  int32_t *seg_list_v = (int32_t*)calloc(D_width*D_height,sizeof(int32_t));
  int32_t seg_list_count;
  int32_t seg_list_curr;
  int32_t u_neighbor[4];
  int32_t v_neighbor[4];
  int32_t u_seg_curr;
  int32_t v_seg_curr;
  
  // 循环中的一些辅助变量
  int32_t addr_start, addr_curr, addr_neighbor;
  
  // 遍历所有像素
  for (int32_t u=0; u<D_width; u++) {
    for (int32_t v=0; v<D_height; v++) {
      
      // 获取当前像素（作为片段起点）的地址
      addr_start = getAddressOffsetImage(u,v,D_width);
                  
      // 如果该像素尚未被处理
      if (*(D_done+addr_start)==0) {
                
        // 初始化片段列表（先加入第一个像素，
        // 并将其设置为下一个待检查的元素）
        *(seg_list_u+0) = u;
        *(seg_list_v+0) = v;
        seg_list_count  = 1;
        seg_list_curr   = 0;
        
        // 只要片段列表中仍有“未处理”的像素（即 seg_list_curr<seg_list_count），
        // 就不断向其中加入与当前像素相邻且相似的像素
        while (seg_list_curr<seg_list_count) {
        
          // 从片段列表中取出当前像素坐标
          u_seg_curr = *(seg_list_u+seg_list_curr);
          v_seg_curr = *(seg_list_v+seg_list_curr);
          
          // 获取当前像素在本片段中的地址
          addr_curr = getAddressOffsetImage(u_seg_curr,v_seg_curr,D_width);
          
          // 填充 4-邻域像素的坐标
          u_neighbor[0] = u_seg_curr-1; v_neighbor[0] = v_seg_curr;
          u_neighbor[1] = u_seg_curr+1; v_neighbor[1] = v_seg_curr;
          u_neighbor[2] = u_seg_curr;   v_neighbor[2] = v_seg_curr-1;
          u_neighbor[3] = u_seg_curr;   v_neighbor[3] = v_seg_curr+1;
          
          // 遍历所有邻居
          for (int32_t i=0; i<4; i++) {
            
            // 检查邻居是否在图像范围内
            if (u_neighbor[i]>=0 && v_neighbor[i]>=0 && u_neighbor[i]<D_width && v_neighbor[i]<D_height) {
              
              // 获取邻居像素地址
              addr_neighbor = getAddressOffsetImage(u_neighbor[i],v_neighbor[i],D_width);
              
              // 检查该邻居是否尚未加入列表且视差为有效值
              if (*(D_done+addr_neighbor)==0 && *(D+addr_neighbor)>=0) {

                // 判断该邻居像素与当前像素的视差是否足够接近
                // （即是否属于同一小片段）
                if (fabs(*(D+addr_curr)-*(D+addr_neighbor))<=param.speckle_sim_threshold) {
                  
                  // 将邻居坐标加入片段列表
                  *(seg_list_u+seg_list_count) = u_neighbor[i];
                  *(seg_list_v+seg_list_count) = v_neighbor[i];
                  seg_list_count++;            
                  
                  // 将该邻居标记为已处理，
                  // 否则可能会作为多个像素的邻居被重复加入列表
                  *(D_done+addr_neighbor) = 1;
                }
              }
              
            } 
          }
          
          // 当前列表元素处理完毕，移动到下一个
          seg_list_curr++;
          
          // 将当前像素在 D_done 中标记为已处理
          *(D_done+addr_curr) = 1;

        } // 结束：while (seg_list_curr<seg_list_count)
        
        // 如果该片段的尺寸太小，则视为噪声，全部置为无效
        if (seg_list_count<D_speckle_size) {
          
          // 将当前片段中的所有像素都置为无效视差
          for (int32_t i=0; i<seg_list_count; i++) {
            addr_curr = getAddressOffsetImage(*(seg_list_u+i),*(seg_list_v+i),D_width);
            *(D+addr_curr) = -10;
          }
        }
      } // 结束：if (*(I_done+addr_start)==0)
      
    }
  }
  
  // 释放片段标记相关内存
  free(D_done);
  free(seg_list_u);
  free(seg_list_v);
}

void Elas::gapInterpolation(float* D) {
  
  // 获取视差图尺寸
  int32_t D_width          = width;
  int32_t D_height         = height;
  int32_t D_ipol_gap_width = param.ipol_gap_width;
  if (param.subsampling) {
    D_width          = width/2;
    D_height         = height/2;
    D_ipol_gap_width = param.ipol_gap_width/2+1;
  }
  
  // 判定深度不连续的阈值
  float discon_threshold = 3.0;
  
  // 循环辅助变量
  int32_t count,addr,v_first,v_last,u_first,u_last;
  float   d1,d2,d_ipol;
  
  // 1. 按行处理：
  // 对每一行进行扫描
  for (int32_t v=0; v<D_height; v++) {
    
    // 初始化空洞长度计数器
    count = 0;
    
    // 遍历该行中的每一个像素
    for (int32_t u=0; u<D_width; u++) {
      
      // 获取当前位置的索引
      addr = getAddressOffsetImage(u,v,D_width);
      
      // 若当前视差有效
      if (*(D+addr)>=0) {
        
        // 检查刚刚跨过的空洞是否足够小
        if (count>=1 && count<=D_ipol_gap_width) {
          
          // 插值区间的起始与结束列
          u_first = u-count;
          u_last  = u-1;
          
          // 若插值区间在图像内部
          if (u_first>0 && u_last<D_width-1) {
            
            // 计算两端视差的均值（或取较小者）
            d1 = *(D+getAddressOffsetImage(u_first-1,v,D_width));
            d2 = *(D+getAddressOffsetImage(u_last+1,v,D_width));
            if (fabs(d1-d2)<discon_threshold) d_ipol = (d1+d2)/2;
            else                              d_ipol = min(d1,d2);
            
            // 将空洞内所有像素赋为插值视差
            for (int32_t u_curr=u_first; u_curr<=u_last; u_curr++)
              *(D+getAddressOffsetImage(u_curr,v,D_width)) = d_ipol;
          }
          
        }
        
        // 当前处于有效区域，重置计数
        count = 0;
      
      // 否则说明处于空洞中，递增计数
      } else {
        count++;
      }
    }
    
    // 若需要完整尺寸的视差图
    if (param.add_corners) {

      // 向左外推填补边缘空洞
      for (int32_t u=0; u<D_width; u++) {

        // 获取当前位置索引
        addr = getAddressOffsetImage(u,v,D_width);

        // 找到第一个有效视差像素后，向左填充
        if (*(D+addr)>=0) {
          for (int32_t u2=max(u-D_ipol_gap_width,0); u2<u; u2++)
            *(D+getAddressOffsetImage(u2,v,D_width)) = *(D+addr);
          break;
        }
      }

      // 向右外推填补边缘空洞
      for (int32_t u=D_width-1; u>=0; u--) {

        // 获取当前位置索引
        addr = getAddressOffsetImage(u,v,D_width);

        // 找到第一个有效视差像素后，向右填充
        if (*(D+addr)>=0) {
          for (int32_t u2=u; u2<=min(u+D_ipol_gap_width,D_width-1); u2++)
            *(D+getAddressOffsetImage(u2,v,D_width)) = *(D+addr);
          break;
        }
      }
    }
  }

  // 2. Column-wise:      
  // 对每一列进行处理（垂直方向插值填补空洞）
  for (int32_t u=0; u<D_width; u++) {
    
    // 初始化空洞长度计数器
    count = 0;
    
    // 遍历该列中的每一个像素
    for (int32_t v=0; v<D_height; v++) {
      
      // 获取当前位置的索引
      addr = getAddressOffsetImage(u,v,D_width);
      
      // 如果视差有效
      if (*(D+addr)>=0) {
        
        // 检查空洞是否在允许范围内
        if (count>=1 && count<=D_ipol_gap_width) {
          
          // 确定插值的首尾位置
          v_first = v-count;
          v_last  = v-1;
          
          // 检查插值范围是否有效
          if (v_first>0 && v_last<D_height-1) {
            
            // 计算平均视差
            d1 = *(D+getAddressOffsetImage(u,v_first-1,D_width));
            d2 = *(D+getAddressOffsetImage(u,v_last+1,D_width));
            if (fabs(d1-d2)<discon_threshold) d_ipol = (d1+d2)/2;
            else                              d_ipol = min(d1,d2);
            
            // 将所有插值位置设为计算出的视差值
            for (int32_t v_curr=v_first; v_curr<=v_last; v_curr++)
              *(D+getAddressOffsetImage(u,v_curr,D_width)) = d_ipol;
          }
          
        }
        
        // 重置计数器
        count = 0;
      
      // 否则增加计数器
      } else {
        count++;
      }
    }

    // 向上下外推以填补边缘空洞（因为底部行有时保持未标记）
    // DS 5/12/2014

    // 若需要完整尺寸的视差图
    if (param.add_corners) {

      // 向上外推
      for (int32_t v=0; v<D_height; v++) {

        // 获取当前位置索引
        addr = getAddressOffsetImage(u,v,D_width);

        // 找到第一个有效视差像素后，向上填充
        if (*(D+addr)>=0) {
          for (int32_t v2=max(v-D_ipol_gap_width,0); v2<v; v2++)
            *(D+getAddressOffsetImage(u,v2,D_width)) = *(D+addr);
          break;
        }
      }

      // 向下外推
      for (int32_t v=D_height-1; v>=0; v--) {

        // 获取当前位置索引
        addr = getAddressOffsetImage(u,v,D_width);

        // 找到第一个有效视差像素后，向下填充
        if (*(D+addr)>=0) {
          for (int32_t v2=v; v2<=min(v+D_ipol_gap_width,D_height-1); v2++)
            *(D+getAddressOffsetImage(u,v2,D_width)) = *(D+addr);
          break;
        }
      }
    }
  }
}

// 该函数实现了对双边滤波的一种近似
void Elas::adaptiveMean (float* D) {
  
  // 获取视差图尺寸
  int32_t D_width          = width;
  int32_t D_height         = height;
  if (param.subsampling) {
    D_width          = width/2;
    D_height         = height/2;
  }
  
  // 分配临时内存
  float* D_copy = (float*)malloc(D_width*D_height*sizeof(float));
  float* D_tmp  = (float*)malloc(D_width*D_height*sizeof(float));
  memcpy(D_copy,D,D_width*D_height*sizeof(float));
  
  // 将输入视差图中无效的位置置为 -10，使这些区域在双边滤波中权重为 0
  for (int32_t i=0; i<D_width*D_height; i++) {
    if (*(D+i)<0) {
      *(D_copy+i) = -10;
      *(D_tmp+i)  = -10;
    }
  }
  
  __m128 xconst0 = _mm_set1_ps(0);
  __m128 xconst4 = _mm_set1_ps(4);
  __m128 xval,xweight1,xweight2,xfactor1,xfactor2;
  
  float *val     = (float *)_mm_malloc(8*sizeof(float),16);
  float *weight  = (float*)_mm_malloc(4*sizeof(float),16);
  float *factor  = (float*)_mm_malloc(4*sizeof(float),16);
  
  // 绝对值掩码（用于快速取绝对值）
  __m128 xabsmask = _mm_set1_ps(0x7FFFFFFF);
  
  // 子采样模式下：使用 4 像素宽度的双边滤波
  if (param.subsampling) {
  
    // 水平滤波
    for (int32_t v=3; v<D_height-3; v++) {

      // 初始化首几个元素
      for (int32_t u=0; u<3; u++)
        val[u] = *(D_copy+v*D_width+u);

      // 主循环
      for (int32_t u=3; u<D_width; u++) {

        // 更新当前窗口内的值
        float val_curr = *(D_copy+v*D_width+(u-1));
        val[u%4] = *(D_copy+v*D_width+u);

        xval     = _mm_load_ps(val);      
        xweight1 = _mm_sub_ps(xval,_mm_set1_ps(val_curr));
        xweight1 = _mm_and_ps(xweight1,xabsmask);
        xweight1 = _mm_sub_ps(xconst4,xweight1);
        xweight1 = _mm_max_ps(xconst0,xweight1);
        xfactor1 = _mm_mul_ps(xval,xweight1);

        _mm_store_ps(weight,xweight1);
        _mm_store_ps(factor,xfactor1);

        float weight_sum = weight[0]+weight[1]+weight[2]+weight[3];
        float factor_sum = factor[0]+factor[1]+factor[2]+factor[3];
        
        if (weight_sum>0) {
          float d = factor_sum/weight_sum;
          if (d>=0) *(D_tmp+v*D_width+(u-1)) = d;
        }
      }
    }

    // 垂直滤波
    for (int32_t u=3; u<D_width-3; u++) {

      // 初始化首几个元素
      for (int32_t v=0; v<3; v++)
        val[v] = *(D_tmp+v*D_width+u);

      // 主循环
      for (int32_t v=3; v<D_height; v++) {

        // 更新当前窗口内的值
        float val_curr = *(D_tmp+(v-1)*D_width+u);
        val[v%4] = *(D_tmp+v*D_width+u);

        xval     = _mm_load_ps(val);      
        xweight1 = _mm_sub_ps(xval,_mm_set1_ps(val_curr));
        xweight1 = _mm_and_ps(xweight1,xabsmask);
        xweight1 = _mm_sub_ps(xconst4,xweight1);
        xweight1 = _mm_max_ps(xconst0,xweight1);
        xfactor1 = _mm_mul_ps(xval,xweight1);

        _mm_store_ps(weight,xweight1);
        _mm_store_ps(factor,xfactor1);

        float weight_sum = weight[0]+weight[1]+weight[2]+weight[3];
        float factor_sum = factor[0]+factor[1]+factor[2]+factor[3];
        
        if (weight_sum>0) {
          float d = factor_sum/weight_sum;
          if (d>=0) *(D+(v-1)*D_width+u) = d;
        }
      }
    }
    
  // 全分辨率模式：使用 8 像素宽度的双边滤波
  } else {
    
  
    // 水平滤波
    for (int32_t v=3; v<D_height-3; v++) {

      // 初始化首几个元素
      for (int32_t u=0; u<7; u++)
        val[u] = *(D_copy+v*D_width+u);

      // 主循环
      for (int32_t u=7; u<D_width; u++) {

        // 更新当前窗口内的值
        float val_curr = *(D_copy+v*D_width+(u-3));
        val[u%8] = *(D_copy+v*D_width+u);

        xval     = _mm_load_ps(val);      
        xweight1 = _mm_sub_ps(xval,_mm_set1_ps(val_curr));
        xweight1 = _mm_and_ps(xweight1,xabsmask);
        xweight1 = _mm_sub_ps(xconst4,xweight1);
        xweight1 = _mm_max_ps(xconst0,xweight1);
        xfactor1 = _mm_mul_ps(xval,xweight1);

        xval     = _mm_load_ps(val+4);      
        xweight2 = _mm_sub_ps(xval,_mm_set1_ps(val_curr));
        xweight2 = _mm_and_ps(xweight2,xabsmask);
        xweight2 = _mm_sub_ps(xconst4,xweight2);
        xweight2 = _mm_max_ps(xconst0,xweight2);
        xfactor2 = _mm_mul_ps(xval,xweight2);

        xweight1 = _mm_add_ps(xweight1,xweight2);
        xfactor1 = _mm_add_ps(xfactor1,xfactor2);

        _mm_store_ps(weight,xweight1);
        _mm_store_ps(factor,xfactor1);

        float weight_sum = weight[0]+weight[1]+weight[2]+weight[3];
        float factor_sum = factor[0]+factor[1]+factor[2]+factor[3];
        
        if (weight_sum>0) {
          float d = factor_sum/weight_sum;
          if (d>=0) *(D_tmp+v*D_width+(u-3)) = d;
        }
      }
    }
  
    // 垂直滤波
    for (int32_t u=3; u<D_width-3; u++) {

      // 初始化首几个元素
      for (int32_t v=0; v<7; v++)
        val[v] = *(D_tmp+v*D_width+u);

      // 主循环
      for (int32_t v=7; v<D_height; v++) {

        // 更新当前窗口内的值
        float val_curr = *(D_tmp+(v-3)*D_width+u);
        val[v%8] = *(D_tmp+v*D_width+u);

        xval     = _mm_load_ps(val);      
        xweight1 = _mm_sub_ps(xval,_mm_set1_ps(val_curr));
        xweight1 = _mm_and_ps(xweight1,xabsmask);
        xweight1 = _mm_sub_ps(xconst4,xweight1);
        xweight1 = _mm_max_ps(xconst0,xweight1);
        xfactor1 = _mm_mul_ps(xval,xweight1);

        xval     = _mm_load_ps(val+4);      
        xweight2 = _mm_sub_ps(xval,_mm_set1_ps(val_curr));
        xweight2 = _mm_and_ps(xweight2,xabsmask);
        xweight2 = _mm_sub_ps(xconst4,xweight2);
        xweight2 = _mm_max_ps(xconst0,xweight2);
        xfactor2 = _mm_mul_ps(xval,xweight2);

        xweight1 = _mm_add_ps(xweight1,xweight2);
        xfactor1 = _mm_add_ps(xfactor1,xfactor2);

        _mm_store_ps(weight,xweight1);
        _mm_store_ps(factor,xfactor1);

        float weight_sum = weight[0]+weight[1]+weight[2]+weight[3];
        float factor_sum = factor[0]+factor[1]+factor[2]+factor[3];
        
        if (weight_sum>0) {
          float d = factor_sum/weight_sum;
          if (d>=0) *(D+(v-3)*D_width+u) = d;
        }
      }
    }
  }
  
  // 释放临时内存
  _mm_free(val);
  _mm_free(weight);
  _mm_free(factor);
  free(D_copy);
  free(D_tmp);
}

void Elas::median (float* D) {
  
  // 获取视差图尺寸
  int32_t D_width          = width;
  int32_t D_height         = height;
  if (param.subsampling) {
    D_width          = width/2;
    D_height         = height/2;
  }

  // 临时缓冲区
  float *D_temp = (float*)calloc(D_width*D_height,sizeof(float));
  
  int32_t window_size = 3;
  
  float *vals = new float[window_size*2+1];
  int32_t i,j;
  float temp;
  
  // 第一步：水平方向中值滤波
  for (int32_t u=window_size; u<D_width-window_size; u++) {
    for (int32_t v=window_size; v<D_height-window_size; v++) {
      if (*(D+getAddressOffsetImage(u,v,D_width))>=0) {    
        j = 0;
        for (int32_t u2=u-window_size; u2<=u+window_size; u2++) {
          temp = *(D+getAddressOffsetImage(u2,v,D_width));
          i = j-1;
          while (i>=0 && *(vals+i)>temp) {
            *(vals+i+1) = *(vals+i);
            i--;
          }
          *(vals+i+1) = temp;
          j++;
        }
        *(D_temp+getAddressOffsetImage(u,v,D_width)) = *(vals+window_size);
      } else {
        *(D_temp+getAddressOffsetImage(u,v,D_width)) = *(D+getAddressOffsetImage(u,v,D_width));
      }
        
    }
  }
  
  // 第二步：垂直方向中值滤波
  for (int32_t u=window_size; u<D_width-window_size; u++) {
    for (int32_t v=window_size; v<D_height-window_size; v++) {
      if (*(D+getAddressOffsetImage(u,v,D_width))>=0) {
        j = 0;
        for (int32_t v2=v-window_size; v2<=v+window_size; v2++) {
          temp = *(D_temp+getAddressOffsetImage(u,v2,D_width));
          i = j-1;
          while (i>=0 && *(vals+i)>temp) {
            *(vals+i+1) = *(vals+i);
            i--;
          }
          *(vals+i+1) = temp;
          j++;
        }
        *(D+getAddressOffsetImage(u,v,D_width)) = *(vals+window_size);
      } else {
        *(D+getAddressOffsetImage(u,v,D_width)) = *(D+getAddressOffsetImage(u,v,D_width));
      }
    }
  }
  
  free(D_temp);
  free(vals);
}
