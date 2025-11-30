/*
版权所有 2011，保留所有权利。
Institute of Measurement and Control Systems
Karlsruhe Institute of Technology, Germany

本文件是 libviso2 的一部分。
Authors: Andreas Geiger

libviso2 是自由软件；你可以根据自由软件基金会发布的 GNU 通用公共许可证
（GNU General Public License）第 2 版，或（由你选择的）任何更高版本的条款
对其进行再发布和/或修改。

发布 libviso2 的目的是希望它能发挥作用，但**不提供任何担保**；甚至不包含
对适销性或特定用途适用性的默示担保。更多细节请参阅 GNU 通用公共许可证。

你应该已经随同 libviso2 一起收到了 GNU 通用公共许可证的副本；
如果没有，请写信至 Free Software Foundation, Inc.,
51 Franklin Street, Fifth Floor, Boston, MA 02110-1301, USA。
*/

#ifndef MATRIX_H
#define MATRIX_H

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <iostream>
#include <vector>
 
// 为了在 Windows / Visual Studio 下顺利编译，这里改为统一包含标准头 <stdint.h>
// 使用其内置的 int8_t / int32_t 等定长整数类型，避免旧代码中自行 typedef 引起重定义
#include <stdint.h>

#define endll endl << endl // 双换行（两个 endl）

typedef double FLOAT;      // 双精度
//typedef float  FLOAT;    // 单精度

class Matrix {

public:

  // 构造 / 析构函数
  Matrix ();                                                  // 初始化为空的 0x0 矩阵
  Matrix (const int32_t m,const int32_t n);                   // 初始化为空的 mxn 矩阵
  Matrix (const int32_t m,const int32_t n,const FLOAT* val_); // 用数组 val 初始化 mxn 矩阵
  Matrix (const Matrix &M);                                   // 通过深拷贝构造
  ~Matrix ();

  // 赋值运算符：拷贝 M 的内容
  Matrix& operator= (const Matrix &M);

  // 将 M 的子矩阵复制到数组 val 中；默认参数表示复制整行/整列/整个矩阵
  void getData(FLOAT* val_,int32_t i1=0,int32_t j1=0,int32_t i2=-1,int32_t j2=-1);

  // 设置或获取当前矩阵的子矩阵
  Matrix getMat(int32_t i1,int32_t j1,int32_t i2=-1,int32_t j2=-1);
  void   setMat(const Matrix &M,const int32_t i,const int32_t j);

  // 将子矩阵全部赋值为某个标量（默认为 0），i2/j2 为 -1 时表示整行/整列/整矩阵
  void setVal(FLOAT s,int32_t i1=0,int32_t j1=0,int32_t i2=-1,int32_t j2=-1);

  // 将对角线（或其中一部分）赋值为某个标量；i2 为 -1 时表示整条对角线
  void setDiag(FLOAT s,int32_t i1=0,int32_t i2=-1);

  // 将矩阵清零
  void zero();
  
  // 按给定索引提取若干列
  Matrix extractCols (std::vector<int> idx);

  // 创建单位矩阵
  static Matrix eye (const int32_t m);
  void          eye ();

  // 用向量 M（nx1 或 1xn）作为对角线元素创建对角矩阵
  static Matrix diag(const Matrix &M);
  
  // 返回一个 m×n 矩阵，其元素按列优先从 M 中取出
  static Matrix reshape(const Matrix &M,int32_t m,int32_t n);

  // 创建 3x3 旋转矩阵（约定参考：http://en.wikipedia.org/wiki/Rotation_matrix）
  static Matrix rotMatX(const FLOAT &angle);
  static Matrix rotMatY(const FLOAT &angle);
  static Matrix rotMatZ(const FLOAT &angle);

  // 简单算术运算
  Matrix  operator+ (const Matrix &M); // 矩阵加法
  Matrix  operator- (const Matrix &M); // 矩阵减法
  Matrix  operator* (const Matrix &M); // 矩阵乘法
  Matrix  operator* (const FLOAT &s);  // 与标量相乘
  Matrix  operator/ (const Matrix &M); // 按元素除以矩阵（或向量）
  Matrix  operator/ (const FLOAT &s);  // 按标量除法
  Matrix  operator- ();                // 取负
  Matrix  operator~ ();                // 转置
  FLOAT   l2norm ();                   // 欧氏范数（向量）/ Frobenius 范数（矩阵）
  FLOAT   mean ();                     // 所有元素的平均值

  // 复杂算术运算
  static Matrix cross (const Matrix &a, const Matrix &b);    // 两个 3×1 向量的叉乘
  static Matrix inv (const Matrix &M);                       // 返回矩阵 M 的逆
  bool   inv ();                                             // 将当前矩阵替换为其逆矩阵
  FLOAT  det ();                                             // 计算矩阵行列式
  bool   solve (const Matrix &M,FLOAT eps=1e-20);            // 解线性系统 M*x = B，*this 与 M 会被消去
  bool   lu(int32_t *idx, FLOAT &d, FLOAT eps=1e-20);        // 将 *this 分解为 LU 形式
  void   svd(Matrix &U,Matrix &W,Matrix &V);                 // 奇异值分解 *this = U*diag(W)*V^T

  // 将矩阵打印到输出流
  friend std::ostream& operator<< (std::ostream& out,const Matrix& M);

  // 直接数据访问
  FLOAT   **val;
  int32_t   m,n;

private:

  void allocateMemory (const int32_t m_,const int32_t n_);
  void releaseMemory ();
  inline FLOAT pythag(FLOAT a,FLOAT b);

};

#endif // MATRIX_H
