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

// 基本的图像读写接口，基于 Pedro Felzenszwalb 的代码实现

#ifndef IMAGE_H
#define IMAGE_H

#include <cstdlib>
#include <climits>
#include <cstring>
#include <fstream>
#include <iostream>

// 使用 imRef 访问图像数据
#define imRef(im, x, y) (im->access[y][x])
  
// 使用 imPtr 获取指向图像数据的指针
#define imPtr(im, x, y) &(im->access[y][x])

#define BUF_SIZE 256

typedef unsigned char uchar;
typedef struct { uchar r, g, b; } rgb;

inline bool operator==(const rgb &a, const rgb &b) {
  return ((a.r == b.r) && (a.g == b.g) && (a.b == b.b));
}

// 图像类定义
template <class T> class image {
public:

  // 创建图像
  image(const int width, const int height, const bool init = false);

  // 删除图像
  ~image();

  // 用给定值初始化整幅图像
  void init(const T &val);

  // 深拷贝一份图像
  image<T> *copy() const;
  
  // 获取图像宽度 / 高度
  int width() const { return w; }
  int height() const { return h; }
  
  // 图像数据缓冲区
  T *data;
  
  // 每一行的指针数组
  T **access;
  
private:
  int w, h;
};

template <class T> image<T>::image(const int width, const int height, const bool init) {
  w = width;
  h = height;
  data = new T[w * h];  // 为图像数据分配空间
  access = new T*[h];   // 为行指针数组分配空间
  
  // 初始化每一行的指针
  for (int i = 0; i < h; i++)
    access[i] = data + (i * w);  
  
  // 若需要，则初始化为 0
  if (init)
    memset(data, 0, w * h * sizeof(T));
}

template <class T> image<T>::~image() {
  delete [] data; 
  delete [] access;
}

template <class T> void image<T>::init(const T &val) {
  T *ptr = imPtr(this, 0, 0);
  T *end = imPtr(this, w-1, h-1);
  while (ptr <= end)
    *ptr++ = val;
}


template <class T> image<T> *image<T>::copy() const {
  image<T> *im = new image<T>(w, h, false);
  memcpy(im->data, data, w * h * sizeof(T));
  return im;
}

class pnm_error {};

inline void pnm_read(std::ifstream &file, char *buf) {
  char doc[BUF_SIZE];
  char c;
  
  file >> c;
  while (c == '#') {
    file.getline(doc, BUF_SIZE);
    file >> c;
  }
  file.putback(c);
  
  file.width(BUF_SIZE);
  file >> buf;
  file.ignore();
}

inline image<uchar> *loadPGM(const char *name) {
  char buf[BUF_SIZE];
  
  // 读取 PNM 头部
  std::ifstream file(name, std::ios::in | std::ios::binary);
  pnm_read(file, buf);
  if (strncmp(buf, "P5", 2)) {
    std::cout << "ERROR: Could not read file " << name << std::endl;
    throw pnm_error();
  }

  pnm_read(file, buf);
  int width = atoi(buf);
  pnm_read(file, buf);
  int height = atoi(buf);

  pnm_read(file, buf);
  if (atoi(buf) > UCHAR_MAX) {
    std::cout << "ERROR: Could not read file " << name << std::endl;
    throw pnm_error();
  }

  // 读取图像数据
  image<uchar> *im = new image<uchar>(width, height);
  file.read((char *)imPtr(im, 0, 0), width * height * sizeof(uchar));

  return im;
}

inline void savePGM(image<uchar> *im, const char *name) {
  int width = im->width();
  int height = im->height();
  std::ofstream file(name, std::ios::out | std::ios::binary);

  file << "P5\n" << width << " " << height << "\n" << UCHAR_MAX << "\n";
  file.write((char *)imPtr(im, 0, 0), width * height * sizeof(uchar));
}

// 新增通用图像加载接口：
// 在 Windows 下支持 PNG/JPG 等格式（通过 image_io.cpp 中的 WIC 实现），
// 在其他平台上等价于调用 loadPGM。
image<uchar>* loadImage(const char* name);

// 以 PNG 格式保存 8 位灰度图（Windows 下通过 WIC 实现）
void savePNG(image<uchar>* im, const char* name);

#endif
