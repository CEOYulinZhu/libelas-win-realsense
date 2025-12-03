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

// 演示程序：展示如何使用 libelas。可运行 "./elas -h" 查看帮助

#include <iostream>
#include <algorithm>
#include <cstring>
#include "elas.h"
#include "image.h"

#include <librealsense2/rs.hpp>
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/calib3d.hpp>

using namespace std;

static cv::Mat g_depthForClick;
static double g_depthFps = 0.0;

static void onDepthMouse(int event, int x, int y, int flags, void* userdata) {
  if (event != cv::EVENT_LBUTTONDOWN) return;
  if (g_depthForClick.empty()) return;
  if (x < 0 || y < 0 || x >= g_depthForClick.cols || y >= g_depthForClick.rows) return;
  float d = g_depthForClick.at<float>(y, x);
  if (d > 0.0f)
    cout << "Depth at (" << x << ", " << y << "): " << d << " m, depth FPS: " << g_depthFps << endl;
  else
    cout << "Depth at (" << x << ", " << y << "): invalid, depth FPS: " << g_depthFps << endl;
}

image<uchar>* loadImage(const char* name);

// 计算一对输入图像 file_1、file_2 的视差
void process (const char* file_1,const char* file_2) {

  cout << "Processing: " << file_1 << ", " << file_2 << endl;

  // 读取输入图像（Windows 下通过 WIC 支持 PNG/JPG/PGM；其他平台默认仅 PGM）
  image<uchar>* I1 = loadImage(file_1);
  image<uchar>* I2 = loadImage(file_2);

  // 若加载失败，避免空指针解引用并给出明确错误信息
  if (I1 == nullptr || I2 == nullptr) {
    cout << "ERROR: Failed to load input images. Please check file paths and formats." << endl;
    if (I1) delete I1;
    if (I2) delete I2;
    return;
  }

  // 检查两张图像的尺寸是否一致且有效
  if (I1->width() <= 0 || I1->height() <= 0 ||
      I2->width() <= 0 || I2->height() <= 0 ||
      I1->width() != I2->width() || I1->height() != I2->height()) {
    cout << "ERROR: Images must be of same size, but" << endl;
    cout << "       I1: " << I1->width() << " x " << I1->height()
         << ", I2: " << I2->width() << " x " << I2->height() << endl;
    delete I1;
    delete I2;
    return;
  }

  // 获取图像宽高
  int32_t width  = I1->width();
  int32_t height = I1->height();

  // 为视差图分配内存
  const int32_t dims[3] = { width, height, width }; // bytes per line = width
  float* D1_data = (float*)malloc(width * height * sizeof(float));
  float* D2_data = (float*)malloc(width * height * sizeof(float));
  if (!D1_data || !D2_data) {
    cout << "ERROR: Out of memory." << endl;
    if (D1_data) free(D1_data);
    if (D2_data) free(D2_data);
    delete I1;
    delete I2;
    return;
  }

  // 执行 ELAS 视差计算
  Elas::parameters param;
  param.postprocess_only_left = false;
  Elas elas(param);
  elas.process(I1->data, I2->data, D1_data, D2_data, dims);

  // 寻找最大视差，用于将结果缩放到 [0..255]
  float disp_max = 0.0f;
  for (int32_t i = 0; i < width * height; ++i) {
    if (D1_data[i] > disp_max) disp_max = D1_data[i];
    if (D2_data[i] > disp_max) disp_max = D2_data[i];
  }
  if (disp_max <= 0.0f) disp_max = 1.0f;

  // 将 float 视差拷贝并缩放为 8 位灰度（uchar）
  image<uchar>* D1 = new image<uchar>(width, height);
  image<uchar>* D2 = new image<uchar>(width, height);
  for (int32_t i = 0; i < width * height; ++i) {
    double v1 = 255.0 * D1_data[i] / disp_max;
    double v2 = 255.0 * D2_data[i] / disp_max;
    if (v1 < 0.0) v1 = 0.0; if (v1 > 255.0) v1 = 255.0;
    if (v2 < 0.0) v2 = 0.0; if (v2 > 255.0) v2 = 255.0;
    D1->data[i] = static_cast<uint8_t>(v1);
    D2->data[i] = static_cast<uint8_t>(v2);
  }

  // 生成输出文件名：去掉任意长度扩展名后追加 "_disp.pgm" 和 "_disp.png"
  char output_1[1024];
  char output_2[1024];
  char output_1_png[1024];
  char output_2_png[1024];
  const char* dot1 = strrchr(file_1, '.');
  const char* dot2 = strrchr(file_2, '.');
  size_t len1 = dot1 ? (size_t)(dot1 - file_1) : strlen(file_1);
  size_t len2 = dot2 ? (size_t)(dot2 - file_2) : strlen(file_2);
  if (len1 > sizeof(output_1) - 10) len1 = sizeof(output_1) - 10;
  if (len2 > sizeof(output_2) - 10) len2 = sizeof(output_2) - 10;
  memcpy(output_1, file_1, len1); output_1[len1] = '\0'; strcat(output_1, "_disp.pgm");
  memcpy(output_2, file_2, len2); output_2[len2] = '\0'; strcat(output_2, "_disp.pgm");
  memcpy(output_1_png, file_1, len1); output_1_png[len1] = '\0'; strcat(output_1_png, "_disp.png");
  memcpy(output_2_png, file_2, len2); output_2_png[len2] = '\0'; strcat(output_2_png, "_disp.png");

  savePGM(D1, output_1);
  savePGM(D2, output_2);
  savePNG(D1, output_1_png);
  savePNG(D2, output_2_png);

  // 释放内存
  delete I1;
  delete I2;
  delete D1;
  delete D2;
  free(D1_data);
  free(D2_data);
}

static int process_realsense_live(int width, int height, int fps) {
  cout << "XiaoPang 11301901" << endl;
  rs2::pipeline pipe;
  rs2::config cfg;
  cfg.enable_stream(RS2_STREAM_INFRARED, 1, width, height, RS2_FORMAT_Y8, fps);
  cfg.enable_stream(RS2_STREAM_INFRARED, 2, width, height, RS2_FORMAT_Y8, fps);
  rs2::pipeline_profile profile = pipe.start(cfg);

  auto left_sp  = profile.get_stream(RS2_STREAM_INFRARED, 1).as<rs2::video_stream_profile>();
  auto right_sp = profile.get_stream(RS2_STREAM_INFRARED, 2).as<rs2::video_stream_profile>();
  rs2_intrinsics inL = left_sp.get_intrinsics();
  rs2_intrinsics inR = right_sp.get_intrinsics();
  rs2_extrinsics extr = left_sp.get_extrinsics_to(right_sp);

  cv::Mat K1 = (cv::Mat_<double>(3,3) << inL.fx, 0, inL.ppx, 0, inL.fy, inL.ppy, 0, 0, 1);
  cv::Mat D1 = (cv::Mat_<double>(1,5) << inL.coeffs[0], inL.coeffs[1], inL.coeffs[2], inL.coeffs[3], inL.coeffs[4]);
  cv::Mat K2 = (cv::Mat_<double>(3,3) << inR.fx, 0, inR.ppx, 0, inR.fy, inR.ppy, 0, 0, 1);
  cv::Mat D2 = (cv::Mat_<double>(1,5) << inR.coeffs[0], inR.coeffs[1], inR.coeffs[2], inR.coeffs[3], inR.coeffs[4]);
  cv::Mat R = (cv::Mat_<double>(3,3) <<
    extr.rotation[0], extr.rotation[1], extr.rotation[2],
    extr.rotation[3], extr.rotation[4], extr.rotation[5],
    extr.rotation[6], extr.rotation[7], extr.rotation[8]);
  cv::Mat T = (cv::Mat_<double>(3,1) << extr.translation[0], extr.translation[1], extr.translation[2]);

  cv::Size sz(inL.width, inL.height);
  cv::Mat R1, R2, P1, P2, Q;
  cv::stereoRectify(K1, D1, K2, D2, sz, R, T, R1, R2, P1, P2, Q, cv::CALIB_ZERO_DISPARITY, 0, sz);
  double f_rect = P1.at<double>(0,0);
  double baseline = -P2.at<double>(0,3) / P2.at<double>(0,0);

  cv::Mat map1L, map2L, map1R, map2R;
  cv::initUndistortRectifyMap(K1, D1, R1, P1, sz, CV_16SC2, map1L, map2L);
  cv::initUndistortRectifyMap(K2, D2, R2, P2, sz, CV_16SC2, map1R, map2R);

  cv::namedWindow("Disparity", cv::WINDOW_NORMAL);
  cv::namedWindow("Depth", cv::WINDOW_NORMAL);
  cv::setMouseCallback("Depth", onDepthMouse);

  int32_t dims[3] = { sz.width, sz.height, sz.width };
  std::vector<float> D1v(sz.width * sz.height);
  std::vector<float> D2v(sz.width * sz.height);
  Elas::parameters param(Elas::MIDDLEBURY);
  param.postprocess_only_left = false;
  param.ipol_gap_width        = 10;
  param.add_corners           = 0;
  Elas elas(param);
  cv::Mat prevDepth;
  float prevDispMax = 0.0f;

  // [Flicker Solution] 1. Use shared CLAHE instead of independent equalizeHist
  // to ensure consistent photometric mapping and suppress noise in low-texture areas.
  cv::Ptr<cv::CLAHE> clahe = cv::createCLAHE(2.0, cv::Size(8, 8));

  while (true) {
    rs2::frameset frames = pipe.wait_for_frames();
    rs2::video_frame fL = frames.get_infrared_frame(1);
    rs2::video_frame fR = frames.get_infrared_frame(2);

    cv::Mat rawL(sz, CV_8UC1, (void*)fL.get_data());
    cv::Mat rawR(sz, CV_8UC1, (void*)fR.get_data());
    cv::Mat rectL, rectR;
    cv::remap(rawL, rectL, map1L, map2L, cv::INTER_LINEAR);
    cv::remap(rawR, rectR, map1R, map2R, cv::INTER_LINEAR);

    cv::Mat procL, procR;
    // [Flicker Solution] Apply shared CLAHE
    clahe->apply(rectL, procL);
    clahe->apply(rectR, procR);

    // Optional: Slight blur to reduce sensor noise further
    cv::GaussianBlur(procL, procL, cv::Size(3,3), 0.0);
    cv::GaussianBlur(procR, procR, cv::Size(3,3), 0.0);

    dims[0] = procL.cols;
    dims[1] = procL.rows;
    dims[2] = (int32_t)procL.step;
    elas.process(procL.data, procR.data, D1v.data(), D2v.data(), dims);

    cv::Mat dispF(sz, CV_32F, D1v.data());
    
    // [Flicker Solution] 3. Bilateral Filter to stabilize disparity edges
    // Filter before median blur to preserve structure better
    cv::Mat dispBilateral;
    cv::bilateralFilter(dispF, dispBilateral, 5, 2.0, 5.0);
    
    cv::Mat dispFiltered;
    cv::medianBlur(dispBilateral, dispFiltered, 5);
    cv::Mat validMask = dispFiltered > 0;

    double minDisp = 0.0, maxDisp = 0.0;
    if (cv::countNonZero(validMask) > 0) {
      cv::minMaxLoc(dispFiltered, &minDisp, &maxDisp, nullptr, nullptr, validMask);
    }

    float disp_max = static_cast<float>(maxDisp);
    if (disp_max <= 0.0f) {
      if (prevDispMax > 0.0f) {
        disp_max = prevDispMax;
      } else {
        disp_max = 1.0f;
      }
    } else {
      if (prevDispMax > 0.0f) {
        float alphaScale = 0.9f;
        disp_max = alphaScale * prevDispMax + (1.0f - alphaScale) * disp_max;
      }
    }
    if (disp_max <= 0.0f) disp_max = 1.0f;
    prevDispMax = disp_max;

    cv::Mat disp8;
    dispFiltered.convertTo(disp8, CV_8U, 255.0f / disp_max);

    cv::Mat depthF = cv::Mat::zeros(sz, CV_32F);
    cv::divide(f_rect * baseline, dispFiltered, depthF, 1.0, CV_32F);
    depthF.setTo(0, ~validMask);

    cv::Mat depthFiltered = depthF.clone();
    cv::Mat tmpDepth = depthF.clone();
    tmpDepth.setTo(0, ~validMask);
    cv::medianBlur(tmpDepth, tmpDepth, 5);
    tmpDepth.copyTo(depthFiltered, validMask);
    depthF = depthFiltered;

    if (!prevDepth.empty()) {
      // [Flicker Solution] 3. Time consistency
      // Reduce alpha to 0.3 (30% new, 70% old) to increase stability
      float alpha = 0.3f; 
      cv::Mat validPrev = prevDepth > 0;
      for (int y = 0; y < depthF.rows; ++y) {
        float* currPtr = depthF.ptr<float>(y);
        float* prevPtr = prevDepth.ptr<float>(y);
        const uchar* currMaskPtr = validMask.ptr<uchar>(y);
        const uchar* prevMaskPtr = validPrev.ptr<uchar>(y);
        for (int x = 0; x < depthF.cols; ++x) {
          bool currValid = currMaskPtr[x] != 0;
          bool prevValid = prevMaskPtr[x] != 0;
          if (currValid && prevValid) {
            currPtr[x] = alpha * currPtr[x] + (1.0f - alpha) * prevPtr[x];
          } else if (!currValid && prevValid) {
            currPtr[x] = prevPtr[x];
          }
        }
      }
    }
    prevDepth = depthF.clone();

    cv::Mat depthVis8;
    double alpha_depth = 30.0;
    depthF.convertTo(depthVis8, CV_8U, alpha_depth);
    cv::Mat depthColor;
    cv::applyColorMap(depthVis8, depthColor, cv::COLORMAP_JET);

    cv::imshow("Disparity", disp8);
    cv::imshow("Depth", depthColor);

    g_depthForClick = depthF.clone();
    double now = (double)cv::getTickCount();
    static double lastTime = now;
    double dt = (now - lastTime) / cv::getTickFrequency();
    if (dt > 0.0)
      g_depthFps = 1.0 / dt;
    lastTime = now;

    int key = cv::waitKey(1);
    if (key == 27 || key == 'q' || key == 'Q') break;
  }

  pipe.stop();
  return 0;
}

int main (int argc, char** argv) {

  // 运行demo
  if (argc==2 && !strcmp(argv[1],"demo")) {
    process("img/cones_left.pgm",   "img/cones_right.pgm");
    process("img/aloe_left.pgm",    "img/aloe_right.pgm");
    process("img/raindeer_left.pgm","img/raindeer_right.pgm");
    process("img/urban1_left.pgm",  "img/urban1_right.pgm");
    process("img/urban2_left.pgm",  "img/urban2_right.pgm");
    process("img/urban3_left.pgm",  "img/urban3_right.pgm");
    process("img/urban4_left.pgm",  "img/urban4_right.pgm");
    cout << "... done!" << endl;

  // 从输入图像对计算视差图
  } else if (argc==3) {
    process(argv[1],argv[2]);
    cout << "... done!" << endl;

  } else if (argc>=2 && !strcmp(argv[1],"realsense")) {
    int w = 640, h = 480, fps = 30;
    if (argc >= 5) { w = atoi(argv[2]); h = atoi(argv[3]); fps = atoi(argv[4]); }
    process_realsense_live(w, h, fps);
    cout << "... done!" << endl;

  // 显示帮助信息
  } else {
    cout << endl;
    cout << "ELAS demo program usage: " << endl;
    cout << "./elas demo ................ process all test images (image dir)" << endl;
    cout << "./elas left right .......... process a single stereo pair" << endl;
    cout << "./elas realsense [w h fps] . run live with D435i (default 640 480 30)" << endl;
    cout << "./elas -h .................. shows this help" << endl;
    cout << endl;
    cout << "Note: Input images are expected to be greylevel images." << endl;
    cout << "      On Windows, PNG/JPG/PGM are supported via WIC; on other" << endl;
    cout << "      platforms, only PGM is supported by default." << endl;
    cout << "      All output disparities will be scaled such that disp_max = 255." << endl;
    cout << endl;
  }

  return 0;
}


