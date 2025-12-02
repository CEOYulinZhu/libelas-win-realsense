# src 代码分析与实时深度差异

## 模块概览
- `src/main.cpp:54-324`：提供静态图片 `process`、实时 `process_realsense_live` 以及命令行入口，串联了图像读写、ELAS 调用和可视化逻辑。
- `src/elas.cpp:31-150` 与 `src/elas.h:77-139`：核心 ELAS 实现，包含输入对齐、梯度描述子生成、支持点匹配、Delaunay 三角剖分、平面拟合、稠密视差、左右一致性、孔洞填充和可选滤波等步骤。
- `src/descriptor.cpp:27-90` / `src/descriptor.h:24-60`：用 `filter::sobel3x3` 计算梯度，并将 16 维块描述子存入 `_mm_malloc` 对齐缓冲，为后续 `computeSupportMatches` 提供特征。
- `src/filter.cpp:13-210` / `src/filter.h:17-77`：实现 3x3/5x5 Sobel、checkerboard、blob 等滤波器，全部基于 SSE 指令优化，保证描述子和支持点检测的速度。
- `src/image.h:1-120` 与 `src/image_io.cpp:42-223`：轻量图像容器及跨平台读写，Windows 下借助 WIC 自动转换 PNG/JPG 为 8bit 灰度，静态模式直接依赖该实现。
- `src/matrix.h:1-140` 及 `src/matrix.cpp`：封装小规模矩阵运算、LU/SVD 求解，被 `computeDisparityPlanes` 等步骤用于平面参数拟合。
- `src/triangle.h:1-230` 及 `src/triangle.cpp`：改写的 Triangle 库，负责在支持点上构建 Delaunay 三角剖分，为平面假设提供拓扑结构。
- `src/timer.h:1-78`：可选的 PROFILE 定义下用于分阶段计时。

## 静态图片处理流程
1. `process` 通过 `loadImage` 读取左右图（`src/main.cpp:60-67`），底层在 Windows 上使用 WIC 解码并转为 8bit 灰度（`src/image_io.cpp:42-150`），保证 photometric 一致性。
2. 校验尺寸并准备 `dims`、视差缓冲区（`src/main.cpp:70-103`）。
3. 调用 `Elas::process`（`src/main.cpp:99-103`）。该函数首先把输入按 16 字节对齐复制到内部缓冲（`src/elas.cpp:31-50`），随后构建描述子（`src/elas.cpp:53-58`）、寻找支持点并执行双向一致性确认（`src/elas.cpp:62-104` 与 `src/elas.cpp:303`），用 Triangle 做 Delaunay 剖分并生成视差平面，然后在每个像素上搜索视差、做左右一致性检查（`src/elas.cpp:905-974`）、小区域移除（`src/elas.cpp:977-1046`）、孔洞插值（`src/elas.cpp:1095-1188`）以及可选的自适应均值/中值滤波。
4. 将 `float` 视差缩放到 `uchar` 并以 PGM/PNG 保存（`src/main.cpp:105-149`）。由于静态流程一次性运行，所有昂贵步骤都只执行一遍。

## 实时 Realsense 流程
1. `process_realsense_live` 打开双红外流并生成相机内参、外参（`src/main.cpp:155-178`），随后通过 `cv::stereoRectify` 与 `initUndistortRectifyMap` 得到重映射表与基线/焦距（`src/main.cpp:179-187`）。
2. 预先分配视差缓冲并实例化 `Elas`，沿用 Robotics 参数但强制 `postprocess_only_left=false`、`ipol_gap_width=10`（`src/main.cpp:193-201`）。
3. 主循环中，采集帧、利用 `cv::remap` 做几何矫正（`src/main.cpp:208-218`），再对两路图像执行 CLAHE 与高斯模糊（`src/main.cpp:219-227`），然后将 `dims` 设为 Mat 实际步长并调用 `elas.process`（`src/main.cpp:228-233`）。
4. 视差结果先经双边滤波、5x5 中值滤波去噪（`src/main.cpp:235-242`），再基于有效 mask 动态调整拉伸范围（`src/main.cpp:244-266`）。
5. 深度通过 `depth = f_rect * baseline / disparity` 计算，并使用额外的中值滤波与跨帧指数平滑（`src/main.cpp:268-299`），以减弱闪烁但会引入滞后。
6. 最终把视差/深度着色后显示，并统计帧率（`src/main.cpp:302-317`）。

## 静态与实时效果差异分析
1. **成像差异与纹理不足**：静态测试对的是官方 PGM，亮度一致且纹理丰富；实时红外图噪声大、时刻在调整曝光，即便增加共享 CLAHE 及 3x3 高斯模糊（`src/main.cpp:221-227`），仍很难满足 `support_texture` 阈值，导致 `computeMatchingDisparity` 直接放弃大量支持点（`src/elas.cpp:303`）。支持点稀疏会让 Delaunay 平面误差增大、视差图出现大片空洞。
2. **视差搜索成本**：Robotics 预设默认 `disp_min=0`、`disp_max=255`、`candidate_stepsize=5`（`src/elas.h:85-107`）。在静态图片中只运行一次还能接受，但实时模式每帧都在 256 个视差级别里搜索，每个像素都需要 4 个 4×4 描述子与 SAD 运算，CPU 负载飙升，自然拖垮帧率。
3. **重建流水线叠加滤波**：实时流程在 ELAS 之前和之后叠加了 CLAHE、双边滤波、中值滤波、跨帧时间滤波等多个 O(N) 操作（`src/main.cpp:206-299`）。这些操作虽然缓解闪烁，却会进一步平滑掉细节，使本就弱纹理的红外图更难匹配，同时增加至少数十毫秒延迟。
4. **视差→深度阶段精度**：`cv::divide` 直接对噪声视差求倒数，再以固定 `alpha_depth=30` 缩放展示（`src/main.cpp:268-307`），没有根据量程自适应 nor 置信度评估。噪声越大，深度越发散，即使之后用中值与时间滤波拉平，也只是掩盖问题。
5. **重复计算右视差**：实时只展示左视差/深度，但仍然在参数里关闭 `postprocess_only_left`（`src/main.cpp:197`），因此 `Elas::process` 中的 `removeSmallSegments`、`gapInterpolation`、`adaptiveMean`、`median` 等都会对 D2 再运行一遍（`src/elas.cpp:107-142`），平白损失 30~40% 的时间预算。
6. **每帧重新构建支持结构**：ELAS 设计上会在每次 `process` 调用时重新分配描述子、支持点、Delaunay 网格和代价体（`src/elas.cpp:31-150`，`src/triangle.h:1-230`）。静态模式运行一次无妨，实时模式却无法复用前一帧的结构，导致大量 malloc/free 与 CPU 瓶颈。

## 可考虑的优化方向
1. **针对实时相机调整参数**：限制 `disp_max` 到相机实际基线允许的视差范围，增加 `candidate_stepsize` 或开启 `subsampling`（`src/elas.h:85-112`），并降低 `support_texture`、`support_threshold` 以适配红外纹理，可显著减少搜索量并提升有效支持点密度。
2. **只计算左视差**：实时渲染只需要左视差/深度，建议把 `param.postprocess_only_left` 设为 `true`（参考当前 `src/main.cpp:197` 的配置）以免重复耗时。
3. **轻量化预/后处理**：可以用 `rs2::option::exposure` 固定曝光、在 GPU 上实现共用的 CLAHE 或改为简单的引导滤波，并把双边滤波替换为更轻量的跨尺度加权，以减少 `src/main.cpp:206-277` 中多次逐像素遍历的代价。
4. **利用硬件/增量信息**：RealSense SDK 已提供对齐后的深度与左右红外，若仍需自算，可复用上一帧的支持点、或在 `Elas` 中启用 `subsampling`＋`grid_size` 调整，甚至用线程池分摊 `triangle` 与 `computeDisparity` 阶段，这些都能缓解 `src/elas.cpp:31-150` 里逐帧重建带来的 CPU 峰值。
