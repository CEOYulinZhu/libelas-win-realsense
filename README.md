# libelas-win-realsense（Windows & RealSense 增强版）

基于 **libelas (LIBrary for Efficient LArge-scale Stereo Matching)** 的非官方改良版实现，提供：

- **命令行工具**：一行命令即可对双目图像计算视差图  
- **Windows 友好支持**：支持 PNG/JPG/PGM 输入，直接运行 `elas.exe`  
- **RealSense 实时模式**：支持如 D435i 的实时红外双目输入，显示视差与深度  
- **中文注释与文档**：便于阅读、调试和二次开发

---

## 致谢与原项目说明

本仓库基于开源项目 **libelas** 修改而来，核心算法与大部分实现来自：

- **Original Author**: Andreas Geiger  
- **Affiliation**: Institute of Measurement and Control Systems,  
  Karlsruhe Institute of Technology, Germany  
- **Original Project**: `libelas` – LIBrary for Efficient LArge-scale Stereo Matching  

如果你在科研或工程中使用本仓库，请优先致谢并引用原作者论文：

```text
@INPROCEEDINGS{Geiger10,
  author    = {Andreas Geiger and Martin Roser and Raquel Urtasun},
  title     = {Efficient Large-Scale Stereo Matching},
  booktitle = {Asian Conference on Computer Vision},
  year      = {2010},
  month     = {November},
  address   = {Queenstown, New Zealand}
}
```

> 本仓库为 **非官方改良版 / 派生作品**，与原作者及其所在单位无直接关联。  
> 原项目的版权与许可证条款仍然适用（GPLv3+），详见 [License](#license)。

---

## 本仓库的改良与新增内容

本仓库在保持原有 ELAS 核心算法结构不变的前提下，围绕“**易用性 / 兼容性 / 实时演示**”做了较大改造（代码中的中文注释即为改良部分）：

- **命令行体验与错误处理优化（`src/main.cpp`）**  
  - 增加清晰的命令行帮助（`./elas -h`），覆盖 demo、单对图片、RealSense 三种模式  
  - 在输入图像加载失败、尺寸不匹配、内存分配失败等场景下，增加安全检查与明确的错误提示，避免空指针解引用  
  - 自动根据输入文件名生成 `*_disp.pgm` 和 `*_disp.png` 两种输出格式，避免手动拼输出路径

- **图像 I/O 与格式兼容性改良（`image.h` / `image_io.cpp` / `main.cpp`）**  
  - 新增统一的 `loadImage` 接口：
    - **Windows 下**：通过 Windows Imaging Component (WIC) 支持 **PNG / JPG / PGM 等常见格式**，并统一转换为 8bit 灰度，兼容实际项目中常见的 PNG/JPG 输入  
    - 若 WIC 流程失败，会回退到原始的 `loadPGM`，最终仍失败时返回 `nullptr` 并由 `main.cpp` 输出清晰错误信息  
    - 通过 `AnsiToWide` 将窄字符路径转换为 UTF-16，**兼容包含中文/空格的 Windows 路径**  
  - 新增 `savePNG`：
    - **Windows 下** 使用 WIC 将 8bit 灰度视差图保存为 PNG，方便在常见图像查看器中直接查看  
    - **非 Windows 平台** 中，此接口会退化为保存 PGM 并给出 warning，保证在不依赖 WIC 的前提下仍能正常编译与运行  
  - 非 Windows 平台上，`loadImage` 等价于调用原始 `loadPGM`，保持与老版本行为一致

- **编译与平台兼容性增强（`CMakeLists.txt` / `matrix.h` / `filter.cpp` / `timer.h`）**  
  - 将 `cmake_minimum_required` 从旧版本提升到 **3.5**，避免在新版 CMake 下出现兼容性警告/错误  
  - 区分编译器配置 SSE：
    - **MSVC**：使用 `/arch:SSE2` 启用 SSE2，并添加 `/utf-8` 保证源码按 UTF-8 编码解析（避免中文注释/路径在 VS 下乱码）  
    - **g++/clang 等其他编译器**：沿用原工程的 `-msse3` 选项，继续启用 SSE3 优化实现  
  - 解决定长整数类型在 Visual Studio 下的重定义/缺失问题：
    - 在 `matrix.h`、`filter.cpp` 中统一改为直接包含标准头 `<stdint.h>`，使用其内置的 `int8_t` / `int32_t` / `uint8_t` 等类型，避免旧代码重复 `typedef` 导致 MSVC 报“重定义”  
    - 在 `timer.h` 中对 `_MSC_VER` 做条件编译，为 MSVC 明确定义 `int32_t` / `uint32_t` 等类型，保证计时工具在 Windows / Linux 下都能通过编译  
  - 以上改动使得本仓库可以在 **Windows + Visual Studio** 下“开箱即用”，同时对其他支持 SSE 的桌面平台也更友好（如需在 Linux/macOS 下使用，可按需调整 `find_package` 与依赖路径）

- **Windows 平台使用体验与文档**  
  - 提供基于 CMake / Visual Studio 的构建说明与可执行文件 `elas.exe`  
  - 在 `docs/使用手册.md` 中详细说明：
    - Windows 10/11 x64 环境要求  
    - 如何准备 `elas.exe`、DLL 依赖与 `img/` 目录  
    - PowerShell / CMD 下的典型调用示例，以及常见错误（DLL 缺失、路径问题等）的排查思路  

- **RealSense 实时深度演示（`realsense` 子命令，`src/main.cpp`）**  
  - 新增 `./elas realsense [w h fps]` 模式，支持如 D435i 的实时红外双目输入：  
    - 使用 `librealsense2` 获取双目红外流，并通过 OpenCV 根据相机内参/外参做极线校正  
    - 使用 ELAS 实时计算视差，并根据基线和焦距换算为深度图（单位：米）  
    - 使用伪彩色方式显示深度，同时提供灰度视差窗口  
    - 鼠标点击深度窗口任意像素，可在控制台输出该点深度和当前 FPS  
  - 对实时深度结果加入中值滤波与帧间平滑，降低噪声和抖动，提高观感稳定性

- **可视化与深度计算改进**  
  - 仍然输出原始浮点视差图，但在显示/保存时：
    - 将合法视差缩放到 `[0, 255]` 进行 8bit 可视化  
    - 在 RealSense 模式下，对无效视差使用掩码，不再把内部的 `-10` 直接压缩成 0，以免与真实“零视差”混淆  
  - 在实时模式中通过 `Q` 矩阵与基线、焦距计算真实深度（米），并对深度图做基本的插值/滤波处理后再上色

- **中文注释与技术文档**  
  - 在核心代码（尤其是 `main.cpp`、图像 I/O、RealSense 管线相关部分）中加入了细致的中文注释，解释每一步的目的与潜在坑点  
  - 补充多份中文技术文档，系统分析 ELAS 与后续可行的改进方向：
    - `docs/ELAS问题分析.md`：  
      - 总结原版实现中在支撑点不足、遮挡处理、边界插值、可视化等方面的典型问题  
    - `docs/algorithm-analysis.md`：  
      - 从算法、实现、功能、鲁棒性、扩展性等多个维度全面分析 ELAS 的优缺点  
    - `docs/improvement-suggestions.md`：  
      - 针对上述问题提出详细的改进思路和示例代码（多线程、AVX2、亚像素视差、置信度图等），为后续工作预留方向  
    - `docs/使用手册.md`：  
      - 面向实际用户的命令行使用说明与常见问题排查  

> 说明：上述文档中部分改进项目前仍处于 **设计/规划** 阶段，未必全部已经实装到当前代码中。  
> 实际功能请以源代码与 Release 说明为准。

- **本仓库维护者 / 改良者**  
  - 代码中中文注释和上述中文文档由 **麦克阿瑟·小胖** 撰写与维护  
  - 如需讨论实现细节或反馈 bug，欢迎在 Issue 中留言

---

## 功能概览

- **离线双目视差计算**：  
  - 输入：两张已校正、同尺寸的灰度图像（Windows 下 PNG/JPG/PGM 均可）  
  - 输出：  
    - 对每张输入图生成 `*_disp.pgm` 和 `*_disp.png`  
    - 视差范围会自动缩放到 `[0, 255]` 以便可视化  

- **RealSense 实时深度演示**：  
  - 支持 D435i 等设备，使用左右红外流  
  - 自动标定、极线校正、视差计算与深度估计  
  - 支持鼠标点击查看像素深度与当前 FPS  

---

## 快速开始

### 1. 编译 / 获取可执行文件

- 推荐使用 CMake + Visual Studio 在 Windows 下构建  
- 编译成功后，在 `Release/` 目录中可以得到 `elas.exe` 及所需 DLL  
- 详细步骤参考原始 `README.TXT` 以及本仓库的 CMake 配置

### 2. 命令行用法（Windows 示例）

在包含 `elas.exe` 的目录下：

- 运行内置 demo：

```powershell
.\elas.exe demo
```

- 处理单对图像（相对路径）：

```powershell
.\elas.exe img\cones_left.pgm img\cones_right.pgm
```

- 处理任意路径下的两张灰度图：

```powershell
.\elas.exe "C:\\Path To\\left.png" "C:\\Path To\\right.png"
```

- RealSense 实时模式（需已连接设备）：

```powershell
.\elas.exe realsense 640 480 30
```

> 更多参数说明与常见问题，请参考 `docs/使用手册.md`。

---

## 项目结构

- **`src/`**：ELAS 核心代码及演示程序  
- **`img/`**：示例双目图像  
- **`docs/`**：  
  - `ELAS问题分析.md`  
  - `algorithm-analysis.md`  
  - `improvement-suggestions.md`  
  - `使用手册.md`  
- **`README.TXT`**：原项目自带说明（英文）  
- **`README.md`**：GitHub 版本说明（本文件）  

---

## License

- 原项目 **libelas** 以 **GNU General Public License v3 (GPLv3) 或更高版本** 发布。  
- 本仓库为基于 libelas 的派生作品，同样遵循 GPLv3 及其要求。  

这意味着（但不限于）：

- 如果你将本库集成到自己的软件中并进行发布，需要 **开源你的整体项目**（遵循 GPLv3）；  
- 如需在闭源商业项目中使用 ELAS，请联系原作者获取商业授权；  
- 分发本仓库的修改版本时，请保留原作者的版权声明与许可证文本，并清晰标注你的改动。
