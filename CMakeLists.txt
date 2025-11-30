# project
 # 为兼容较新的 CMake 版本，将最低版本从原来的 2.6 提升到 3.5，
 # 避免新版本 CMake 报 "Compatibility with CMake < 3.5 has been removed" 错误
cmake_minimum_required (VERSION 3.5)
project (libelas)

# directories
set (LIBELAS_SRC_DIR src)

# include directory
include_directories("${LIBELAS_SRC_DIR}")

 # 使用 SSE 指令集：
 #   - 原始工程在 Linux/g++ 下使用 -msse3；
 #   - 在 Visual Studio (MSVC) 下直接使用 -msse3 会报不支持，因此这里区分编译器：
 #     * MSVC: 使用 /arch:SSE2（在 x64 下即使被忽略也不会影响编译）；
 #     * 其他编译器（g++/clang）: 继续使用原来的 -msse3 以启用 SSE3 优化。
if (MSVC)
  add_compile_options(/utf-8)      # Treat source files as UTF-8 to avoid codepage issues
  add_compile_options(/arch:SSE2)  # VS 专用语法，等价于启用 SSE2 指令集
else ()
  add_compile_options(-msse3)      # 保持与原 Linux 工程一致的 SSE3 编译选项
endif ()

# sources
FILE(GLOB LIBELAS_SRC_FILES "src/*.cpp")

# make release version
set(CMAKE_BUILD_TYPE Release)

# build demo program
add_executable(elas ${LIBELAS_SRC_FILES})

find_package(realsense2 REQUIRED)
find_package(OpenCV REQUIRED COMPONENTS core imgproc highgui calib3d)
include_directories(${OpenCV_INCLUDE_DIRS})

target_link_libraries(elas PRIVATE realsense2::realsense2 ${OpenCV_LIBS})

# 在 Windows / MSVC 下，image_io.cpp 使用 Windows Imaging Component (WIC)
# 来加载 PNG/JPG，需要链接 windowscodecs 和 Ole32（CoInitializeEx 等）。
if (MSVC)
  target_link_libraries(elas PRIVATE windowscodecs Ole32)
endif ()

