# GS-LIVO Rewrite: 构建系统设计

## 1. 总体要求

| 要求 | 说明 |
|------|------|
| CMake 最低版本 | 3.24+ (使用 FILE_SET, RUNTIME_DEPENDENCIES 等新特性) |
| C++ 标准 | C++20 |
| CUDA 版本 | 11.8+ (与 LibTorch 匹配) |
| 目标平台 | x86 Linux (Ubuntu 22.04+) |
| 构建工具 | CMake + Make/Ninja |
| 包管理器 | vcpkg (可选) / 系统包 + FetchContent |
| 编译器 | GCC 11+ / Clang 16+ |
| NVIDIA GPU | Compute Capability 7.0+ (Turing/Ampere/Ada) |

## 2. 目录结构

```
gs-livo/
|-- CMakeLists.txt                  # 顶层 CMake
|-- cmake/
|   |-- CompilerOptions.cmake       # 编译器选项 (C++20, warnings)
|   |-- CudaOptions.cmake           # CUDA 架构 + 编译选项
|   |-- Dependencies.cmake          # 依赖管理 (FetchContent / find_package)
|   |-- InstallRules.cmake          # 安装规则
|   +-- Sanitizers.cmake            # ASAN / UBSan / TSAN 支持
|-- src/
|   |-- CMakeLists.txt              # 所有库目标的工厂
|   |-- core/                       # 基础类型、工具、ConcurrentQueue
|   |   |-- CMakeLists.txt
|   |   |-- types.h
|   |   |-- config.h
|   |   |-- config.cpp
|   |   +-- concurrent_queue.h
|   |-- interfaces/                 # 抽象接口 (纯虚类)
|   |   +-- CMakeLists.txt
|   |-- ekf/                        # EkfStateManager
|   |   |-- CMakeLists.txt
|   |   |-- ekf_state_manager.h
|   |   +-- ekf_state_manager.cpp
|   |-- sensor/                     # 传感器驱动
|   |   |-- CMakeLists.txt
|   |   |-- lidar_driver.h / .cpp
|   |   |-- camera_driver.h / .cpp
|   |   +-- sensor_synchronizer.h / .cpp
|   |-- imu/                        # IMU 预积分
|   |   |-- CMakeLists.txt
|   |   |-- imu_preintegrator.h / .cpp
|   |   +-- imu_propagator.h / .cpp
|   |-- lio/                        # LiDAR 惯性里程计
|   |   |-- CMakeLists.txt
|   |   |-- lidar_processor.h / .cpp
|   |   |-- lio_engine.h / .cpp
|   |   +-- voxel_map.h / .cpp
|   |-- map/                        # 统一地图管理
|   |   |-- CMakeLists.txt
|   |   |-- map_manager.h / .cpp
|   |   +-- octree.h                # Octree<T> 模板 (header-only)
|   |-- vio/                        # 视觉惯性里程计
|   |   |-- CMakeLists.txt
|   |   |-- vio_engine.h / .cpp
|   |   +-- vio_gs.h / .cpp         # 3DGS 集成的 VIO 逻辑
|   |-- gs/                         # 3DGS 包装
|   |   |-- CMakeLists.txt
|   |   |-- gaussian_splatting.h / .cpp
|   |   +-- gs_types.h              # 桥接 lib3dgs 类型
|   |-- orchestrator/               # 主调度器
|   |   |-- CMakeLists.txt
|   |   +-- orchestrator.h / .cpp
|   +-- main.cpp                    # 入口
|-- external/                       # Git submodules
|   +-- lib3dgs/                    # 3DGS 渲染引擎 (CUDA + LibTorch)
|-- tests/
|   |-- CMakeLists.txt
|   |-- ekf/                        # EkfStateManager 测试
|   |-- lio/                        # LIO 模块测试
|   |-- map/                        # 八叉树测试
|   |-- sensor/                     # 同步逻辑测试
|   +-- integration/                # 端到端测试
|-- config/
|   |-- default.yaml                # 默认配置
|   +-- sensors/                    # 传感器标定模板
|-- scripts/
|   |-- run_bag.sh                  # 回放 rosbag 脚本
|   +-- eval_trajectory.py          # 轨迹评估工具
+-- .clang-format                   # 代码格式化
+-- .clang-tidy                     # 静态分析
```

## 3. 顶层 CMakeLists.txt

```cmake
# ============================================================
# CMakeLists.txt (顶层)
# ============================================================
cmake_minimum_required(VERSION 3.24)
project(gs-livo VERSION 0.1.0 LANGUAGES CXX CUDA)

# --- C++ Standard ---
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CUDA_STANDARD 20)
set(CMAKE_CUDA_STANDARD_REQUIRED ON)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

# --- Build type ---
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "Release" CACHE STRING "Build type" FORCE)
endif()

# --- Include cmake modules ---
list(APPEND CMAKE_MODULE_PATH "${CMAKE_SOURCE_DIR}/cmake")
include(CompilerOptions)
include(CudaOptions)
include(Dependencies)
include(Sanitizers)

# --- Options ---
option(GS_LIVO_BUILD_TESTS   "Build tests" ON)
option(GS_LIVO_BUILD_VIEWER  "Build viewer" OFF)
option(GS_LIVO_USE_SANITIZER "Enable sanitizers (debug builds)" OFF)

# --- Dependencies ---
# 此函数设置所有 find_package / FetchContent
gs_livo_setup_dependencies()

# --- Subdirectories ---
add_subdirectory(src)

if(GS_LIVO_BUILD_TESTS)
    enable_testing()
    add_subdirectory(tests)
endif()

# --- Install ---
include(InstallRules)
```

## 4. cmake/ 辅助模块

### 4.1 CompilerOptions.cmake

```cmake
# ============================================================
# cmake/CompilerOptions.cmake
# ============================================================
function(gs_livo_set_compiler_options target)
    if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
        target_compile_options(${target} PRIVATE
            -Wall -Wextra -Wpedantic
            -Wconversion -Wsign-conversion
            -Wshadow -Wnon-virtual-dtor
            -Wold-style-cast -Wcast-align
            -Wunused -Woverloaded-virtual
            $<$<CONFIG:Debug>:-O0 -g3 -fno-omit-frame-pointer>
            $<$<CONFIG:Release>:-O3 -DNDEBUG -march=native>
        )
    elseif(CMAKE_CXX_COMPILER_ID MATCHES "MSVC")
        target_compile_options(${target} PRIVATE
            /W4 /permissive- /Zc:__cplusplus
        )
    endif()
endfunction()
```

### 4.2 CudaOptions.cmake

```cmake
# ============================================================
# cmake/CudaOptions.cmake
# ============================================================
# 允许用户在命令行覆盖 CUDA 架构
set(GS_LIVO_CUDA_ARCH "75;80;86;89" CACHE STRING
    "CUDA architectures (compute capabilities)")

function(gs_livo_set_cuda_options target)
    set_target_properties(${target} PROPERTIES
        CUDA_ARCHITECTURES "${GS_LIVO_CUDA_ARCH}"
        CUDA_SEPARABLE_COMPILATION ON
        CUDA_RESOLVE_DEVICE_SYMBOLS OFF
    )

    target_compile_options(${target} PRIVATE
        $<$<COMPILE_LANGUAGE:CUDA>:
            --use_fast_math
            --expt-extended-lambda
            --expt-relaxed-constexpr
        >
    )
endfunction()
```

### 4.3 Dependencies.cmake

```cmake
# ============================================================
# cmake/Dependencies.cmake
# ============================================================
function(gs_livo_setup_dependencies)
    # --- Eigen3 (header-only, 必须) ---
    # 优先 find_package，fallback FetchContent
    find_package(Eigen3 3.4 QUIET)
    if(NOT Eigen3_FOUND)
        include(FetchContent)
        FetchContent_Declare(eigen
            GIT_REPOSITORY https://gitlab.com/libeigen/eigen.git
            GIT_TAG 3.4.0
        )
        FetchContent_MakeAvailable(eigen)
    endif()

    # --- OpenCV (图像处理) ---
    find_package(OpenCV 4 REQUIRED
        COMPONENTS core imgproc highgui
    )

    # --- spdlog (日志) ---
    find_package(spdlog QUIET)
    if(NOT spdlog_FOUND)
        include(FetchContent)
        FetchContent_Declare(spdlog
            GIT_REPOSITORY https://github.com/gabime/spdlog.git
            GIT_TAG v1.12.0
        )
        FetchContent_MakeAvailable(spdlog)
    endif()

    # --- yaml-cpp (配置) ---
    find_package(yaml-cpp QUIET)
    if(NOT yaml-cpp_FOUND)
        include(FetchContent)
        FetchContent_Declare(yaml-cpp
            GIT_REPOSITORY https://github.com/jbeder/yaml-cpp.git
            GIT_TAG 0.8.0
        )
        FetchContent_MakeAvailable(yaml-cpp)
    endif()

    # --- ZeroMQ (可视化 IPC) ---
    find_package(cppzmq QUIET)
    if(NOT cppzmq_FOUND)
        include(FetchContent)
        FetchContent_Declare(cppzmq
            GIT_REPOSITORY https://github.com/zeromq/cppzmq.git
            GIT_TAG v4.10.0
        )
        FetchContent_MakeAvailable(cppzmq)
    endif()

    # --- LibTorch (从官网下载) ---
    # 用户需设置 LibTorch_DIR 或下载到本地
    if(DEFINED ENV{LibTorch_DIR})
        set(LibTorch_DIR $ENV{LibTorch_DIR})
    endif()
    find_package(Torch REQUIRED)

    # --- lib3dgs (git submodule, CUDA) ---
    if(EXISTS "${CMAKE_SOURCE_DIR}/external/lib3dgs/CMakeLists.txt")
        add_subdirectory(external/lib3dgs)
    else()
        message(FATAL_ERROR
            "lib3dgs submodule not found. Run:\n"
            "  git submodule update --init --recursive"
        )
    endif()
endfunction()
```

## 5. src/ 库组织

### 5.1 src/CMakeLists.txt (工厂)

```cmake
# ============================================================
# src/CMakeLists.txt
# ============================================================
add_subdirectory(core)
add_subdirectory(interfaces)
add_subdirectory(ekf)
add_subdirectory(sensor)
add_subdirectory(imu)
add_subdirectory(lio)
add_subdirectory(map)
add_subdirectory(gs)
add_subdirectory(vio)
add_subdirectory(orchestrator)

# --- 可执行文件 ---
add_executable(gs_livo main.cpp)
target_link_libraries(gs_livo PRIVATE
    orchestrator_lib
    core_lib
)

gs_livo_set_compiler_options(gs_livo)
```

### 5.2 模块库示例

```cmake
# ============================================================
# src/core/CMakeLists.txt
# 基础库: 所有模块都依赖
# ============================================================
add_library(core_lib
    config.cpp
)
target_link_libraries(core_lib
    PUBLIC  Eigen3::Eigen
            spdlog::spdlog
    PRIVATE yaml-cpp
)
target_include_directories(core_lib PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})
gs_livo_set_compiler_options(core_lib)

# ============================================================
# src/ekf/CMakeLists.txt
# EKF 状态管理
# ============================================================
add_library(ekf_lib
    ekf_state_manager.cpp
)
target_link_libraries(ekf_lib
    PUBLIC  core_lib
            Eigen3::Eigen
)
target_include_directories(ekf_lib PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})
gs_livo_set_compiler_options(ekf_lib)

# ============================================================
# src/gs/CMakeLists.txt
# 3DGS 包装 (CUDA)
# ============================================================
add_library(gs_lib
    gaussian_splatting.cpp
)
target_link_libraries(gs_lib
    PUBLIC  core_lib
            Eigen3::Eigen
    PRIVATE ${TORCH_LIBRARIES}
            lib3dgs::lib3dgs_lib
)
target_include_directories(gs_lib PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})
gs_livo_set_compiler_options(gs_lib)
gs_livo_set_cuda_options(gs_lib)  # 启用 CUDA 选项

# ============================================================
# src/orchestrator/CMakeLists.txt
# ============================================================
add_library(orchestrator_lib
    orchestrator.cpp
)
target_link_libraries(orchestrator_lib
    PUBLIC  ekf_lib
    PRIVATE sensor_lib
            imu_lib
            lio_lib
            gs_lib
            vio_lib
            map_lib
)
target_include_directories(orchestrator_lib PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})
gs_livo_set_compiler_options(orchestrator_lib)
```

## 6. lib3dgs 集成注意事项

### 6.1 必须修改的 CMake 问题

lib3dgs 的 CMakeLists.txt 中使用了 `PROJECT_SOURCE_DIR`，当作为 `add_subdirectory` 引入时，`PROJECT_SOURCE_DIR` 指向的是外层项目而非 lib3dgs 自身。

**所有出现 `PROJECT_SOURCE_DIR` 的地方需要改为 `CMAKE_CURRENT_SOURCE_DIR`**。

```cmake
# 原版 (错误 - 作为子项目时指向父项目):
#   target_include_directories(3dgs_lib PRIVATE ${PROJECT_SOURCE_DIR}/cuda_rasterizer)

# 修正版:
#   target_include_directories(3dgs_lib PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/cuda_rasterizer)
```

### 6.2 必须修改的目标命名

lib3dgs 的目标名（如 `3dgs_lib`）需要在顶层明确，避免命名冲突。建议使用命名空间:

```cmake
# 在 lib3dgs/CMakeLists.txt 中添加:
add_library(lib3dgs::lib3dgs_lib ALIAS 3dgs_lib)
```

### 6.3 CUDA_ARCHITECTURES

lib3dgs 可能硬编码 CUDA 架构。需要确保在 lib3dgs 的 CMakeLists.txt 中:

```cmake
set_target_properties(3dgs_lib PROPERTIES
    CUDA_ARCHITECTURES "${CUDA_ARCHITECTURES}"  # 使用父项目传入的
)
```

或者在顶层统一设置:
```cmake
# 在顶层 CMakeLists.txt 设置，所有子项目继承
set(CMAKE_CUDA_ARCHITECTURES "75;80;86;89" CACHE STRING "CUDA architectures")
```

## 7. 测试框架

### 7.1 Catch2

```cmake
# cmake/Dependencies.cmake 中添加:
find_package(Catch2 QUIET)
if(NOT Catch2_FOUND)
    include(FetchContent)
    FetchContent_Declare(catch2
        GIT_REPOSITORY https://github.com/catchorg/Catch2.git
        GIT_TAG v3.5.0
    )
    FetchContent_MakeAvailable(catch2)
endif()
```

### 7.2 测试目录结构

```cmake
# ============================================================
# tests/CMakeLists.txt
# ============================================================
find_package(Catch2 REQUIRED)

add_subdirectory(ekf)
add_subdirectory(map)
add_subdirectory(sensor)

# --- 集成测试 ---
add_executable(integration_test
    integration/test_sync_pipeline.cpp
    integration/test_lio_vio_loop.cpp
)
target_link_libraries(integration_test PRIVATE Catch2::Catch2 orchestrator_lib)
catch_discover_tests(integration_test)
```

```cmake
# ============================================================
# tests/ekf/CMakeLists.txt
# ============================================================
add_executable(ekf_test
    test_ekf_state_manager.cpp
    test_ekf_predict.cpp
    test_ekf_update.cpp
)
target_link_libraries(ekf_test PRIVATE Catch2::Catch2 ekf_lib)
catch_discover_tests(ekf_test)
```

### 7.3 测试类型清单

| 测试 | 类型 | 内容 |
|------|------|------|
| EkfStateManager 并发 | 单元 | 验证 shared_lock / unique_lock 行为 |
| EKF predict | 单元 | 给定 IMU 输入，验证状态传播正确性 |
| EKF update | 单元 | 给定 H, residual, R，验证状态更新 |
| Octree 插入/查询 | 单元 | 验证空间查询结果正确性 |
| 传感器同步 | 单元 | 给定时间偏移的数据，验证对齐正确性 |
| ConfigManager | 单元 | YAML 解析 + 默认值 |
| 集成测试 | 集成 | rosbag 回放，轨迹 ATE 对比 |

## 8. CI/CD 基础设计

### 8.1 GitHub Actions

```yaml
# .github/workflows/ci.yml
name: CI
on: [push, pull_request]

jobs:
  build:
    runs-on: ubuntu-22.04
    strategy:
      matrix:
        build_type: [Debug, Release]

    steps:
    - uses: actions/checkout@v4
      with:
        submodules: recursive

    - name: Install system dependencies
      run: |
        sudo apt-get update
        sudo apt-get install -y \
          libopencv-dev \
          libeigen3-dev \
          libyaml-cpp-dev \
          libspdlog-dev \
          libzmq3-dev \
          ninja-build

    - name: Setup CUDA
      uses: Jimver/cuda-toolkit@v0.2
      with:
        cuda: '11.8'

    - name: Download LibTorch
      run: |
        wget https://download.pytorch.org/libtorch/cu118/libtorch-cxx11-abi-shared-with-deps-2.0.1.zip
        unzip libtorch-cxx11-abi-shared-with-deps-2.0.1.zip
        echo "LibTorch_DIR=${PWD}/libtorch" >> $GITHUB_ENV

    - name: Configure
      run: |
        cmake -B build -G Ninja \
          -DCMAKE_BUILD_TYPE=${{ matrix.build_type }} \
          -DGS_LIVO_BUILD_TESTS=ON

    - name: Build
      run: cmake --build build --parallel

    - name: Test
      run: cd build && ctest --output-on-failure
```

### 8.2 代码质量

```cmake
# cmake/CodeQuality.cmake (可选)
# clang-format 检查
# clang-tidy 静态分析
# include-what-you-use 检查
```

格式化配置:
```yaml
# .clang-format
BasedOnStyle: Google
IndentWidth: 4
ColumnLimit: 100
AllowShortFunctionsOnASingleLine: None
```

## 9. 构建命令速查

```bash
# 完整构建
mkdir build && cd build
cmake .. -G Ninja -DCMAKE_BUILD_TYPE=Release
cmake --build . --parallel

# Debug 构建 + ASAN
cmake .. -G Ninja -DCMAKE_BUILD_TYPE=Debug -DGS_LIVO_USE_SANITIZER=ON
cmake --build . --parallel

# 运行测试
cd build && ctest --output-on-failure

# 运行单个测试
./build/tests/ekf/ekf_test "EKF predict"

# 覆盖 CUDA 架构
cmake .. -DGS_LIVO_CUDA_ARCH="75;80"

# 指定 LibTorch 路径
cmake .. -DLibTorch_DIR=/path/to/libtorch
```
