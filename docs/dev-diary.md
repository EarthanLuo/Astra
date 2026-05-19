# GS-LIVO 重写开发日记

## 概述

- **项目**: GS-LIVO 从零重写
- **起点**: 2026-05-19
- **目标**: 彻底解耦 ROS 依赖，现代化 C++20 架构，10 阶段渐进实现

---

## Phase 1: 基础设施 — 2026-05-19

### 目标

可编译的空项目框架：CMake 构建系统、消息类型、配置管理、日志。

### 产出

```
CMakeLists.txt
cmake/CompilerOptions.cmake
cmake/CudaOptions.cmake
cmake/Dependencies.cmake
cmake/InstallRules.cmake
cmake/Sanitizers.cmake       (额外)
src/main.cpp
src/core/types.h
src/core/config.h
src/core/config.cpp
src/core/concurrent_queue.h
src/core/logger.h
src/CMakeLists.txt
src/core/CMakeLists.txt
src/{interfaces,ekf,sensor,imu,lio,map,gs,vio,orchestrator}/CMakeLists.txt  (占位)
config/default.yaml
tests/CMakeLists.txt
.clang-format
```

### 关键决策

| 决策 | 选择 | 原因 |
|------|------|------|
| 依赖获取方式 | `find_package` (系统包) | FetchContent 在国内极慢，4s vs timeout |
| CUDA | 可选，`CUDAToolkit_FOUND` 守卫 | 早期阶段不需要 GPU |
| OpenCV | 可选，`#ifdef GS_LIVO_HAS_OPENCV` | Phase 2+ 才用，先不强制 |
| 测试框架 | **GTest** (非计划中的 Catch2) | ECC C++ 规则要求 GoogleTest |
| 代码风格 | clang-format (Google style, 4-space indent) | C++ ECC 规则要求 |

### 偏差记录

1. **FetchContent → 系统包**: 计划用 FetchContent 安装 Eigen3/spdlog/yaml-cpp，实际全部走 `apt install` 的系统包。`Dependencies.cmake` 移除了所有 FetchContent fallback。
2. **Catch2 → GTest**: 计划用 Catch2，切换到 GTest 以符合 ECC C++ 规则 (`~/.claude/rules/ecc/cpp/testing.md`)。
3. **Sanitizers.cmake**: 额外添加了 `-fsanitize=address,undefined` 的 CMake 模块以备后续调试。

### 验证

```
✅ cmake -B build -G Ninja -DCMAKE_BUILD_TYPE=Debug   (3.2s)
✅ cmake --build build --parallel                      (4 targets)
✅ ./build/src/gs_livo --config config/default.yaml    (正常输出)
✅ ctest                                                (No tests were found!!!)
✅ clang-format --dry-run (后格式化)                    (通过)
```

### 提交

```
83cf0d6 fix: Catch2 → GTest + clang-format 格式化所有源文件
a775ac3 feat: Phase 1 基础设施 — 可编译的空项目框架
```

---

## Phase 2: 传感器驱动 + 同步

_待实施_

## Phase 3: IMU 处理 + EKF 状态管理

_待实施_

## Phase 4: LiDAR 预处理 + LIO

_待实施_

## Phase 5: 3DGS 引擎集成

_待实施_

## Phase 6: VIO + 3DGS 融合

_待实施_

## Phase 7: Orchestrator 主循环

_待实施_

## Phase 8: 可视化 + 日志记录

_待实施_

## Phase 9: 性能优化

_待实施_

## Phase 10: 测试 + 调优

_待实施_
