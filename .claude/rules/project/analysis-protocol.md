# Phase 1 分析协议

## Agent 行为规则

- 使用 architect（opus）做架构分析和方案设计
- 使用 planner（opus）做实施计划拆解
- 使用 code-explorer（sonnet）做代码扫描和依赖追踪
- 所有 agent 并行 spawn，主 session 做合成

## 分析流程

### Step 1: 全景扫描

  code-explorer 并行扫描：

- ROS 节点拓扑 → `rosnode list`, launch 文件
- CMake 模块结构 → `CMakeLists.txt` 递归
- CUDA 文件 → `**/*.cu`, `**/*.cuh`
- 核心头文件 → `**/*.h` 顶层接口

### Step 2: 深度分析

  architect 分析：

- 模块耦合度（哪些模块不该互相依赖）
- 数据传输热点（cudaMemcpy 频次、ROS topic 带宽）
- 架构模式（是面向对象还是过程式？）
- 技术债务（裸指针、全局变量、巨型源文件）

### Step 3: 方案产出

  architect + planner 协作：

- 目标架构图（模块划分 + 接口）
- 迁移策略（先拆哪些后拆哪些）
- ADR(架构决策记录)
- 分阶段实施计划

## 产出物模板

- `.claude/plans/architecture-overview.md` — 架构分析报告
- `.claude/plans/adr-*.md` — 架构决策记录
- `.claude/plans/implementation-phases.md` — 分阶段计划
