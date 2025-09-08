# RISC-V-PCPU-with-L1-Cache-and-Simple-CSR-Registers
# 五级流水线 RISC-V CPU 核设计

A Simple 5-Stage Pipelined RISC-V CPU with L1 Cache and CSRs, implemented in SystemVerilog.

## 简介

这是一个基于 RISC-V RV32I 指令集架构实现的简单五级流水线处理器核。该项目旨在学习和实践现代处理器设计中的关键技术，包括流水线、数据前推、冒险检测、缓存（Cache）以及控制状态寄存器（CSR）等。

整个设计采用 SystemVerilog 硬件描述语言编写，具有清晰的模块化结构，易于理解和扩展，本project可运行在Vivado 2023上。

## 核心特性

*   **RISC-V 架构**: 实现了 RV32I 基本整数指令集。
*   **五级经典流水线**:
    1.  **IF** (取指, Instruction Fetch)
    2.  **ID** (译码, Instruction Decode)
    3.  **EX** (执行, Execute)
    4.  **MEM** (访存, Memory Access)
    5.  **WB** (写回, Write Back)
*   **哈佛结构**: 指令内存与数据内存分离。
    *   **指令内存 (I-Mem)**: 只读，单周期响应，后期将更新为指令缓存和指令内存的协同工作。
    *   **数据内存 (D-Mem)**: 基于 DRAM 模型实现，多周期延时读写。
*   **L1 数据缓存 (D-Cache)**:
    *   **2路组相联 (2-way Set-Associative)**: 提高了缓存命中率。
    *   **LRU (Least Recently Used)** 替换策略: 实现了高效的缓存块替换算法。
*   **冒险处理**:
    *   通过**数据前推 (Forwarding)** 解决大部分数据冒险。
    *   通过**停顿 (Stall)** 解决加载-使用（Load-Use）数据冒险。
    *   通过一直预测分支不跳转以及在译码阶段计算跳转地址来处理控制冒险。
*   **CSR 支持**: 实现了基本的控制状态寄存器，用于处理器状态控制和信息记录。支持六个基本的CSR指令和Ebreak，Ecall指令，更多指令仍然有待完善。

## 项目结构

本设计的顶层模块为 `top`，它实例化了 CPU 核心 `cpu` 和内存模块 `mem`。CPU 核心 `cpu` 内部包含了构成处理器的各个组件。

```
top (top.sv)
│
├── mem: MEM_Dram (Memory0.sv)       // 数据 DRAM 内存
│   ├── DRAM_data
│   └── DRAM_inst
│
└── cpu: Core (Datapath.sv)           // CPU 核心
    ├── pc_mux_branch                  // 分支跳转 PC 选择器
    ├── pc_mux_csr                     // CSR 跳转 PC 选择器
    ├── pc_mux_exception               // 异常处理 PC 选择器
    ├── ProgrammeCounter: PC           // 程序计数器
    ├── regs: Reg_Files (Registers.sv) // 寄存器堆
    ├── control: Control               // 控制单元
    ├── gene: Imme_Gen                 // 立即数生成器
    ├── hazard: Hazard                 // 冒险检测单元
    ├── alu: ALU                       // 算术逻辑单元
    ├── BC: Branch_Comparision         // 分支比较单元
    ├── dcache: DataCache (Cache.sv)   // 数据缓存
    ├── sw_forward: sw_forward         // 数据前推逻辑
    ├── CSR_Regs: CSR_Regfiles         // CSR 寄存器堆
    ├── internal_regs_package          // 流水线寄存器数据类型结构体
    ├── cache_pkg                      // 缓存和CPU与内存交互的接口
    └── ... (其他多路选择器、加法器等)
    
```

## 环境与工具链

*   **硬件描述语言**: SystemVerilog
*   **仿真工具**:
    *   Vivado Simulator
*   **综合工具**:
    *   Vivado

## 快速开始

### 仿真

您可以使用提供的 testbench 文件进行功能仿真。

在tb.sv中通过向顶层测试模块传入带有机器指令的16进制文档(.hex)即可初始化指令内存。数据内存可以另外指定文件初始化。

### 综合与实现 

您可以使用 Vivado 等 EDA 工具对设计进行综合，以评估资源消耗和时序性能，或在CSR寄存器中自行实现监视功能。

1.  在 Vivado 中创建一个新项目。
2.  添加 当前 目录下的所有 `.sv` 文件作为设计源文件。
3.  添加/修改约束文件 (constraints)。
4.  运行“Run Synthesis”和“Run Implementation”。

## 未来工作
*   [ ] 实现自动化测试工作，运用Sppike，QEMU等工具进行进一步模拟。
*   [ ] 实现更完整的 RISC-V 扩展指令集 (如 M, A类指令)。
*   [ ] 实现完善的异常和中断处理机制。
*   [ ] 优化分支预测逻辑 (例如，实现动态分支预测器)。
*   [ ] 添加 L1 指令缓存 (I-Cache)。
*   [ ] 移植到具体的 FPGA 开发板上进行原型验证。
*   [ ] 实现指令重排和指令保留站，初步实现乱序执行核心。
