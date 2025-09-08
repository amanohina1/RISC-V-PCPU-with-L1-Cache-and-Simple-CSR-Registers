# RISC-V-PCPU-with-L1-Cache-and-Simple-CSR-Registers
# 五级流水线 RISC-V CPU 核设计 (PCPU—Core)

A 5-Stage Pipelined RISC-V CPU with L1 Cache, CSRs and precise exception handling, implemented in SystemVerilog.

## 简介

这是一个基于 RISC-V RV32I 指令集架构实现的简单五级流水线处理器核。该项目旨在学习和实践现代处理器设计中的关键技术，包括流水线、数据前推、冒险检测、缓存（Cache）以及控制状态寄存器（CSR）等。

整个设计采用 SystemVerilog 硬件描述语言编写，具有清晰的模块化结构，易于理解和扩展，本project可运行在Vivado 2023.2上。

## 核心特性

*   **RISC-V 架构**: 实现了 RV32I 基本整数指令集。
*   **五级经典流水线**:
    1.  **IF** (取指, Instruction Fetch)
    2.  **ID** (译码, Instruction Decode)
    3.  **EX** (执行, Execute)
    4.  **MEM** (访存, Memory Access)
    5.  **WB** (写回, Write Back)
*   **哈佛结构**: 指令内存与数据内存分离。
    *   **指令内存 (I-Mem)**: 只读，单周期响应，我目前用它来模拟一个巨大的指令缓存。后期将更新为指令缓存和指令内存的协同工作。
    *   **数据内存 (D-Mem)**: 基于 DRAM 模型实现，多周期延时读写，与处理器通过一个L1缓存交互。
*   **L1 数据缓存 (D-Cache)**:
    *   **2路组相联 (2-way Set-Associative)**: 提高了缓存命中率。
    *   **LRU (Least Recently Used)** 替换策略: 实现了高效的缓存块替换算法。
*   **冒险处理**:
    *   通过**数据前推 (Forwarding)** 解决大部分数据冒险。
    *   通过**停顿 (Stall)** 解决加载-使用（Load-Use）数据冒险。
    *   通过一直预测分支不跳转以及在译码阶段计算跳转地址来处理控制冒险。
*   **CSRs & 异常处理:** 实现了 `mcycle`, `minstret` 等标准性能计数器与自定义计数器。支持M/U模式切换，能够精确捕获并处理访问/写入内存时的地址未对齐、非法指令等异常，支持Ecall和Ebreak指令。

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
    ├── dcache: DataCache (Cache.sv)   // 数据缓存，hit时单周期返回数据
    ├── sw_forward: sw_forward         // 数据前推逻辑
    ├── CSR_Regs: CSR_Regfiles         // CSR 寄存器堆
    ├── internal_regs_package          // 流水线寄存器数据类型结构体
    ├── cache_pkg                      // 缓存和CPU与内存交互的接口
    └── ... (其他多路选择器、加法器等)
    
```
设计遵循“控制与数据通路分离”的核心思想：
- **`Control` 单元:** 纯组合逻辑，根据指令操作码生成所有控制信号。
- **`Hazard` 单元:** 负责检测数据冒险和控制冒险，生成流水线停顿与冲刷信号。
- **`Datapath` 模块:** 包含PC、寄存器堆、ALU、流水线寄存器以及L1缓存等所有状态和执行单元。

## 环境与工具链

*   **硬件描述语言**: SystemVerilog
*   **仿真工具**:
    *   Vivado Simulator
*   **综合工具**:
    *   Vivado

## 快速开始

### 仿真

你可以使用提供的 testbench 文件进行功能仿真。

在tb.sv中通过向顶层测试模块传入带有机器指令的16进制文档(.hex)即可初始化指令内存。数据内存可以另外指定文件初始化。

### 综合与实现 

你可以使用 Vivado 等 EDA 工具对设计进行综合，以评估资源消耗和时序性能，或在CSR寄存器中自行实现监视功能。

1.  在 Vivado 中创建一个新项目。
2.  添加 当前 目录下的所有 `.sv` 文件作为设计源文件。
3.  添加/修改约束文件 (constraints)。
4.  运行“Run Synthesis”和“Run Implementation”。

## 一个性能基准测试

*   **测试程序:** **递归计算斐波那契数列第15项 (F(15))**。(或者第N项也可以)
```
_start:
    li      sp, 4000
    jal     ra, main
done:
    li      a7, 0x0A          # Use a unique signature to represent successful exit
    j       done

main:
	# Prologue
	addi    sp, sp, -8
    sw      ra, 4(sp)

   
	li      a0, 15           # We want to calculate F(15)
    jal     ra, fib      	 # Call the fibonacci function

	# Epilogue
	lw      ra, 4(sp)
    addi    sp, sp, 8

    ret

# The Fibonacci Function: fib(n)
fib:
    # Prologue: stack frame 
    addi    sp, sp, -16     
    sw      ra, 12(sp)      # Save return address
    sw      s0, 8(sp)       # Save callee-saved register s0 (will hold n)
    sw      s1, 4(sp)       # Save callee-saved register s1 (will hold fib(n-1))
    
    mv      s0, a0          # s0 = n, save a copy of the argument

    li      t0, 1
    ble     a0, t0, base_case

    # Calculate fib(n-1)
    addi    a0, s0, -1      # a0 = n - 1
    jal     ra, fib         # call fib(n-1)
    mv      s1, a0          # s1 = fib(n-1)

    # Calculate fib(n-2)
    addi    a0, s0, -2      # a0 = n - 2
    jal     ra, fib         # call fib(n-2)

    add     a0, s1, a0

    j       epilogue        # Jump to the function exit

base_case:
    # For n=1, the result is n.

epilogue:
    # Epilogue
    lw      s1, 4(sp)
    lw      s0, 8(sp)
    lw      ra, 12(sp)
    addi    sp, sp, 16      
    ret
```

*   **选择原因:** 该程序具有**深度递归**和**频繁的内存访问**（函数调用栈操作），能对CPU的控制流逻辑（分支/跳转）、数据通路（前递/冒险处理）和内存子系统产生巨大的压力。
*   **测量工具:** 通过读取以下CSR寄存器获取硬件层面的精确数据：
    1.  `mcycle`: 记录总消耗的时钟周期数。
    2.  `minstret`: 记录成功退休（执行完毕）的指令总数。
    3.  **自定义性能计数器1:** 记录因内存访问（主要是加载-使用冒险）导致的流水线停顿周期数。
    4.  **自定义性能计数器2:** 记录被采纳（Taken）的分支和跳转指令的总数。

*   **关键性能指标**

| 性能指标 | 十六进制 (Hex) | 十进制 (Decimal) | 描述 |
| :--- | :--- | :--- | :--- |
| **总运行周期数** | `0xa7c8` | **42,952** | 完成F(15)计算消耗的总时钟周期 |
| **总退休指令数** | `0x90a4` | **37,028** | 成功执行的指令总数 |
| **内存停顿周期** | `0x205` | **517** | 因加载-使用等数据冒险导致的停顿 |
| **被采纳的分支/跳转数** | `0x1722` | **5,922** | 控制流发生改变的总次数 |

## 性能分析

### 整体流水线效率: CPI ≈ 1.16

**CPI (Cycles Per Instruction)** 是衡量流水线效率的黄金标准，其理论最优值为1.0。

*   **计算:** `CPI = 总周期数 / 总指令数 = 42,952 / 37,028 ≈ 1.16`

**结论:** CPI为 **1.16** 是一个**极为出色**的指标。它证明了CPU的流水线设计（特别是前递和冒险检测单元）效率极高，成功地解决了绝大多数数据冒险，使CPU在绝大部分时间里都保持着每个周期完成一条指令的高速运转。

### 流水线停顿归因分析

CPU为了执行37,028条指令，付出了 `42,952 - 37,028 = 5,924` 个停顿周期的代价。通过自定义性能计数器，我们可以精确地剖析这些停顿的来源：

| 停顿来源 | 相关周期/事件数 | 占总停顿比例 | 根本原因 |
| :--- | :--- | :--- | :--- |
| **控制冒险** | **~5,407** 周期 | **~91.3%** | **分支/跳转指令** |
| **数据冒险** | **517** 周期 | **~8.7%** | **内存加载-使用** |

*   **控制冒险是主导因素:**
    我们测量到总共有 **5,922** 次被采纳的分支/跳转，而这导致的非内存停顿周期高达 **5,407** 个。这表明，**当前设计为每一次控制流的改变，付出了约0.91个周期的固定开销**。这主要是由于流水线在等待分支结果时产生的“气泡”所致。

*   **数据冒险是次要因素:**
    由`lw`后紧跟`ret`这类指令导致的、无法通过前递完全消除的内存访问停顿只有 **517** 个周期，在总停顿中占比很小。

### 最终结论

本项目成功设计并实现了一个**高性能、高效率、功能较完备的RISC-V处理器核心**。通过内置的硬件性能监视器进行的精确剖析数据显示，CPU的基础流水线效率已接近理论最优值（CPI ≈ 1.16），其主要的性能瓶颈是**控制冒险**。

这个数据驱动的分析，验证了本CPU设计的正确性与Robustness和应用LRU策略的二路组相联缓存的有效性，为下一步的性能优化指明了方向：为消除91.3%的停顿周期，我们要实现一个动态分支预测器，以避免因分支/跳转导致的流水线冲刷。

## 未来工作
*   [ ] 优化分支预测逻辑。
*   [ ] 添加 L1 指令缓存。
*   [ ] 实现更完整的 RISC-V 扩展指令集 (如 M, A类指令)。     
*   [ ] 实现自动化测试，运用Sppike，QEMU等工具进行进一步模拟。
*   [ ] 实现完善的异常和中断处理机制。
*   [ ] 移植到具体的 FPGA 开发板上进行原型验证。
*   [ ] 实现指令重排和指令保留站，初步实现乱序执行核心。

## 设计与调试亮点

本项目在开发过程中解决了一系列经典的硬件设计挑战。其中一些关键问题的分析与解决过程被记录在了 [设计与调试日志 (DESIGN_INSIGHTS.md)](./DESIGN_INSIGHTS.md) 中，包括对一个由内存写时序竞争导致的堆栈损坏BUG的深度剖析。

