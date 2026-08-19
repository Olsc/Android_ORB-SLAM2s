# Flowchart Generator Tools (`flowchart_generator_tools`)

本目录包含用于自动生成项目架构与算法流程图的矢量图绘制工具。

---

## 🛠️ 脚本工具说明

### 1. `generate_architecture_flowchart.py` (高保真视觉因果流程图生成器)

使用 Graphviz 引擎生成 `Android_ORB-SLAM2s` 的高保真系统因果逻辑流程图（支持中英文多语言输出）。

#### ✨ 核心特性
- **现代化视觉风格**：采用渐变填充、圆角节点与软阴影效果，生成媲美专业设计工具的 SVG 矢量图。
- **多语言原生支持**：支持导出英文版（`aesthetic_visual_causal_flow_en.svg`）与中文版（`aesthetic_visual_causal_flow_zh.svg`）。
- **模块化因果图示**：完整覆盖 Camera 线程、ORB 特征提取、Tracking 跟踪、Local Mapping 局部建图与 Loop Closing 闭环检测流程。

#### 📦 环境要求

需要安装 Python `graphviz` 库以及系统级的 Graphviz 可执行文件：

```bash
pip install graphviz
```

> **注意**：Windows 环境下需确保系统已安装 Graphviz（或在 Conda 环境中执行 `conda install graphviz`）。

#### 🚀 使用方法

```bash
# 运行脚本，自动生成中英文 SVG 流程图
python docs/flowchart_generator_tools/generate_architecture_flowchart.py
```

生成的文件将放置在 `docs/` 根目录下：
- `docs/aesthetic_visual_causal_flow_en.svg`
- `docs/aesthetic_visual_causal_flow_zh.svg`
