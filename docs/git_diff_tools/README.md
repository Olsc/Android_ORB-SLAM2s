# Git Diff Tools (`git_diff_tools`)

本目录包含用于可视化分析 Git 变更与提交历史的交互式仪表盘生成工具。

---

## 🛠️ 脚本工具说明

### 1. `generate_git_diff_dashboard.py` (Git 变更交互式仪表盘生成器)

自动分析 Git 仓库提交差异，并生成极具视觉吸引力、单文件可运行的 HTML 交互式文件树与仪表盘。

#### ✨ 核心特性
- **完整目录树还原**：整合新增、删除、修改、移动/重命名以及未修改的文件，100% 还原项目目录结构。
- **实时过滤与搜索**：支持按文件状态（新增/删除/修改/移动/未修改）过滤以及按文件名/路径实时搜索。
- **逐文件 Git 提交历史与 Diff 视图**：点击任意节点弹窗查看该文件在对比区间内的 Commit 历史及逐行代码 Diff 高亮。
- **单文件独立运行**：生成的 HTML 无第三方 JS 框架运行时依赖，直接双击或在浏览器中打开即可使用。

#### 🚀 使用方法

```bash
# 1. 默认对比首次提交至 HEAD，在当前目录生成 git_diff_tree.html
python docs/git_diff_tools/generate_git_diff_dashboard.py

# 2. 指定起始 Commit Hash 或分支进行对比
python docs/git_diff_tools/generate_git_diff_dashboard.py -s 355cdc1

# 3. 指定输出 HTML 文件路径
python docs/git_diff_tools/generate_git_diff_dashboard.py -s main -o diff_report.html
```

#### 📋 参数说明
| 参数 | 长参数 | 默认值 | 描述 |
| :--- | :--- | :--- | :--- |
| `-s` | `--start` | `None` (首次 Commit) | 指定对比起点的 Commit Hash、分支名或 Tag |
| `-o` | `--output` | `git_diff_tree.html` | 指定生成的 HTML 仪表盘文件路径及名称 |
