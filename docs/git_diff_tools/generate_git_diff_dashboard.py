#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
Git 变更交互式仪表盘生成器 (Git Diff Interactive Dashboard Generator)
功能：
1. 对比指定提交(或首次提交)到最新提交的变更。
2. 自动识别新增、删除、修改、重命名/移动的文件。
3. 整合未修改的文件，还原项目完整目录树结构。
4. 抓取每个变更文件在对比区间内的详细 Git 提交历史。
5. 生成包含树状图、分类过滤、实时搜索、点击弹窗查看修改历史的单文件 HTML 交互网页。
"""

import os
import sys
import subprocess
import json
import argparse
import html
from typing import Dict, Any, List, Tuple

# ==========================================
# 1. GIT 数据提取模块
# ==========================================

def run_command(cmd: List[str]) -> str:
    """运行系统命令并返回输出字符"""
    try:
        result = subprocess.run(cmd, stdout=subprocess.PIPE, stderr=subprocess.PIPE, encoding='utf-8', errors='replace', check=True)
        return (result.stdout or "").strip()
    except subprocess.CalledProcessError as e:
        print(f"[警告] 执行命令失败: {' '.join(cmd)}. 错误: {e.stderr.strip()}", file=sys.stderr)
        return ""
    except FileNotFoundError:
        print(f"[警告] 未找到系统命令: {cmd[0]}。将使用演示数据。", file=sys.stderr)
        return ""

def get_file_commits(filepath: str, start_commit: str) -> List[Dict[str, str]]:
    """获取某个文件在对比区间内的提交记录"""
    cmd = ["git", "log", "-n", "5", f"--format=%h|%an|%ar|%s", f"{start_commit}..HEAD", "--", filepath]
    log_output = run_command(cmd)
    commits = []
    if log_output:
        for line in log_output.split('\n'):
            parts = line.split('|')
            if len(parts) >= 4:
                commits.append({
                    "sha": parts[0],
                    "author": parts[1],
                    "date": parts[2],
                    "message": parts[3]
                })
    return commits

def get_file_diff(filepath: str, start_commit: str, status: str = "modified", old_path: str = None) -> List[Dict[str, str]]:
    """获取文件在对比区间内的逐行 diff 详情"""
    if status in ("unchanged",):
        return []

    # 对于已删除文件，使用 parent of HEAD 对比（显示所有行被删）
    # 对于重命名文件，使用旧路径
    if status == "renamed" and old_path:
        cmd = ["git", "diff", "-U3", "-M", f"{start_commit}..HEAD", "--", old_path]
    else:
        cmd = ["git", "diff", "-U3", f"{start_commit}..HEAD", "--", filepath]

    diff_output = run_command(cmd)
    if not diff_output:
        return []

    lines = []
    for line in diff_output.split('\n'):
        if not line:
            continue
        # 跳过元数据行
        if line.startswith('diff --git') or line.startswith('index ') or \
           line.startswith('--- ') or line.startswith('+++ '):
            continue
        if line.startswith('@@'):
            lines.append({"type": "hunk", "content": line})
        elif line.startswith('+'):
            lines.append({"type": "added", "content": line})
        elif line.startswith('-'):
            lines.append({"type": "removed", "content": line})
        else:
            lines.append({"type": "context", "content": line})

    # 限制 diff 行数，防止超大 diff 撑爆页面
    MAX_DIFF_LINES = 800
    if len(lines) > MAX_DIFF_LINES:
        lines = lines[:MAX_DIFF_LINES]
        lines.append({"type": "hunk", "content": f"... 差异过大，仅显示前 {MAX_DIFF_LINES} 行 ..."})
    return lines


def get_git_changes(start_commit_arg: str = None) -> Tuple[List[Dict[str, Any]], bool, str]:
    """
    分析 Git 变更，返回变更文件列表、是否为真实 Git 以及对比起点 Commit Hash
    """
    # 检查当前目录是否在 git 仓库中
    is_git = run_command(["git", "rev-parse", "--is-inside-work-tree"])
    if is_git != "true":
        print("[提示] 当前目录不是 Git 仓库，将自动启用 [演示模式] 生成模拟数据...")
        return get_mock_changes(), False, "N/A"

    start_commit = None

    # 验证用户指定的起始 commit
    if start_commit_arg:
        resolved_commit = run_command(["git", "rev-parse", "--verify", start_commit_arg])
        if resolved_commit:
            start_commit = resolved_commit.split('\n')[0]
            print(f"[日志] 成功验证用户指定的起始 Commit: {start_commit[:7]} ({start_commit_arg})")
        else:
            print(f"[警告] 无法解析您指定的 Commit: '{start_commit_arg}'。将自动获取首次提交进行对比...", file=sys.stderr)

    # 寻找首次提交 (First Commit)
    if not start_commit:
        first_commit = run_command(["git", "rev-list", "--max-parents=0", "HEAD"])
        if not first_commit:
            print("[警告] 无法获取 Git 首次提交 hash，将启用 [演示模式]...")
            return get_mock_changes(), False, "N/A"
        start_commit = first_commit.split('\n')[0]
        print(f"[日志] 默认采用首次提交作为对比起点: {start_commit[:7]}")

    print(f"[日志] 对比提交 {start_commit[:7]} 到最新提交 HEAD 之间的所有变更...")

    # 1. 采用 -M 参数启用重命名检测获取变更状态
    diff_status = run_command(["git", "diff", "--name-status", "-M", start_commit, "HEAD"])
    
    # 2. 获取原始行数统计数据（无需在 numstat 中启用重命名检测，以便分别计算旧文件删除和新文件新增的行数）
    diff_numstat = run_command(["git", "diff", "--numstat", start_commit, "HEAD"])
    
    # 3. 列出当前最新提交的所有文件（未修改文件基础）
    all_tracked_files = run_command(["git", "ls-files"])

    if not diff_status:
        print("[提示] 指定提交到当前提交之间没有检测到代码差异，启用 [演示模式] 以便查看效果...")
        return get_mock_changes(), False, start_commit

    # 解析行数统计
    numstat_map = {}
    if diff_numstat:
        for line in diff_numstat.strip().split('\n'):
            if not line: continue
            parts = line.split('\t')
            if len(parts) >= 3:
                added = int(parts[0]) if parts[0].isdigit() else 0
                deleted = int(parts[1]) if parts[1].isdigit() else 0
                numstat_map[parts[2]] = (added, deleted)

    # 收集已变更的文件
    changed_paths = set()
    changes = []
    
    # 解析状态信息
    for line in diff_status.strip().split('\n'):
        if not line: continue
        parts = line.split('\t')
        if len(parts) < 2: continue
        
        status_code = parts[0]
        
        if status_code.startswith('R'): # 重命名 / 移动
            old_path = parts[1]
            new_path = parts[2]
            
            # 重命名文件的行数变化：新路径对应新增行数，旧路径对应删除行数
            added = numstat_map.get(new_path, (0, 0))[0]
            deleted = numstat_map.get(old_path, (0, 0))[1]
            
            print(f"[日志] 发现移动文件: {old_path} -> {new_path}")
            
            changes.append({
                "path": new_path,
                "status": "renamed",
                "added": added,
                "deleted": deleted,
                "old_path": old_path,
                "commits": get_file_commits(new_path, start_commit),
                "diff": get_file_diff(new_path, start_commit, "renamed", old_path)
            })
            changed_paths.add(new_path)
            changed_paths.add(old_path)
            
        elif status_code.startswith('A'): # 新增
            filepath = parts[1]
            added, deleted = numstat_map.get(filepath, (0, 0))
            changes.append({
                "path": filepath,
                "status": "added",
                "added": added,
                "deleted": deleted,
                "old_path": None,
                "commits": get_file_commits(filepath, start_commit),
                "diff": get_file_diff(filepath, start_commit, "added")
            })
            changed_paths.add(filepath)

        elif status_code.startswith('D'): # 删除
            filepath = parts[1]
            added, deleted = numstat_map.get(filepath, (0, 0))
            changes.append({
                "path": filepath,
                "status": "deleted",
                "added": added,
                "deleted": deleted,
                "old_path": None,
                "commits": [],
                "diff": []
            })
            changed_paths.add(filepath)
            
        else: # 修改 M 或者其他
            filepath = parts[1]
            added, deleted = numstat_map.get(filepath, (0, 0))
            changes.append({
                "path": filepath,
                "status": "modified",
                "added": added,
                "deleted": deleted,
                "old_path": None,
                "commits": get_file_commits(filepath, start_commit),
                "diff": get_file_diff(filepath, start_commit, "modified")
            })
            changed_paths.add(filepath)

    # 4. 补充加入"未修改"文件到列表中
    if all_tracked_files:
        for filepath in all_tracked_files.strip().split('\n'):
            if filepath not in changed_paths:
                changes.append({
                    "path": filepath,
                    "status": "unchanged",
                    "added": 0,
                    "deleted": 0,
                    "old_path": None,
                    "commits": [], # 未修改文件在对比区间内没有任何 Commit
                    "diff": []
                })

    print(f"[日志] 成功加载 {len(changes)} 个文件的文件树节点（含修改和未修改文件）。")
    return changes, True, start_commit

def get_mock_changes() -> List[Dict[str, Any]]:
    """提供高逼真度的演示变更数据，覆盖所有状态和提交记录"""
    return [
        {
            "path": "src/main.py", "status": "modified", "added": 120, "deleted": 45, "old_path": None,
            "commits": [
                {"sha": "a2b3c4d", "author": "Alice", "date": "2小时前", "message": "优化核心逻辑算法性能"},
                {"sha": "f8e9d1c", "author": "Bob", "date": "3天前", "message": "修复主控制循环的内存泄漏问题"}
            ],
            "diff": [
                {"type": "hunk", "content": "@@ -120,7 +120,12 @@"},
                {"type": "context", "content": " def process_data(data):"},
                {"type": "context", "content": "     result = []"},
                {"type": "context", "content": "     for item in data:"},
                {"type": "removed", "content": "-        result.append(item * 2)"},
                {"type": "added", "content": "+        transformed = transform_item(item)"},
                {"type": "added", "content": "+        if transformed is not None:"},
                {"type": "added", "content": "+            result.append(transformed)"},
                {"type": "context", "content": "     return result"},
                {"type": "hunk", "content": "@@ -200,8 +200,6 @@"},
                {"type": "context", "content": " def cleanup():"},
                {"type": "removed", "content": "-    cache.clear()"},
                {"type": "removed", "content": "-    logger.flush()"},
                {"type": "context", "content": "     gc.collect()"},
                {"type": "context", "content": "     reset_state()"},
            ]
        },
        {
            "path": "src/utils/helpers.py", "status": "added", "added": 85, "deleted": 0, "old_path": None,
            "commits": [
                {"sha": "7b8c9d0", "author": "Alice", "date": "1天前", "message": "新增通用助手工具方法集"}
            ],
            "diff": [
                {"type": "hunk", "content": "@@ -0,0 +1,10 @@"},
                {"type": "added", "content": "+import os"},
                {"type": "added", "content": "+import json"},
                {"type": "added", "content": "+"},
                {"type": "added", "content": "+def format_date(dt):"},
                {"type": "added", "content": "+    return dt.strftime('%Y-%m-%d %H:%M:%S')"},
                {"type": "added", "content": "+"},
                {"type": "added", "content": "+def merge_dicts(a, b):"},
                {"type": "added", "content": "+    result = a.copy()"},
                {"type": "added", "content": "+    result.update(b)"},
                {"type": "added", "content": "+    return result"},
            ]
        },
        {
            "path": "src/utils/old_parser.py", "status": "deleted", "added": 0, "deleted": 150, "old_path": None,
            "commits": [],
            "diff": [
                {"type": "hunk", "content": "@@ -1,30 +0,0 @@"},
                {"type": "removed", "content": "-import re"},
                {"type": "removed", "content": "-import xml.etree.ElementTree as ET"},
                {"type": "removed", "content": "-"},
                {"type": "removed", "content": "-class OldXMLParser:"},
                {"type": "removed", "content": "-    def parse(self, content):"},
                {"type": "removed", "content": "-        return ET.fromstring(content)"},
                {"type": "removed", "content": "-"},
                {"type": "removed", "content": "-    def extract_text(self, elem):"},
                {"type": "removed", "content": "-        return elem.text or ''"},
            ]
        },
        {
            "path": "src/network/client.py", "status": "renamed", "added": 30, "deleted": 10, "old_path": "src/net_client.py",
            "commits": [
                {"sha": "1a2b3c4", "author": "Charlie", "date": "5小时前", "message": "整理目录结构，重构网络客户端到 network 目录下"}
            ],
            "diff": [
                {"type": "hunk", "content": "@@ -1,4 +1,4 @@"},
                {"type": "removed", "content": "-# net_client.py"},
                {"type": "added", "content": "+# client.py - Network Client"},
                {"type": "context", "content": " import socket"},
                {"type": "context", "content": " import ssl"},
                {"type": "hunk", "content": "@@ -50,8 +50,12 @@ class NetworkClient:"},
                {"type": "context", "content": "         self.host = host"},
                {"type": "context", "content": "         self.port = port"},
                {"type": "removed", "content": "-        self.timeout = 30"},
                {"type": "added", "content": "+        self.timeout = 60"},
                {"type": "added", "content": "+        self.retry_count = 3"},
                {"type": "added", "content": "+        self._connected = False"},
                {"type": "context", "content": " "},
                {"type": "context", "content": "     def connect(self):"},
                {"type": "added", "content": "+        for i in range(self.retry_count):"},
            ]
        },
        {
            "path": "tests/test_main.py", "status": "modified", "added": 30, "deleted": 5, "old_path": None,
            "commits": [
                {"sha": "4e5f6g7", "author": "Bob", "date": "2天前", "message": "补充主逻辑测试用例，覆盖边缘情况"}
            ],
            "diff": [
                {"type": "hunk", "content": "@@ -30,6 +30,12 @@ def test_process_data():"},
                {"type": "context", "content": "     result = process_data([1, 2, 3])"},
                {"type": "context", "content": "     assert len(result) == 3"},
                {"type": "added", "content": "+"},
                {"type": "added", "content": "+def test_process_data_empty():"},
                {"type": "added", "content": "+    result = process_data([])"},
                {"type": "added", "content": "+    assert result == []"},
                {"type": "added", "content": "+"},
                {"type": "added", "content": "+def test_process_data_none():"},
                {"type": "added", "content": "+    with pytest.raises(ValueError):"},
                {"type": "added", "content": "+        process_data(None)"},
            ]
        },
        {
            "path": "docs/README.md", "status": "modified", "added": 15, "deleted": 2, "old_path": None,
            "commits": [
                {"sha": "9h8i7j6", "author": "Alice", "date": "4天前", "message": "更新架构图说明文档"}
            ],
            "diff": [
                {"type": "hunk", "content": "@@ -10,7 +10,7 @@"},
                {"type": "context", "content": " ## Architecture"},
                {"type": "removed", "content": "-![Old Architecture](docs/old_arch.png)"},
                {"type": "added", "content": "+![New Architecture](docs/new_arch.png)"},
                {"type": "hunk", "content": "@@ -45,2 +45,5 @@"},
                {"type": "context", "content": " ## Quick Start"},
                {"type": "added", "content": "+"},
                {"type": "added", "content": "+### Prerequisites"},
                {"type": "added", "content": "+- Python 3.8+"},
                {"type": "added", "content": "+- Node.js 16+"},
            ]
        },
        {
            "path": "docs/architecture.pdf", "status": "added", "added": 0, "deleted": 0, "old_path": None,
            "commits": [],
            "diff": [{"type": "hunk", "content": "[Binary file — diff not available]"}]
        },
        {
            "path": "config/settings.json", "status": "unchanged", "added": 0, "deleted": 0, "old_path": None,
            "commits": [],
            "diff": []
        },
        {
            "path": "config/database.yml", "status": "unchanged", "added": 0, "deleted": 0, "old_path": None,
            "commits": [],
            "diff": []
        },
        {
            "path": "requirements.txt", "status": "modified", "added": 4, "deleted": 1, "old_path": None,
            "commits": [
                {"sha": "3k2l1m0", "author": "Charlie", "date": "1周前", "message": "升级 requests 和 PyYAML 依赖版本"}
            ],
            "diff": [
                {"type": "hunk", "content": "@@ -1,7 +1,10 @@"},
                {"type": "removed", "content": "-requests==2.28.0"},
                {"type": "added", "content": "+requests==2.31.0"},
                {"type": "removed", "content": "-PyYAML==6.0"},
                {"type": "added", "content": "+PyYAML==6.1"},
                {"type": "added", "content": "+pandas==2.0.0"},
                {"type": "added", "content": "+numpy==1.24.0"},
            ]
        },
        {
            "path": "legacy_run.sh", "status": "deleted", "added": 0, "deleted": 20, "old_path": None,
            "commits": [],
            "diff": [
                {"type": "hunk", "content": "@@ -1,5 +0,0 @@"},
                {"type": "removed", "content": "-#!/bin/bash"},
                {"type": "removed", "content": "-echo 'Starting legacy service...'"},
                {"type": "removed", "content": "-cd /opt/legacy"},
                {"type": "removed", "content": "-python run.py"},
                {"type": "removed", "content": "-"},
            ]
        }
    ]

# ==========================================
# 2. HTML + JS 交互界面模板
# ==========================================

HTML_TEMPLATE = """<!DOCTYPE html>
<html lang="zh-CN" class="dark">
<head>
    <meta charset="UTF-8">
    <meta name="viewport" content="width=device-width, initial-scale=1.0">
    <title>Git 历史变更交互式仪表盘</title>
    <!-- Tailwind CSS CDN -->
    <script src="https://cdn.tailwindcss.com"></script>
    <script>
        tailwind.config = {
            darkMode: 'class',
            theme: {
                extend: {
                    colors: {
                        slate: {
                            950: '#0b0f19',
                        }
                    }
                }
            }
        }
    </script>
    <style>
        ::-webkit-scrollbar {
            width: 8px;
            height: 8px;
        }
        ::-webkit-scrollbar-track {
            background: #0f172a;
        }
        ::-webkit-scrollbar-thumb {
            background: #334155;
            border-radius: 4px;
        }
        ::-webkit-scrollbar-thumb:hover {
            background: #475569;
        }
        .custom-blur {
            backdrop-filter: blur(8px);
        }
        /* Diff 视图样式 */
        .diff-line {
            display: flex;
            padding: 0 8px;
            line-height: 1.6;
            min-height: 22px;
            white-space: pre-wrap;
            word-break: break-all;
            font-family: 'Cascadia Code', 'Fira Code', 'JetBrains Mono', 'Consolas', monospace;
            font-size: 12px;
        }
        .diff-marker {
            display: inline-block;
            width: 18px;
            flex-shrink: 0;
            text-align: center;
            user-select: none;
        }
        .diff-text {
            flex: 1;
        }
        .diff-added {
            background-color: rgba(16, 185, 129, 0.12);
            border-left: 3px solid rgb(16, 185, 129);
        }
        .diff-added .diff-marker {
            color: rgb(16, 185, 129);
        }
        .diff-added .diff-text {
            color: rgb(110, 231, 183);
        }
        .diff-removed {
            background-color: rgba(239, 68, 68, 0.12);
            border-left: 3px solid rgb(239, 68, 68);
        }
        .diff-removed .diff-marker {
            color: rgb(239, 68, 68);
        }
        .diff-removed .diff-text {
            color: rgb(252, 165, 165);
        }
        .diff-context {
            background-color: transparent;
        }
        .diff-context .diff-text {
            color: rgb(148, 163, 184);
        }
        .diff-hunk {
            background-color: rgba(59, 130, 246, 0.08);
            border-left: 3px solid rgb(59, 130, 246);
            padding-top: 6px;
            padding-bottom: 6px;
            margin-top: 4px;
        }
        .diff-hunk .diff-text {
            color: rgb(96, 165, 250);
            font-weight: 600;
        }
    </style>
</head>
<body class="bg-slate-950 text-slate-100 min-h-screen font-sans antialiased">

    <!-- 顶部主条 -->
    <header class="border-b border-slate-800 bg-slate-900/50 backdrop-blur sticky top-0 z-40">
        <div class="max-w-7xl mx-auto px-4 py-4 flex flex-col md:flex-row md:items-center md:justify-between gap-4">
            <div>
                <div class="flex items-center gap-2">
                    <span class="px-2.5 py-0.5 rounded-full text-xs font-semibold bg-blue-500/10 text-blue-400 border border-blue-500/20" id="mode-badge">
                        加载中...
                    </span>
                    <h1 class="text-xl font-extrabold tracking-tight bg-gradient-to-r from-blue-400 via-indigo-400 to-emerald-400 bg-clip-text text-transparent">
                        Git 变更交互式文件树
                    </h1>
                </div>
                <p class="text-xs text-slate-400 mt-1" id="commit-range-text">对比范围: --</p>
            </div>
            
            <!-- 头部操作：全部展开/收起 -->
            <div class="flex items-center gap-2">
                <button onclick="expandAllDirs()" class="px-3 py-1.5 bg-slate-800 hover:bg-slate-700 text-xs text-slate-200 font-medium rounded-lg transition border border-slate-700">
                    📂 全部展开
                </button>
                <button onclick="collapseAllDirs()" class="px-3 py-1.5 bg-slate-800 hover:bg-slate-700 text-xs text-slate-200 font-medium rounded-lg transition border border-slate-700">
                    📁 全部收起
                </button>
            </div>
        </div>
    </header>

    <main class="max-w-7xl mx-auto px-4 py-6">
        
        <!-- 全局统计面板 -->
        <div class="grid grid-cols-2 md:grid-cols-5 gap-4 mb-6">
            <div class="bg-slate-900/80 border border-slate-800 rounded-xl p-4 shadow-xl">
                <p class="text-xs font-semibold text-slate-400">新增文件</p>
                <p class="text-2xl font-bold text-emerald-400 mt-1" id="stat-added">0</p>
            </div>
            <div class="bg-slate-900/80 border border-slate-800 rounded-xl p-4 shadow-xl">
                <p class="text-xs font-semibold text-slate-400">删除文件</p>
                <p class="text-2xl font-bold text-rose-400 mt-1" id="stat-deleted">0</p>
            </div>
            <div class="bg-slate-900/80 border border-slate-800 rounded-xl p-4 shadow-xl">
                <p class="text-xs font-semibold text-slate-400">修改文件</p>
                <p class="text-2xl font-bold text-blue-400 mt-1" id="stat-modified">0</p>
            </div>
            <div class="bg-slate-900/80 border border-slate-800 rounded-xl p-4 shadow-xl">
                <p class="text-xs font-semibold text-slate-400">移动/重命名</p>
                <p class="text-2xl font-bold text-amber-400 mt-1" id="stat-renamed">0</p>
            </div>
            <div class="bg-slate-900/80 border border-slate-800 rounded-xl p-4 shadow-xl col-span-2 md:col-span-1">
                <p class="text-xs font-semibold text-slate-400">未修改文件</p>
                <p class="text-2xl font-bold text-slate-400 mt-1" id="stat-unchanged">0</p>
            </div>
        </div>

        <!-- 聚合吞吐量卡片 -->
        <div class="bg-gradient-to-r from-emerald-950/20 to-rose-950/20 border border-slate-800/80 rounded-xl p-4 mb-6 flex flex-wrap justify-between items-center gap-4">
            <div class="flex items-center gap-3">
                <div class="p-2 bg-slate-800/80 rounded-lg text-lg">📊</div>
                <div>
                    <h3 class="text-sm font-bold text-slate-200">行数吞吐量统计</h3>
                    <p class="text-xs text-slate-400">仅针对对比区间内检测到增减行数的文件</p>
                </div>
            </div>
            <div class="flex gap-6">
                <div>
                    <span class="text-xs text-emerald-400 font-bold block">++ 累计增加行数</span>
                    <span class="text-xl font-extrabold text-emerald-400" id="stat-lines-added">0</span>
                </div>
                <div class="border-l border-slate-800 h-10 my-auto"></div>
                <div>
                    <span class="text-xs text-rose-400 font-bold block">-- 累计删除行数</span>
                    <span class="text-xl font-extrabold text-rose-400" id="stat-lines-deleted">0</span>
                </div>
            </div>
        </div>

        <!-- 过滤与搜索 -->
        <div class="bg-slate-900/50 border border-slate-800/60 rounded-xl p-4 mb-6 flex flex-col md:flex-row gap-4 items-center justify-between">
            <!-- 检索框 -->
            <div class="relative w-full md:w-80">
                <span class="absolute inset-y-0 left-3 flex items-center text-slate-500">🔍</span>
                <input type="text" id="search-input" oninput="handleSearch()" placeholder="实时搜索文件名或路径..." 
                       class="w-full pl-9 pr-4 py-2 bg-slate-950 border border-slate-800 rounded-lg text-sm text-slate-100 placeholder-slate-500 focus:outline-none focus:ring-2 focus:ring-blue-500/50 focus:border-blue-500 transition">
            </div>

            <!-- 过滤标签卡 -->
            <div class="flex flex-wrap gap-2 w-full md:w-auto">
                <button onclick="filterTree('all')" id="btn-filter-all" class="filter-btn px-3 py-1.5 rounded-lg text-xs font-semibold transition bg-blue-500 text-white shadow-md shadow-blue-500/20">
                    全部
                </button>
                <button onclick="filterTree('added')" id="btn-filter-added" class="filter-btn px-3 py-1.5 rounded-lg text-xs font-semibold transition bg-slate-800 hover:bg-slate-700 text-emerald-400">
                    仅新增
                </button>
                <button onclick="filterTree('deleted')" id="btn-filter-deleted" class="filter-btn px-3 py-1.5 rounded-lg text-xs font-semibold transition bg-slate-800 hover:bg-slate-700 text-rose-400">
                    仅删除
                </button>
                <button onclick="filterTree('modified')" id="btn-filter-modified" class="filter-btn px-3 py-1.5 rounded-lg text-xs font-semibold transition bg-slate-800 hover:bg-slate-700 text-blue-400">
                    仅修改
                </button>
                <button onclick="filterTree('renamed')" id="btn-filter-renamed" class="filter-btn px-3 py-1.5 rounded-lg text-xs font-semibold transition bg-slate-800 hover:bg-slate-700 text-amber-400">
                    仅移动
                </button>
                <button onclick="filterTree('unchanged')" id="btn-filter-unchanged" class="filter-btn px-3 py-1.5 rounded-lg text-xs font-semibold transition bg-slate-800 hover:bg-slate-700 text-slate-400">
                    仅未修改
                </button>
            </div>
        </div>

        <!-- 主结构树状图展示区 -->
        <div class="bg-slate-900/80 border border-slate-800 rounded-xl shadow-2xl p-6 overflow-hidden">
            <div class="text-xs font-bold text-slate-500 uppercase tracking-wider mb-4 pb-2 border-b border-slate-800 flex justify-between">
                <span>📁 项目目录拓扑结构</span>
                <span id="nodes-count-text" class="text-slate-400">共 0 个节点</span>
            </div>
            
            <!-- 动态生成的树状容器 -->
            <div id="tree-container" class="space-y-1 font-mono text-sm overflow-x-auto">
                <!-- 节点由此处渲染 -->
            </div>
        </div>
    </main>

    <!-- 弹窗模态窗 (文件修改记录历史) -->
    <div id="modal-backdrop" class="fixed inset-0 bg-slate-950/80 backdrop-blur-sm z-50 hidden transition-all duration-300 flex items-center justify-center p-4">
        <div id="modal-card" class="bg-slate-900 border border-slate-800 w-full max-w-2xl rounded-2xl shadow-2xl flex flex-col max-h-[85vh] scale-95 opacity-0 transition-all duration-300">
            <!-- 弹窗头部 -->
            <div class="p-5 border-b border-slate-800 flex justify-between items-start">
                <div>
                    <div class="flex items-center gap-2">
                        <span id="modal-file-badge" class="px-2 py-0.5 rounded text-xs font-bold"></span>
                        <h3 id="modal-file-name" class="text-base font-bold text-slate-100"></h3>
                    </div>
                    <p id="modal-file-path" class="text-xs text-slate-400 mt-1 break-all"></p>
                    <p id="modal-old-path" class="text-xs text-amber-400 mt-1 break-all hidden"></p>
                </div>
                <button onclick="closeModal()" class="text-slate-400 hover:text-slate-200 text-xl p-1 bg-slate-800/50 hover:bg-slate-800 rounded-lg transition">
                    ✕
                </button>
            </div>
            
            <!-- 弹窗内容 -->
            <div class="p-6 overflow-y-auto space-y-6">
                <!-- 行数可视化 -->
                <div>
                    <h4 class="text-xs font-bold text-slate-400 uppercase tracking-wider mb-3">⚡ 代码行数吞吐</h4>
                    <div class="grid grid-cols-2 gap-4">
                        <div class="bg-emerald-950/20 border border-emerald-900/50 rounded-xl p-3 text-center">
                            <span class="text-xs text-emerald-400 font-medium">增加行数</span>
                            <p id="modal-lines-added" class="text-xl font-black text-emerald-400 mt-1">0</p>
                        </div>
                        <div class="bg-rose-950/20 border border-rose-900/50 rounded-xl p-3 text-center">
                            <span class="text-xs text-rose-400 font-medium">删除行数</span>
                            <p id="modal-lines-deleted" class="text-xl font-black text-rose-400 mt-1">0</p>
                        </div>
                    </div>
                </div>

                <!-- 选项卡：提交历史 / 代码变更 -->
                <div>
                    <div class="flex border-b border-slate-800 mb-4">
                        <button id="tab-commits" class="tab-btn px-4 py-2 text-xs font-bold border-b-2 border-blue-500 text-blue-400 transition"
                                onclick="switchModalTab('commits')">
                            ⏳ 提交历史
                        </button>
                        <button id="tab-diff" class="tab-btn px-4 py-2 text-xs font-bold border-b-2 border-transparent text-slate-400 hover:text-slate-200 transition"
                                onclick="switchModalTab('diff')">
                            📝 代码变更
                        </button>
                    </div>

                    <!-- 提交历史面板 -->
                    <div id="panel-commits">
                        <h4 class="text-xs font-bold text-slate-400 uppercase tracking-wider mb-4">⏳ 对比区间内修改历史 (最近5次)</h4>
                        <div id="modal-commits-container" class="space-y-4">
                            <!-- Commit 列表 -->
                        </div>
                    </div>

                    <!-- 代码变更面板 -->
                    <div id="panel-diff" class="hidden">
                        <h4 class="text-xs font-bold text-slate-400 uppercase tracking-wider mb-4">📝 逐行代码变更</h4>
                        <div id="modal-diff-container" class="font-mono text-xs leading-relaxed overflow-x-auto">
                            <!-- Diff 行 -->
                        </div>
                    </div>
                </div>
            </div>

            <!-- 弹窗脚部 -->
            <div class="p-4 border-t border-slate-800 bg-slate-950/50 flex justify-end">
                <button onclick="closeModal()" class="px-4 py-2 bg-slate-800 hover:bg-slate-700 text-xs font-bold rounded-lg transition text-slate-200">
                    关闭
                </button>
            </div>
        </div>
    </div>

    <!-- 数据源嵌入区 -->
    <script id="git-data" type="application/json">
        __GIT_DATA_JSON__
    </script>

    <!-- 交互逻辑控制 JS 脚本 -->
    <script>
        // 读取 Python 注入的 Git 变化数据
        const rawData = JSON.parse(document.getElementById('git-data').textContent);
        
        let currentFilter = 'all';
        let searchQuery = '';
        
        // 渲染基础参数
        document.getElementById('mode-badge').innerText = rawData.is_real_git ? '真实 Git 模式' : '演示模式';
        document.getElementById('commit-range-text').innerText = `对比起点 Commit: ${rawData.start_commit.substring(0, 7)} ➜ 当前最新 (HEAD)`;
        
        // 设置仪表盘统计值
        document.getElementById('stat-added').innerText = rawData.summary.files_added;
        document.getElementById('stat-deleted').innerText = rawData.summary.files_deleted;
        document.getElementById('stat-modified').innerText = rawData.summary.files_modified;
        document.getElementById('stat-renamed').innerText = rawData.summary.files_renamed;
        document.getElementById('stat-unchanged').innerText = rawData.summary.files_unchanged;
        document.getElementById('stat-lines-added').innerText = `+${rawData.summary.total_added}`;
        document.getElementById('stat-lines-deleted').innerText = `-${rawData.summary.total_deleted}`;

        // ----------------------------------------------------
        // 树状图构建逻辑
        // ----------------------------------------------------
        function getFileEmoji(filename, isDir) {
            if (isDir) return "📁";
            const name = filename.toLowerCase();
            if (name.endsWith('.py') || name.endsWith('.pyw')) return "🐍";
            if (name.endsWith('.js') || name.endsWith('.ts') || name.endsWith('.jsx') || name.endsWith('.tsx')) return "⚙️";
            if (name.endsWith('.json') || name.endsWith('.yml') || name.endsWith('.yaml')) return "⚙️";
            if (name.endsWith('.md') || name.endsWith('.txt')) return "📄";
            if (name.endsWith('.png') || name.endsWith('.jpg') || name.endsWith('.jpeg') || name.endsWith('.gif') || name.endsWith('.svg') || name.endsWith('.pdf')) return "🎨";
            if (name.endsWith('.sh') || name.endsWith('.bat') || name.endsWith('.cmd')) return "🐚";
            if (name === "requirements.txt" || name === "package.json") return "📦";
            return "📄";
        }

        // 构建文件夹嵌套树结构
        function buildTreeData(files) {
            const root = { name: "root", is_dir: true, children: {}, files: [] };
            
            files.forEach(file => {
                const parts = file.path.split('/');
                let current = root;
                
                for (let i = 0; i < parts.length - 1; i++) {
                    const dirName = parts[i];
                    if (!current.children[dirName]) {
                        current.children[dirName] = {
                            name: dirName,
                            is_dir: true,
                            children: {},
                            files: [],
                            stats: { added: 0, deleted: 0, files_count: 0 }
                        };
                    }
                    current = current.children[dirName];
                }
                
                const fileName = parts[parts.length - 1];
                current.files.push({
                    ...file,
                    name: fileName
                });
            });
            
            return root;
        }

        const rootTree = buildTreeData(rawData.files);

        // 自底向上统计各个目录下的汇总数
        function computeDirectoryStats(node) {
            let added = 0;
            let deleted = 0;
            let count = 0;
            
            node.files.forEach(f => {
                added += f.added;
                deleted += f.deleted;
                count++;
            });
            
            Object.values(node.children).forEach(child => {
                const childStats = computeDirectoryStats(child);
                added += childStats.added;
                deleted += childStats.deleted;
                count += childStats.count;
            });
            
            if (node.name !== "root") {
                node.stats = { added, deleted, count };
            }
            return { added, deleted, count };
        }
        computeDirectoryStats(rootTree);

        // ----------------------------------------------------
        // 渲染 HTML 树
        // ----------------------------------------------------
        let dirExpandedState = {}; // 保存各个目录路径的展开收起状态，默认全展开
        let renderedNodesCount = 0;

        function getDirKey(pathParts) {
            return pathParts.join('/');
        }

        function isNodeVisible(file) {
            // 搜索过滤
            if (searchQuery) {
                const searchLower = searchQuery.toLowerCase();
                const matchesName = file.name.toLowerCase().includes(searchLower) || file.path.toLowerCase().includes(searchLower);
                if (!matchesName) return false;
            }
            // 状态过滤
            if (currentFilter !== 'all') {
                if (file.status !== currentFilter) return false;
            }
            return true;
        }

        // 检查目录内是否含有可见子节点
        function hasVisibleChildren(node, pathParts) {
            // 检查当前目录下是否有满足条件的文件
            const matchedFile = node.files.some(f => isNodeVisible(f));
            if (matchedFile) return true;
            
            // 递归检查子目录
            return Object.values(node.children).some(child => {
                return hasVisibleChildren(child, [...pathParts, child.name]);
            });
        }

        function renderTree(node, depth = 0, pathParts = [], parentHasNext = []) {
            let htmlStr = '';
            
            // 排序：先展示子目录，再展示子文件
            const sortedDirs = Object.keys(node.children).sort();
            const sortedFiles = node.files.sort((a, b) => a.name.localeCompare(b.name));
            
            // 1. 渲染子目录
            sortedDirs.forEach((dirName, index) => {
                const childNode = node.children[dirName];
                const currentParts = [...pathParts, dirName];
                const dirKey = getDirKey(currentParts);
                const encodedDirKey = encodeURIComponent(dirKey).replace(/'/g, '%27');
                
                // 默认初始化文件夹为展开
                if (dirExpandedState[dirKey] === undefined) {
                    dirExpandedState[dirKey] = true; 
                }
                
                // 如果当前文件夹及所有子代都没有符合过滤/搜索条件的文件，则不渲染该文件夹
                if (!hasVisibleChildren(childNode, currentParts)) return;
                
                const isExpanded = dirExpandedState[dirKey];
                const hasNext = index < (sortedDirs.length + sortedFiles.length - 1);
                const nextAncestors = [...parentHasNext, hasNext];
                
                renderedNodesCount++;
                
                htmlStr += `<div class="tree-node-dir">`;
                // 渲染行
                htmlStr += `
                    <div class="group flex items-center justify-between py-1 px-2 hover:bg-slate-800/40 rounded-lg cursor-pointer transition select-none" 
                         onclick='toggleDir("${encodedDirKey}", event)'>
                        <div class="flex items-center min-w-0">
                            <!-- 连线骨架 -->
                            ${renderLines(depth, parentHasNext)}
                            <!-- 折线连线 -->
                            <span class="text-slate-600 mr-1.5">${hasNext ? '├─' : '└─'}</span>
                            <!-- 折叠指示器 -->
                            <span class="text-xs text-slate-400 mr-1.5 transform transition-transform duration-200 ${isExpanded ? 'rotate-90' : ''}">▶</span>
                            <!-- 图标 -->
                            <span class="text-base mr-2">${getFileEmoji(dirName, true)}</span>
                            <!-- 目录名字 -->
                            <span class="font-bold text-slate-300 truncate">${safeHtml(dirName)}</span>
                            <!-- 子节点计数 -->
                            <span class="ml-1.5 px-1.5 py-0.2 bg-slate-800 rounded text-[10px] text-slate-400 border border-slate-700/60 font-sans">${childNode.stats.count}个文件</span>
                        </div>
                        <div class="flex items-center gap-2 text-xs font-mono shrink-0">
                            ${childNode.stats.added > 0 ? `<span class="text-emerald-500 font-semibold">+${childNode.stats.added}</span>` : ''}
                            ${childNode.stats.deleted > 0 ? `<span class="text-rose-500 font-semibold">-${childNode.stats.deleted}</span>` : ''}
                        </div>
                    </div>
                `;
                
                // 渲染内容子节点
                if (isExpanded) {
                    htmlStr += `<div class="tree-dir-content">`;
                    htmlStr += renderTree(childNode, depth + 1, currentParts, nextAncestors);
                    htmlStr += `</div>`;
                }
                htmlStr += `</div>`;
            });
            
            // 2. 渲染文件
            sortedFiles.forEach((file, index) => {
                if (!isNodeVisible(file)) return;
                
                const hasNext = index < (sortedFiles.length - 1);
                renderedNodesCount++;
                
                // 根据状态赋予定制的高亮背景色彩和边框样式
                let statusBgClass = "hover:bg-slate-800/40";
                let nameClass = "text-slate-100";
                let badgeClass = "";
                let badgeText = "";
                
                if (file.status === 'added') {
                    statusBgClass = "bg-emerald-950/20 hover:bg-emerald-950/30 border-l-4 border-emerald-500 pl-1";
                    nameClass = "text-emerald-400 font-bold";
                    badgeClass = "bg-emerald-500/10 text-emerald-400 border border-emerald-500/20";
                    badgeText = "新增";
                } else if (file.status === 'deleted') {
                    statusBgClass = "bg-rose-950/20 hover:bg-rose-950/30 border-l-4 border-rose-500 pl-1 line-through";
                    nameClass = "text-slate-400";
                    badgeClass = "bg-rose-500/10 text-rose-400 border border-rose-500/20";
                    badgeText = "删除";
                } else if (file.status === 'modified') {
                    statusBgClass = "border-l-4 border-blue-500 pl-1 hover:bg-slate-800/40";
                    nameClass = "text-blue-400 font-bold";
                    badgeClass = "bg-blue-500/10 text-blue-400 border border-blue-500/20";
                    badgeText = "修改";
                } else if (file.status === 'renamed') {
                    statusBgClass = "bg-amber-950/20 hover:bg-amber-950/30 border-l-4 border-amber-500 pl-1";
                    nameClass = "text-amber-400 font-bold";
                    badgeClass = "bg-amber-500/10 text-amber-400 border border-amber-500/20";
                    badgeText = "移动";
                } else if (file.status === 'unchanged') {
                    nameClass = "text-slate-400";
                    badgeClass = "bg-slate-800 text-slate-500 border border-slate-700/50";
                    badgeText = "未修改";
                }

                // 数据绑定为 HTML data 属性便于读取
                const fileJSON = encodeURIComponent(JSON.stringify(file)).replace(/'/g, '%27');
                
                htmlStr += `
                    <div class="group flex items-center justify-between py-1 px-2 ${statusBgClass} rounded-lg cursor-pointer transition select-none"
                         onclick='openFileModal("${fileJSON}")'>
                        <div class="flex items-center min-w-0">
                            <!-- 连线骨架 -->
                            ${renderLines(depth, parentHasNext)}
                            <!-- 折线连线 -->
                            <span class="text-slate-600 mr-1.5">${hasNext ? '├─' : '└─'}</span>
                            <!-- 文件图标 -->
                            <span class="text-base mr-2 shrink-0">${getFileEmoji(file.name, false)}</span>
                            <!-- 文件名称 -->
                            <span class="${nameClass} truncate">${safeHtml(file.name)}</span>
                            <!-- 重命名附加原路径提示 -->
                            ${file.old_path ? `<span class="ml-2 text-[11px] text-slate-500 italic truncate">(原路径: ${safeHtml(file.old_path)})</span>` : ''}
                            
                            <!-- 状态标签 -->
                            <span class="ml-2 px-1.5 py-0.2 rounded text-[10px] uppercase tracking-wide shrink-0 ${badgeClass}">${badgeText}</span>
                        </div>
                        
                        <!-- 增减行数指标 -->
                        <div class="flex items-center gap-2 text-xs font-mono shrink-0">
                            ${file.added > 0 ? `<span class="text-emerald-500 font-bold">+${file.added}</span>` : ''}
                            ${file.deleted > 0 ? `<span class="text-rose-500 font-bold">-${file.deleted}</span>` : ''}
                            ${file.status !== 'unchanged' && file.commits && file.commits.length > 0 ? `<span class="text-[10px] bg-slate-800 text-slate-400 px-1 py-0.2 rounded opacity-0 group-hover:opacity-100 transition-opacity">查看日志</span>` : ''}
                        </div>
                    </div>
                `;
            });
            
            return htmlStr;
        }

        // 绘制辅助线结构
        function renderLines(depth, parentHasNext) {
            let linesHtml = '';
            for (let i = 0; i < depth; i++) {
                if (parentHasNext[i]) {
                    linesHtml += `<span class="text-slate-700 w-4 inline-block text-center border-r border-slate-800/80 mr-1">│</span>`;
                } else {
                    linesHtml += `<span class="w-4 inline-block mr-1"></span>`;
                }
            }
            return linesHtml;
        }

        function toggleDir(encodedDirKey, event) {
            event.stopPropagation();
            const dirKey = decodeURIComponent(encodedDirKey);
            dirExpandedState[dirKey] = !dirExpandedState[dirKey];
            triggerRender();
        }

        function expandAllDirs() {
            Object.keys(dirExpandedState).forEach(k => dirExpandedState[k] = true);
            triggerRender();
        }

        function collapseAllDirs() {
            Object.keys(dirExpandedState).forEach(k => dirExpandedState[k] = false);
            triggerRender();
        }

        // 触发动态渲染更新
        function triggerRender() {
            renderedNodesCount = 0;
            const container = document.getElementById('tree-container');
            const htmlContent = renderTree(rootTree);
            container.innerHTML = htmlContent || `<div class="text-slate-500 text-center py-10">⚠️ 没有匹配到任何文件</div>`;
            document.getElementById('nodes-count-text').innerText = `已加载并显示 ${renderedNodesCount} 个文件/目录节点`;
        }

        // ----------------------------------------------------
        // 搜索与过滤交互
        // ----------------------------------------------------
        function filterTree(filterType) {
            currentFilter = filterType;
            
            // 重置过滤按钮高亮
            document.querySelectorAll('.filter-btn').forEach(btn => {
                btn.className = "filter-btn px-3 py-1.5 rounded-lg text-xs font-semibold transition bg-slate-800 hover:bg-slate-700 text-slate-300";
            });
            
            const activeBtn = document.getElementById(`btn-filter-${filterType}`);
            if (activeBtn) {
                if (filterType === 'added') activeBtn.className = "filter-btn px-3 py-1.5 rounded-lg text-xs font-semibold transition bg-emerald-500 text-white shadow-md shadow-emerald-500/20";
                else if (filterType === 'deleted') activeBtn.className = "filter-btn px-3 py-1.5 rounded-lg text-xs font-semibold transition bg-rose-500 text-white shadow-md shadow-rose-500/20";
                else if (filterType === 'modified') activeBtn.className = "filter-btn px-3 py-1.5 rounded-lg text-xs font-semibold transition bg-blue-500 text-white shadow-md shadow-blue-500/20";
                else if (filterType === 'renamed') activeBtn.className = "filter-btn px-3 py-1.5 rounded-lg text-xs font-semibold transition bg-amber-500 text-white shadow-md shadow-amber-500/20";
                else if (filterType === 'unchanged') activeBtn.className = "filter-btn px-3 py-1.5 rounded-lg text-xs font-semibold transition bg-slate-400 text-slate-950 shadow-md shadow-slate-400/20";
                else activeBtn.className = "filter-btn px-3 py-1.5 rounded-lg text-xs font-semibold transition bg-blue-500 text-white shadow-md shadow-blue-500/20";
            }
            
            // 过滤时为了可见性自动展开所有
            expandAllDirs();
        }

        function handleSearch() {
            searchQuery = document.getElementById('search-input').value;
            // 搜索时为了可见性自动展开所有
            expandAllDirs();
        }

        // ----------------------------------------------------
        // 弹窗逻辑控制器
        // ----------------------------------------------------
        function openFileModal(fileJSONStr) {
            const file = JSON.parse(decodeURIComponent(fileJSONStr));
            
            // 文件基础信息填入
            document.getElementById('modal-file-name').innerText = file.name;
            document.getElementById('modal-file-path').innerText = file.path;
            
            const oldPathEl = document.getElementById('modal-old-path');
            if (file.old_path) {
                oldPathEl.innerText = `原路径: ${file.old_path}`;
                oldPathEl.classList.remove('hidden');
            } else {
                oldPathEl.classList.add('hidden');
            }
            
            // 行数
            document.getElementById('modal-lines-added').innerText = `+${file.added}`;
            document.getElementById('modal-lines-deleted').innerText = `-${file.deleted}`;
            
            // 状态标签
            const badge = document.getElementById('modal-file-badge');
            badge.innerText = file.status.toUpperCase();
            if (file.status === 'added') {
                badge.className = "px-2 py-0.5 rounded text-xs font-bold bg-emerald-500/20 text-emerald-400 border border-emerald-500/30";
                badge.innerText = "新增";
            } else if (file.status === 'deleted') {
                badge.className = "px-2 py-0.5 rounded text-xs font-bold bg-rose-500/20 text-rose-400 border border-rose-500/30";
                badge.innerText = "删除";
            } else if (file.status === 'modified') {
                badge.className = "px-2 py-0.5 rounded text-xs font-bold bg-blue-500/20 text-blue-400 border border-blue-500/30";
                badge.innerText = "修改";
            } else if (file.status === 'renamed') {
                badge.className = "px-2 py-0.5 rounded text-xs font-bold bg-amber-500/20 text-amber-400 border border-amber-500/30";
                badge.innerText = "移动";
            } else {
                badge.className = "px-2 py-0.5 rounded text-xs font-bold bg-slate-800 text-slate-400 border border-slate-700/50";
                badge.innerText = "未修改";
            }

            // 渲染 Commit 列表
            const commitsContainer = document.getElementById('modal-commits-container');
            commitsContainer.innerHTML = '';
            
            if (file.status === 'unchanged') {
                commitsContainer.innerHTML = `
                    <div class="text-xs text-slate-500 italic p-4 text-center border border-dashed border-slate-800 rounded-xl">
                        此文件在本次对比区间内没有进行过任何代码提交。
                    </div>
                `;
            } else if (!file.commits || file.commits.length === 0) {
                commitsContainer.innerHTML = `
                    <div class="text-xs text-slate-500 italic p-4 text-center border border-dashed border-slate-800 rounded-xl">
                        暂未获取到此文件的专项提交明细（可能属于未提交修改或被直接删除的文件）。
                    </div>
                `;
            } else {
                file.commits.forEach(commit => {
                    commitsContainer.innerHTML += `
                        <div class="flex gap-3 relative group">
                            <!-- 时间轴圆点线 -->
                            <div class="flex flex-col items-center">
                                <div class="w-2 h-2 rounded-full bg-blue-500 mt-1.5 shadow-lg shadow-blue-500/50"></div>
                                <div class="w-0.5 flex-1 bg-slate-800 my-1"></div>
                            </div>
                            <div class="flex-1 bg-slate-950/60 border border-slate-800 hover:border-slate-700 rounded-xl p-3 transition">
                                <div class="flex justify-between items-center gap-2">
                                    <div class="flex items-center gap-2">
                                        <span class="text-xs font-black text-blue-400 bg-blue-950/30 px-1.5 py-0.5 rounded border border-blue-900/30 font-mono">${commit.sha}</span>
                                        <span class="text-xs font-semibold text-slate-200">${commit.author}</span>
                                    </div>
                                    <span class="text-[10px] text-slate-500">${commit.date}</span>
                                </div>
                                <p class="text-xs text-slate-300 mt-2 leading-relaxed">${commit.message}</p>
                            </div>
                        </div>
                    `;
                });
            }

            // 渲染 Diff 面板
            renderDiff(file);

            // 默认显示提交历史 tab
            switchModalTab('commits');

            // 显示 Modal
            const backdrop = document.getElementById('modal-backdrop');
            const card = document.getElementById('modal-card');
            
            backdrop.classList.remove('hidden');
            setTimeout(() => {
                card.classList.remove('scale-95', 'opacity-0');
                card.classList.add('scale-100', 'opacity-100');
            }, 10);
        }

        function closeModal() {
            const backdrop = document.getElementById('modal-backdrop');
            const card = document.getElementById('modal-card');

            card.classList.remove('scale-100', 'opacity-100');
            card.classList.add('scale-95', 'opacity-0');
            setTimeout(() => {
                backdrop.classList.add('hidden');
            }, 200);
        }

        // ----------------------------------------------------
        // Tab 切换 & Diff 渲染
        // ----------------------------------------------------
        function switchModalTab(tab) {
            const panelCommits = document.getElementById('panel-commits');
            const panelDiff = document.getElementById('panel-diff');
            const tabCommits = document.getElementById('tab-commits');
            const tabDiff = document.getElementById('tab-diff');

            if (tab === 'diff') {
                panelCommits.classList.add('hidden');
                panelDiff.classList.remove('hidden');
                tabCommits.className = 'tab-btn px-4 py-2 text-xs font-bold border-b-2 border-transparent text-slate-400 hover:text-slate-200 transition';
                tabDiff.className = 'tab-btn px-4 py-2 text-xs font-bold border-b-2 border-blue-500 text-blue-400 transition';
            } else {
                panelDiff.classList.add('hidden');
                panelCommits.classList.remove('hidden');
                tabDiff.className = 'tab-btn px-4 py-2 text-xs font-bold border-b-2 border-transparent text-slate-400 hover:text-slate-200 transition';
                tabCommits.className = 'tab-btn px-4 py-2 text-xs font-bold border-b-2 border-blue-500 text-blue-400 transition';
            }
        }

        function renderDiff(file) {
            const container = document.getElementById('modal-diff-container');
            container.innerHTML = '';

            const diff = file.diff;
            if (!diff || diff.length === 0) {
                container.innerHTML = `
                    <div class="text-xs text-slate-500 italic p-4 text-center border border-dashed border-slate-800 rounded-xl">
                        此文件没有可查看的代码变更详情。
                    </div>
                `;
                return;
            }

            let htmlArr = [];
            for (const line of diff) {
                const content = escapeHtml(line.content);
                switch (line.type) {
                    case 'added':
                        htmlArr.push(`<div class="diff-line diff-added"><span class="diff-marker">+</span><span class="diff-text">${content.substring(1)}</span></div>`);
                        break;
                    case 'removed':
                        htmlArr.push(`<div class="diff-line diff-removed"><span class="diff-marker">-</span><span class="diff-text">${content.substring(1)}</span></div>`);
                        break;
                    case 'hunk':
                        htmlArr.push(`<div class="diff-line diff-hunk"><span class="diff-marker"> </span><span class="diff-text">${content}</span></div>`);
                        break;
                    default:
                        htmlArr.push(`<div class="diff-line diff-context"><span class="diff-marker"> </span><span class="diff-text">${content}</span></div>`);
                        break;
                }
            }
            container.innerHTML = htmlArr.join('');
        }

        function escapeHtml(str) {
            const div = document.createElement('div');
            div.textContent = str;
            // 除了 HTML 编码，还要转义反引号和 ${}，防止破坏 JS 模板字面量
            return div.innerHTML.replace(/`/g, '&#96;').replace(/\\$\\{/g, '&#36;&#123;');
        }
        // 安全插值：在 JS 模板字面量中使用用户数据时转义反引号和 ${
        function safeHtml(str) {
            return String(str).replace(/`/g, '&#96;').replace(/\\$\\{/g, '&#36;&#123;');
        }

        // 点击空白处关闭
        document.getElementById('modal-backdrop').addEventListener('click', function(e) {
            if (e.target === this) {
                closeModal();
            }
        });

        // 默认初始化运行一次渲染
        triggerRender();
    </script>
</body>
</html>
"""

# ==========================================
# 3. 主流程与写出控制
# ==========================================

def calculate_summary(changes: List[Dict[str, Any]]) -> Dict[str, Any]:
    """汇总各个文件的变更统计指标"""
    summary = {
        "files_added": 0,
        "files_deleted": 0,
        "files_modified": 0,
        "files_renamed": 0,
        "files_unchanged": 0,
        "total_added": 0,
        "total_deleted": 0
    }
    
    for item in changes:
        status = item["status"]
        if status == "added":
            summary["files_added"] += 1
        elif status == "deleted":
            summary["files_deleted"] += 1
        elif status == "modified":
            summary["files_modified"] += 1
        elif status == "renamed":
            summary["files_renamed"] += 1
        elif status == "unchanged":
            summary["files_unchanged"] += 1
            
        summary["total_added"] += item["added"]
        summary["total_deleted"] += item["deleted"]
        
    return summary

def main():
    print("=" * 60)
    print("      Git 变更交互式文件树 & 仪表盘生成器")
    print("=" * 60)
    
    # 解析命令行参数
    parser = argparse.ArgumentParser(description="分析 Git 提交变更并渲染为交互式网页。")
    parser.add_argument(
        "-s", "--start", 
        type=str, 
        default=None, 
        help="指定开始对比的 Commit Hash、分支名（如 main）或 Tag。若不提供，默认从首次提交开始对比。"
    )
    parser.add_argument(
        "-o", "--output", 
        type=str, 
        default="git_diff_tree.html", 
        help="指定生成的 HTML 仪表盘文件的路径及名称 (默认: git_diff_tree.html)"
    )
    args = parser.parse_args()
    
    # 1. 提取所有变更数据（含未修改文件和文件详细 commit 日志）
    changes, is_real_git, start_commit = get_git_changes(args.start)
    
    # 2. 统计计算总体概览
    total_summary = calculate_summary(changes)
    
    # 3. 组装 JSON 负载
    payload = {
        "start_commit": start_commit,
        "end_commit": "HEAD",
        "is_real_git": is_real_git,
        "summary": total_summary,
        "files": changes
    }
    
    json_data = json.dumps(payload, ensure_ascii=False, indent=2).replace("<", "\\u003c")
    
    # 4. 替换 HTML 模板中的占位符
    html_content = HTML_TEMPLATE.replace("__GIT_DATA_JSON__", json_data)
    
    # 5. 写出交互网页
    try:
        with open(args.output, "w", encoding="utf-8") as f:
            f.write(html_content)
        print(f"[成功] 交互式仪表盘网页已成功生成在: {os.path.abspath(args.output)}")
        print(f"[提示] 请直接双击或在浏览器中打开 [{args.output}] 查看极致的树状交互效果！")
    except Exception as e:
        print(f"[错误] 保存 HTML 失败: {e}", file=sys.stderr)
        
    print("=" * 60)

if __name__ == "__main__":
    main()