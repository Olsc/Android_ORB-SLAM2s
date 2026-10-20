#!/usr/bin/env bash
# Copyright (C) 2026 Olsc <OlscStudio@outlook.com>
# SPDX-License-Identifier: Apache-2.0
#
# 将 .gitmodules 中的每个子模块更新到它所跟踪分支的最新提交。
#
# 跟踪分支由 .gitmodules 里的 submodule.<name>.branch 决定；
# 若未配置，则使用子模块远程仓库的默认分支（remote HEAD）。
#
# 用法:
#   docs/update_submodules.sh [选项]
#
# 选项:
#   --filter <list>      仅更新列表中的子模块，逗号/空格分隔。
#                        可以填子模块路径、名称或目录名；留空或 "all" 表示全部。
#   --depth <n>          fetch/克隆的深度。默认不限制（完整历史）；
#                        传 1 可浅克隆，节省磁盘与时间（CI 中推荐）。
#   --summary-file <f>   把适合作为 PR 描述的 Markdown 汇总写入文件 <f>。
#   --paths-file <f>     把发生变化的子模块路径（每行一个）写入文件 <f>。
#   --github-output <f>  向 GitHub Actions 的 $GITHUB_OUTPUT 文件追加
#                        changed / paths 输出。
#   --dry-run            只检测是否有更新，不修改工作区。
#   -h, --help           显示帮助。
#
# 退出码: 0 正常；非 0 表示初始化/更新/网络等错误。

set -euo pipefail

usage() {
    sed -n '2,40p' "$0" | sed 's/^# \{0,1\}//'
}

REPO_ROOT="$(git rev-parse --show-toplevel 2>/dev/null || true)"
if [[ -z "$REPO_ROOT" ]]; then
    echo "error: 当前目录不在 git 仓库中" >&2
    exit 1
fi
cd "$REPO_ROOT"

FILTER=""
DEPTH=""
SUMMARY_FILE=""
PATHS_FILE=""
GH_OUTPUT_FILE=""
DRY_RUN="false"

while [[ $# -gt 0 ]]; do
    case "$1" in
        --filter)        FILTER="${2:-}"; shift 2 ;;
        --depth)         DEPTH="${2:-}"; shift 2 ;;
        --summary-file)  SUMMARY_FILE="${2:-}"; shift 2 ;;
        --paths-file)    PATHS_FILE="${2:-}"; shift 2 ;;
        --github-output) GH_OUTPUT_FILE="${2:-}"; shift 2 ;;
        --dry-run)       DRY_RUN="true"; shift ;;
        -h|--help)       usage; exit 0 ;;
        *) echo "error: 未知选项 '$1'" >&2; usage >&2; exit 2 ;;
    esac
done

# ---------------------------------------------------------------------------
# 收集子模块: path -> name 映射（name 即 .gitmodules 中的 submodule key）
# ---------------------------------------------------------------------------
declare -a ALL_PATHS=()
declare -A NAME_BY_PATH=()

while IFS= read -r line; do
    [[ -z "$line" ]] && continue
    key="${line%% *}"
    path="${line#* }"
    name="${key#submodule.}"
    name="${name%.path}"
    ALL_PATHS+=("$path")
    NAME_BY_PATH["$path"]="$name"
done < <(git config -f .gitmodules --get-regexp '^submodule\..*\.path$' || true)

if [[ ${#ALL_PATHS[@]} -eq 0 ]]; then
    echo "没有在 .gitmodules 中找到任何子模块"
    if [[ -n "$GH_OUTPUT_FILE" ]]; then
        echo "changed=false" >> "$GH_OUTPUT_FILE"
        echo "paths=" >> "$GH_OUTPUT_FILE"
    fi
    exit 0
fi

# 判断某个子模块是否被 --filter 选中
matches_filter() {
    local path="$1" name="$2" token normalized
    if [[ -z "$FILTER" || "$FILTER" == "all" ]]; then
        return 0
    fi
    normalized="${FILTER//,/ }"
    for token in $normalized; do
        [[ "$token" == "$path" ]] && return 0
        [[ "$token" == "$name" ]] && return 0
        [[ "$token" == "$(basename "$path")" ]] && return 0
    done
    return 1
}

# 把普通 GitHub 远程地址转换为网页地址，用于生成可点击链接
to_web_url() {
    local url="$1"
    case "$url" in
        https://github.com/*) echo "${url%.git}" ;;
        http://github.com/*)  echo "https://${url#http://}" | sed 's/\.git$//' ;;
        git@github.com:*)     echo "https://github.com/${url#git@github.com:}" | sed 's/\.git$//' ;;
        git://github.com/*)   echo "https://github.com/${url#git://github.com/}" | sed 's/\.git$//' ;;
        *) echo "" ;;
    esac
}

# ---------------------------------------------------------------------------
# 主循环
# ---------------------------------------------------------------------------
# 禁用自动 GC 和后台维护任务，防止浅克隆（--depth）下后台 gc 与前台 fetch
# 并发修改 .git/shallow 引发 "fatal: shallow file has changed since we read it"
export GIT_CONFIG_COUNT=3
export GIT_CONFIG_KEY_0="gc.auto"
export GIT_CONFIG_VALUE_0="0"
export GIT_CONFIG_KEY_1="gc.autodetach"
export GIT_CONFIG_VALUE_1="false"
export GIT_CONFIG_KEY_2="maintenance.auto"
export GIT_CONFIG_VALUE_2="false"

declare -a CHANGED=()
declare -a UNCHANGED=()
declare -A ROW_BY_PATH=()

echo "== 同步子模块远程配置 =="
git submodule sync --quiet || true

for path in "${ALL_PATHS[@]}"; do
    name="${NAME_BY_PATH[$path]}"

    if ! matches_filter "$path" "$name"; then
        echo "跳过  $path（未通过 --filter）"
        continue
    fi

    branch="$(git config -f .gitmodules --get "submodule.${name}.branch" || true)"
    if [[ -z "$branch" ]]; then
        branch="(remote HEAD)"
    fi
    remote_url="$(git config -f .gitmodules --get "submodule.${name}.url" || true)"

    # 更新前的基准提交：默认取父仓库索引里记录的 gitlink。
    # 注意：子模块未初始化时目录是个空目录，`git -C "$path" rev-parse HEAD`
    # 会向上找到父仓库并返回父仓库的 HEAD（错误值），所以必须先判断
    # `$path/.git` 是否存在，只有真正初始化了才用工作区 HEAD。
    before_sha="$(git rev-parse "HEAD:$path" 2>/dev/null || echo '')"
    if [[ -e "$path/.git" ]]; then
        worktree_sha="$(git -C "$path" rev-parse HEAD 2>/dev/null || echo '')"
        if [[ -n "$worktree_sha" ]]; then
            before_sha="$worktree_sha"
        fi
    fi

    # 要拉取的引用：配置了 branch 就用它，否则用远端默认分支(HEAD)
    fetch_ref="$branch"
    if [[ "$branch" == "(remote HEAD)" ]]; then
        fetch_ref="HEAD"
    fi

    fetch_opts=(--quiet)
    if [[ -n "$DEPTH" && "$DEPTH" != "0" ]]; then
        fetch_opts+=(--depth="$DEPTH")
    fi

    if [[ "$DRY_RUN" == "true" ]]; then
        if [[ ! -e "$path/.git" ]]; then
            echo "跳过  $path（未初始化，--dry-run 不进行克隆）"
            continue
        fi
        git -C "$path" fetch "${fetch_opts[@]}" origin "$fetch_ref" >/dev/null 2>&1 || true
        after_sha="$(git -C "$path" rev-parse FETCH_HEAD 2>/dev/null || echo "$before_sha")"
    else
        echo "更新  $path -> origin/$branch"
        # 1) 先把子模块初始化到父仓库记录的提交（浅克隆行为与 actions/checkout 一致）
        sub_args=(update --init --checkout)
        if [[ -n "$DEPTH" && "$DEPTH" != "0" ]]; then
            sub_args+=(--depth="$DEPTH")
        fi
        git submodule "${sub_args[@]}" -- "$path"
        # 2) 再显式 fetch 跟踪分支并检出。
        #    不使用 `git submodule update --remote`：当子模块默认分支与配置的
        #    branch 不同时，--depth 生成的是“单分支”浅克隆，里面没有
        #    refs/remotes/origin/<branch>，--remote 会报
        #    "Unable to find refs/remotes/origin/<branch> revision" 并以 128 退出。
        #    带重试与清理浅克隆锁机制，防止极端情况下文件锁竞争。
        fetch_success="false"
        for attempt in 1 2 3; do
            if git -C "$path" fetch "${fetch_opts[@]}" origin "$fetch_ref"; then
                fetch_success="true"
                break
            fi
            echo "  fetch 失败 (第 $attempt 次尝试)，清理锁并重试..." >&2
            git_dir="$(git -C "$path" rev-parse --git-dir 2>/dev/null || true)"
            if [[ -n "$git_dir" && -f "$git_dir/shallow.lock" ]]; then
                rm -f "$git_dir/shallow.lock"
            fi
            sleep 1
        done
        if [[ "$fetch_success" != "true" ]]; then
            echo "error: fetch $path 失败" >&2
            exit 1
        fi
        git -C "$path" checkout --quiet --detach FETCH_HEAD
        after_sha="$(git -C "$path" rev-parse HEAD 2>/dev/null || echo '')"
    fi

    if [[ -n "$before_sha" && "$before_sha" != "$after_sha" ]]; then
        CHANGED+=("$path")
        if [[ "$DRY_RUN" != "true" ]]; then
            # 暂存子模块 gitlink，便于查看 diff / 提交
            git add -- "$path"
        fi
        web="$(to_web_url "$remote_url")"
        old_cell="\`${before_sha:0:9}\`"
        new_cell="\`${after_sha:0:9}\`"
        diff_cell="—"
        if [[ -n "$web" ]]; then
            old_cell="[\`${before_sha:0:9}\`](${web}/commit/${before_sha})"
            new_cell="[\`${after_sha:0:9}\`](${web}/commit/${after_sha})"
            diff_cell="[对比](${web}/compare/${before_sha}...${after_sha})"
        fi
        ROW_BY_PATH["$path"]="| \`${path}\` | \`${branch}\` | ${old_cell} | ${new_cell} | ${diff_cell} |"
        echo "  变更  ${before_sha:0:9} -> ${after_sha:0:9}"
    else
        UNCHANGED+=("$path")
        echo "  已是最新（${after_sha:0:9}）"
    fi
done

# ---------------------------------------------------------------------------
# 生成 PR 描述
# ---------------------------------------------------------------------------
if [[ -n "$SUMMARY_FILE" && ${#CHANGED[@]} -gt 0 ]]; then
    server="${GITHUB_SERVER_URL:-https://github.com}"
    repo="${GITHUB_REPOSITORY:-}"
    ref="${BASE_BRANCH:-${GITHUB_REF_NAME:-main}}"
    workflow_link="${server}/${repo}/blob/${ref}/.github/workflows/submodule-auto-update.yml"

    {
        echo "## 子模块自动更新"
        echo
        echo "> 本 PR 由 [\`submodule-auto-update\`](${workflow_link}) 工作流定期自动生成。"
        echo
        echo "| 子模块 | 跟踪分支 | 旧提交 | 新提交 | 变更 |"
        echo "| :--- | :--- | :--- | :--- | :--- |"
        for path in "${CHANGED[@]}"; do
            echo "${ROW_BY_PATH[$path]}"
        done
        echo
        if [[ ${#UNCHANGED[@]} -gt 0 ]]; then
            echo "<details><summary>已检查但无更新（${#UNCHANGED[@]}）</summary>"
            echo
            for path in "${UNCHANGED[@]}"; do
                echo "- \`${path}\`"
            done
            echo
            echo "</details>"
            echo
        fi
    } > "$SUMMARY_FILE"
fi

# ---------------------------------------------------------------------------
# 输出结果
# ---------------------------------------------------------------------------
changed="false"
if [[ ${#CHANGED[@]} -gt 0 ]]; then
    changed="true"
fi

if [[ -n "$PATHS_FILE" ]]; then
    : > "$PATHS_FILE"
    for path in "${CHANGED[@]}"; do
        echo "$path" >> "$PATHS_FILE"
    done
fi

if [[ -n "$GH_OUTPUT_FILE" ]]; then
    echo "changed=$changed" >> "$GH_OUTPUT_FILE"
    {
        echo "paths<<__SUBMODULE_PATHS_EOF__"
        for path in "${CHANGED[@]}"; do
            echo "$path"
        done
        echo "__SUBMODULE_PATHS_EOF__"
    } >> "$GH_OUTPUT_FILE"
fi

echo
echo "== 结果 =="
echo "发生变化: ${#CHANGED[@]} 个；无更新: ${#UNCHANGED[@]} 个"
for path in "${CHANGED[@]}"; do
    echo "  * $path"
done

exit 0
