#!/usr/bin/env bash
# 用和 CI 完全相同的工具与版本格式化整个仓库。
#
#   scripts/format.sh          # 直接改文件
#   scripts/format.sh --check  # 只检查，有不合规的就返回 1（pre-commit / CI 用）
#
# 版本必须和 .github/workflows/build.yml 里钉的一致，否则本地改完推上去照样红。

set -euo pipefail

cd "$(dirname "${BASH_SOURCE[0]}")/.."

check=0
if [ "${1:-}" = "--check" ]; then
  check=1
fi

CLANG_FORMAT=${CLANG_FORMAT:-clang-format-18}
RUFF=${RUFF:-ruff}

need() {
  command -v "$1" >/dev/null 2>&1 || {
    echo "缺少 $1，请先安装：$2" >&2
    exit 127
  }
}

need "$CLANG_FORMAT" "sudo apt install clang-format-18"
need "$RUFF" "pipx install ruff==0.17.4"

cpp_files=$(git ls-files '*.h' '*.hpp' '*.c' '*.cc' '*.cpp' '*.cxx')
py_files=$(git ls-files '*.py')

if [ "$check" -eq 1 ]; then
  status=0

  for f in $cpp_files; do
    if ! "$CLANG_FORMAT" --dry-run --Werror "$f"; then
      status=1
    fi
  done

  if [ -n "$py_files" ]; then
    "$RUFF" format --check --diff $py_files || status=1
  fi

  exit $status
fi

[ -n "$cpp_files" ] && echo "$CLANG_FORMAT: $(echo "$cpp_files" | wc -l) 个文件" && \
  echo "$cpp_files" | xargs "$CLANG_FORMAT" -i
[ -n "$py_files" ] && echo "$RUFF: $(echo "$py_files" | wc -l) 个文件" && \
  echo "$py_files" | xargs "$RUFF" format

echo "格式化完成"
