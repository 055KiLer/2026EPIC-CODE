#!/usr/bin/env bash
# ============================================================================
#  台架调试工程 · 主机端编译校验（不用 Keil、不用板子）
#
#  为什么有它：真机构建在用户机的 Keil 上，本机（WSL/gcc）没有 Keil ——
#  改 debug_corexy 的 User 层代码时，唯一的即时把关手段就是这个脚本：
#  用 Tools/hostcheck/ 里的 HAL/CMSIS 替身做 gcc -fsyntax-only，
#  把"语法/类型/名字/宏拼写"这一层全都挡住（证明不了运行时行为，那归台架实测）。
#
#  用法（在 debug_corexy 目录下）：  bash Tools/hostcheck/check.sh
#  判定：最后一行出现「OK 0 错误 / 0 警告」才算过。
# ============================================================================
set -u
cd "$(dirname "$0")/../.." || exit 2

INC="-I Tools/hostcheck -I Config -I Core/Inc \
      -I User/Algorithm/Inc -I User/App/Inc -I User/Bsp/Inc -I User/Devices/Inc"
FLAGS="-fsyntax-only -std=gnu99 -Wall -Wextra -Wno-unused-parameter"

SRC="User/Algorithm/Src/*.c User/App/Src/*.c User/Bsp/Src/*.c User/Devices/Src/*.c"

out=$(gcc $FLAGS $INC $SRC 2>&1)
n_err=$(printf '%s' "$out" | grep -c ' error:')
n_warn=$(printf '%s' "$out" | grep -c ' warning:')

if [ "$n_err" != "0" ] || [ "$n_warn" != "0" ]; then
    printf '%s\n' "$out" | grep -E ' error:| warning:' | head -40
    echo "------------------------------------------------------------"
    echo "FAIL ${n_err} 错误 / ${n_warn} 警告（Core/Src 不在校验范围）"
    exit 1
fi

echo "OK 0 错误 / 0 警告（User 四层 13 个 .c 全过；Core/Src 真机才验）"