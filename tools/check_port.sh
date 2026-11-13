#!/usr/bin/env bash
# Compile the target port's C files with the host GCC and the same strict warnings, so GCC-only
# warnings show up even when the ARM build uses clang (inline assembly is stubbed out on the host).
set -euo pipefail
cd "$(dirname "$0")/.."
FLAGS="-std=c99 -Wall -Wextra -Wpedantic -Wconversion -Wsign-conversion -Wshadow -Wdouble-promotion -Wcast-qual -Wstrict-prototypes -Wmissing-prototypes -Werror -fsyntax-only"
for f in port/stm32g431/hal_stm32g4.c port/stm32g431/mem.c; do
    gcc $FLAGS -fno-builtin -Iport/stm32g431/libc -Ifirmware/inc -Iport/stm32g431 "$f"
    echo "ok  $f"
done
