#!/bin/sh
set -eu
repo_dir=$(CDPATH= cd -- "$(dirname -- "$0")/.." && pwd)
test_build=$(mktemp -d "${TMPDIR:-/tmp}/qo100-tuning-tests.XXXXXX")
trap 'rm -rf "$test_build"' EXIT HUP INT TERM
"${CC:-cc}" -std=c11 -Wall -Wextra -Werror -I"$repo_dir/tests/stubs" -I"$repo_dir" \
    "$repo_dir/tests/test_si4732_tuning.c" "$repo_dir/si4732.c" \
    -o "$test_build/test_si4732_tuning"
"$test_build/test_si4732_tuning"
