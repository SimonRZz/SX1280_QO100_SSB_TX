#!/bin/sh
# Fetch the Si473x SSB patch and convert it into a plain-C header.
#
# WHY THIS IS NOT IN THE REPOSITORY
# The Si4732 cannot demodulate SSB on its own. About 15 KB of firmware has to be
# pushed into it over I2C after every power-up. That firmware is a Silicon Labs
# binary, published by Vadim Afonkin and carried by the PU2CLR SI4735 Arduino
# library, whose own header states the patch "is not part of this library" and
# discourages commercial use. This project is GPL and its repository is public,
# so the blob stays out of git and is fetched on demand instead.
#
# Run once before the first build:
#     sh tools/fetch_ssb_patch.sh
#
# The result, ssb_patch.h, is listed in .gitignore.

set -e

SRC="https://raw.githubusercontent.com/pu2clr/SI4735/master/src/patch_full.h"
DIR=$(CDPATH= cd -- "$(dirname -- "$0")/.." && pwd)
OUT="$DIR/ssb_patch.h"
TMP="$OUT.tmp"

if [ -f "$OUT" ]; then
    printf 'ssb_patch.h is already here (%s bytes). Delete it to re-fetch.\n' \
        "$(wc -c < "$OUT" | tr -d ' ')"
    exit 0
fi

echo "Fetching the SSB patch ..."
if command -v curl > /dev/null 2>&1; then
    curl -fsSL -o "$TMP" "$SRC"
elif command -v wget > /dev/null 2>&1; then
    wget -qO "$TMP" "$SRC"
else
    echo "Neither curl nor wget found." >&2
    exit 1
fi

# The upstream file is an Arduino header: it declares the array PROGMEM, which
# only means anything on AVR, and it states no size. Drop the attribute, give
# the array internal linkage and derive the size with sizeof.
sed 's/^const PROGMEM uint8_t ssb_patch_content\[\] =/static const uint8_t ssb_patch_content[] =/' \
    "$TMP" > "$OUT"

if ! grep -q 'static const uint8_t ssb_patch_content' "$OUT"; then
    echo "Upstream file no longer matches the expected declaration - not patched." >&2
    rm -f "$OUT" "$TMP"
    exit 1
fi

cat >> "$OUT" << 'EOF'

#define SSB_PATCH_SIZE (sizeof(ssb_patch_content))
EOF

rm -f "$TMP"

# The compiler never saw this header on an earlier build, so no dependency on it
# was recorded and make would happily skip si4732.c — leaving a binary without
# SSB and no hint that anything is missing. Force the rebuild.
touch "$DIR/si4732.c"

printf 'Wrote %s (%s bytes).\n' "$OUT" "$(wc -c < "$OUT" | tr -d ' ')"
echo 'si4732.c marked for rebuild. Now: cmake --build build'
