#!/usr/bin/env bash
set -e

if [ ! -d "libyuv" ]; then
  git clone https://chromium.googlesource.com/libyuv/libyuv
fi

cd libyuv
git reset --hard 4a14cb2e81235ecd656e799aecaaf139db8ce4a2
cmake .
make

if [ -f /KA2 ]; then
  mv libyuv.a ../larch64/lib/
fi

## To create universal binary on Darwin:
## ```
## lipo -create -output Darwin/libyuv.a path-to-x64/libyuv.a path-to-arm64/libyuv.a
## ```
