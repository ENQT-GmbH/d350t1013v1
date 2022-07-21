#!/bin/bash
set -e

source ../../toolchain-netztester2.sh
make ARCH=arm64 CROSS_COMPILE=aarch64-buildroot-linux-gnueabihf-

