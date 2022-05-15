#!/bin/bash
PATH=`pwd`/aarch64-linux-android-4.9/bin:$PATH
mkdir -p out
make -j32 O=out ARCH=arm64 CROSS_COMPILE=aarch64-linux-android- sdm660_defconfig
make -j32 O=out ARCH=arm64 CROSS_COMPILE=aarch64-linux-android-
