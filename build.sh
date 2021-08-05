#!/bin/sh
sudo rm -rf ./out
mkdir -p out
make O=out ARCH=arm64 lineageos_judyln_defconfig

