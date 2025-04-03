#!/bin/bash

#rm -rf ./build
mkdir build
mkdir build/win
mkdir build/linux
cmake -S ./ -B ./build/win -DCMAKE_TOOLCHAIN_FILE=./cmake/windows.cmake -D CMAKE_BUILD_TYPE=Release
cmake -S ./ -B ./build/linux -D CMAKE_BUILD_TYPE=Release
