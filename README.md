# build


```
cd build

cmake -S ../ -B ./ -DCMAKE_TOOLCHAIN_FILE=../cmake/windows.cmake -D CMAKE_BUILD_TYPE=Release

make -j 16
```
