#!/bin/sh
echo "win build:"
cmake --build ./build/win -j 16
cmake --install ./build/win --prefix ./build/win/install # --strip
echo "linux build:"
cmake --build ./build/linux -j 16
cmake --install ./build/linux --prefix ./build/linux/install # --strip