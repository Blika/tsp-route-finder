if not exist build_debug mkdir build_debug
cd build_debug
cmake -S ../ -B . -G "MinGW Makefiles" -DCMAKE_BUILD_TYPE=Debug
mingw32-make.exe
move debug__route_finder.exe ../
cd ..
pause