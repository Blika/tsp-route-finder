if not exist build mkdir build
cd build
cmake -S ../ -B . -G "MinGW Makefiles" -DCMAKE_BUILD_TYPE=Release
mingw32-make.exe
move route_finder.exe ../
cd ..
pause