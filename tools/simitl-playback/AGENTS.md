# quad copter simulation ghost playback tool

c++ cli tool for ghost.json playback

ghost.h contains json structure definition and parsing

main.cpp contains main playback loop and cli application

to build the whole project call ```./build.sh``` from ````cd ./../../`` nothing more! 

dont use tail command to check the build output, it is too long. use grep and filter for errors.

this is a linux and windows project and crossbuilds for both with mingw

for testing call ```../../build/linux/install/bin/simitl-playback```

never use literal UTF-8 characters in the source code, this breaks your ability to edit files!