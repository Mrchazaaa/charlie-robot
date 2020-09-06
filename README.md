![alt text](./charlierobot.png)
# charlierobot
A driver for Speed Dreams 2 implementing the open source Anti-Lock Braking System I developed as part of my dissertation.

## Installing Speed Dreams 2
You might not need to build from source if you just want to work on CharlieRobot and dont need access to Speed Dreams source files.

## Building Speed Dreams 2 from Source Files
To further develop CharlieRobot you might find it useful to build Speed Dreams from source to allow access to 

Source files are available via http://www.speed-dreams.org/#download, which incude useful installation instructions for Linux, Windows and MacOs https://sourceforge.net/p/speed-dreams/code/HEAD/tree/trunk/INSTALL.txt.

### Linux 
Whilst building from source on a Linux machine, I've found the following guide written by Roman M. Yagodin very helpful http://roman-yagodin.github.io/compile-guide/2016/03/30/build-speed-dreams-linux.

You might run into errors concerning missing header files for the PLIB Library, the installation of which is outlined helpfully at http://www.berniw.org/tutorials/robot/torcs/install/plib-install.html as part of a wider guide outlining driver creation for TORCS (a driving simulator from which Speed Dreams descended).


### Windows 
1.

## Installing Charlie Robot
1. in speed-dreams/src/drivers/charlie-robot/src/include/, clone the following repos:
git@github.com:eigenteam/eigen-git-mirror.git
git@github.com:autodiff/autodiff.git (needs to be at commit version 1cb0d9bde762f2e4b06c7cb053b6da822c6d45ca)
https://github.com/simondlevy/TinyEKF
2. In speed-dreams/src/drivers/CMakeLists.txt add "SD_ADD_SUBDIRECTORY(charlie-robot)"
3. We need to add some kind of speed-dreams/data/charlie-robot.xml file
4. cd "speed-dreams/build" run "cmake .." and then "make" (possibly followed by "make install")

## Basic Usage
