![alt text](./charlierobot.png)
# charlierobot
A driver for Speed Dreams 2 implementing the open source Anti-Lock Braking System I developed as part of my dissertation.

## Installing Speed Dreams 2
You might not need to build from source if you just want to work on CharlieRobot and dont need access to Speed Dreams source files.

## Building Speed Dreams 2 from Source Files
To further develop CharlieRobot you might find it useful to build Speed Dreams from source to allow access to 

Source files are available via http://www.speed-dreams.org/#download, incuding installation instructions for Linux, Windows and MacOs https://sourceforge.net/p/speed-dreams/code/HEAD/tree/trunk/INSTALL.txt.

### Linux 
Whilst building from source on a Linux machine, I've found the following guide written by Roman M. Yagodin very helpful http://roman-yagodin.github.io/compile-guide/2016/03/30/build-speed-dreams-linux.

You might run into errors concerning missing header files for the PLIB Library, the installation of which is outlined helpfully at http://www.berniw.org/tutorials/robot/torcs/install/plib-install.html as part of a wider guide outlining driver creation for TORCS (a driving simulator from which Speed Dreams descended).

### Windows 
1.

## Installing Charlie Robot
1. Copy <strong>{CHARLIE-ROBOT-DIR}/src/charlie-robot</strong> into <strong>{SPEED-DREAMS-DIR}/src/drivers</strong>
2. In <strong>{SPEED-DREAMS-DIR}/src/drivers/CMakeLists.txt</strong> add <strong>"SD_ADD_SUBDIRECTORY(charlie-robot)"</strong>
3. Copy <strong>{CHARLIE-ROBOT-DIR}/data/charlierobot</strong> into <strong>{SPEED-DREAMS-DIR}/data/drivers</strong>
4. In <strong>{SPEED-DREAMS-DIR}/data/CMakeLists.txt</strong> add <strong>"SD_ADD_SUBDIRECTORY(charlierobot)"</strong>
3. cd "speed-dreams/build" run "cmake .." and then "make" (possibly followed by "make install")

## Basic Usage
