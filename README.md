<image src="./charlierobot.png" width="200px">

# charlierobot

<image style="float: right" src="./speeddreams.png" width="600px">

A Speed Dreams 2 robot driver implementing the open source Anti-Lock Braking System I developed as part of my Computer Science BSc dissertation. The system aims to accurately model the commercial systems in wide use today by the general public, a difficult task owing to the highly sensitive nature of the source code belonging to automotive manufacturers. This was achieved by replicating model behavior as outlined in "Brakes, Brake control and driver Assistance Systems: Function, Regulation and Components" by K. Reif, where a typical ABS control loop is defined in its various stages (p. 82).

[View a copy of the dissertation here!](https://charliehowlett.com/ABSConstruction.pdf)

## Building Speed Dreams 2 from Source Files
To run CharlieRobot, you need to build Speed Dreams from source, steps for which are briefly outlined below.

Source files are available via http://www.speed-dreams.org/#download, incuding installation instructions for Linux, Windows and MacOs https://sourceforge.net/p/speed-dreams/code/HEAD/tree/trunk/INSTALL.txt.

### Linux 
Whilst building from source on a Linux machine, I've found the following guide written by Roman M. Yagodin very helpful read alongside the source installation instructions http://roman-yagodin.github.io/compile-guide/2016/03/30/build-speed-dreams-linux.

You might run into errors concerning missing header files for the PLIB Library, the installation of which is outlined helpfully at http://www.berniw.org/tutorials/robot/torcs/install/plib-install.html as part of a wider guide outlining driver creation for TORCS (a driving simulator from which Speed Dreams descended).

## Installing Charlie Robot
1. Copy <strong>{CHARLIE-ROBOT-DIR}/src/charlie-robot</strong> into <strong>{SPEED-DREAMS-DIR}/src/drivers</strong>
2. In <strong>{SPEED-DREAMS-DIR}/src/drivers/CMakeLists.txt</strong> add <strong>"SD_ADD_SUBDIRECTORY(charlie-robot)"</strong>
3. Copy <strong>{CHARLIE-ROBOT-DIR}/data/charlierobot</strong> into <strong>{SPEED-DREAMS-DIR}/data/drivers</strong>
4. In <strong>{SPEED-DREAMS-DIR}/data/CMakeLists.txt</strong> add <strong>"SD_ADD_SUBDIRECTORY(charlierobot)"</strong>
3. cd "speed-dreams/build" run "cmake .." and then "make" (possibly followed by "make install")

## Basic Usage
Once CharlieRobot is installed: 
1. start Speed Dreams and select Race -> Practice -> Configure
2. You will be given the option to select a race map, by default, CharlieRobot will drive automatically until 140kmh is reached at which point an emergency brake will be initiated. To best support this, select a track with lots of straight road where the robot will be able to quickly accelerate to the required velocity.
3. Next, you'll be displayed driver options. Select CharlieRobot.
4. Finally, weather conditions can be configured allowing simultation of ABS braking in wet conditions.
5. Once configuration is done, you can use the existing setup on consequent practice runs. Simply hit "Start" and observe the braking sequence.