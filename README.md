# ABS Speed Dreams Driver

A Speed Dreams 2 robot driver for testing [the open source Anti-Lock Braking System I developed as part of my Computer Science BSc dissertation]([https://charliehowlett.co.uk/ABSConstruction.pdf](https://github.com/mrchazaaa/anti-lock-braking-system)). The system aims to accurately model the commercial systems in use today by the general public. This was achieved by replicating model behavior as outlined in "Brakes, Brake control and driver Assistance Systems: Function, Regulation and Components" by K. Reif, where a typical ABS control loop is defined in its various stages (p. 82).

[View a copy of my dissertation here!](https://charliehowlett.co.uk/ABSConstruction.pdf)

<image style="float: right" src="./speeddreams.png" width="600px">

## Building Speed Dreams 2 from Source Files
To use you need to build Speed Dreams from source, steps for which are briefly outlined below.

Source files are available via http://www.speed-dreams.org/#download, incuding installation instructions for Linux, Windows and MacOs https://sourceforge.net/p/speed-dreams/code/HEAD/tree/trunk/INSTALL.txt.

### Linux
Whilst building from source on a Linux machine, I've found the following guide written by Roman M. Yagodin very helpful read alongside the source installation instructions http://roman-yagodin.github.io/compile-guide/2016/03/30/build-speed-dreams-linux.

You might run into errors concerning missing header files for the PLIB Library, the installation of which is outlined helpfully at http://www.berniw.org/tutorials/robot/torcs/install/plib-install.html as part of a wider guide outlining driver creation for TORCS (a driving simulator from which Speed Dreams descended).

## Installation
1. Clone this repository with submodules: <strong>`git clone --recurse-submodules ...`</strong> or, in an existing checkout, run <strong>`git submodule sync --recursive && git submodule update --init --recursive`</strong>.
2. Copy <strong>/src/abs-speed-dreams-driver</strong> into <strong>{SPEED-DREAMS-DIR}/src/drivers</strong>.
3. Copy <strong>/src/abs-speed-dreams-driver/src/third_party/ABS</strong> into <strong>{SPEED-DREAMS-DIR}/src/drivers/abs-speed-dreams-driver/src/third_party</strong>.
4. In <strong>{SPEED-DREAMS-DIR}/src/drivers/CMakeLists.txt</strong> add <strong>"SD_ADD_SUBDIRECTORY(abs-speed-dreams-driver)"</strong>.
5. Copy <strong>/data/absspeeddreamsdriver</strong> into <strong>{SPEED-DREAMS-DIR}/data/drivers</strong>.
6. In <strong>{SPEED-DREAMS-DIR}/data/CMakeLists.txt</strong> add <strong>"SD_ADD_SUBDIRECTORY(absspeeddreamsdriver)"</strong>.
7. cd "speed-dreams/build" run "cmake .." and then "make" (possibly followed by "make install").

## Basic Usage
Once installed:
1. start Speed Dreams and select Race -> Practice -> Configure
2. You will be given the option to select a race map. By default the driver will drive automatically until 140kmh is reached at which point an emergency brake will be initiated. To best support this, select a track with lots of straight road where the robot will be able to quickly accelerate to the required velocity.
3. Next driver options will be displayed. Select "ABS Speed Dreams Driver".
4. Finally, weather conditions can be configured allowing simultation of ABS braking in wet conditions.
5. Once configuration is done, you can use the existing setup on consequent practice runs. Simply hit "Start" and observe the braking sequence.
