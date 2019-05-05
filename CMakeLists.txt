INCLUDE(../../../cmake/macros.cmake)

#include_directories(src/include)
include_directories(src/include/TinyEKF/src)
include_directories(src/include/autodiff)
include_directories(src/include/eigen-git-mirror)

set(CMAKE_CXX_STANDARD 17)

SET(ROBOT_NAME "charlierobot")

SET(ROBOT_INTERFACE LEGACY WELCOME)

SET(ROBOT_SOURCES 
    src/ClothoidPath.cpp
    src/ClothoidPath.h
    src/cubic.cpp
    src/cubic.h
    src/charlierobot.cpp
    src/danpath.cpp
    src/danpath.h
    src/driver.cpp
    src/driver.h
    src/globaldefinitions.h
    src/LinePath.cpp
    src/LinePath.h
    src/MyTrack.cpp
    src/MyTrack.h
    src/opponent.cpp
    src/opponent.h
    src/pidcontroller.cpp
    src/pidcontroller.h
    src/pit.cpp
    src/pit.h
    src/Seg.h
    src/spline.cpp
    src/spline.h
    src/torcs_or_sd.h
    src/Utils.cpp
    src/Utils.h
    src/Vec2d.h
    src/Vec3d.h
    src/ABS.c
    src/ABS.h
    src/velocity_EKF.cpp
    src/velocity_EKF.h
    src/include/TinyEKF/src/tiny_ekf.c
    src/include/TinyEKF/src/tiny_ekf.h
    src/tinyekf_config.h
    #src/tiny_ekf.c
    #src/tiny_ekf.h
    src/include/TinyEKF/src/tiny_ekf_struct.h
    src/include/eigen-git-mirror/Eigen/LU
    src/include/eigen-git-mirror/Eigen/Core
)

# Official-only charlierobot instances.
# SET(ROBOT_CLONES charlierobot_36GP charlierobot_67GP charlierobot_lp1 charlierobot_ls1 charlierobot_ls2 charlierobot_mpa1
#                  charlierobot_mpa11 charlierobot_mpa12 charlierobot_rs charlierobot_sc charlierobot_srw charlierobot_trb1)

# The ubiquitous robot module and its clones.
ROBOT_MODULE(NAME ${ROBOT_NAME} VERSION 2.0.0 SOVERSION 1.0.0
             INTERFACE ${ROBOT_INTERFACE}
             SOURCES ${ROBOT_SOURCES}
             CLONENAMES ${ROBOT_CLONES})

# For data associated to the ubiquitous robot module and its clones,
# see data/drivers/charlierobot*/CMakeLists.txt
