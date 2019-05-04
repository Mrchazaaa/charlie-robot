INCLUDE(../../../cmake/macros.cmake)

# INCLUDE(/src/drivers/charlierobot/src/velocity_estimator/eigen-git-mirror)
# INCLUDE(/src/drivers/charlierobot/src/velocity_estimator/autodiff)

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
    src/velocity_estimator/velocity_EKF.cpp
    src/velocity_estimator/velocity_EKF.h
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
