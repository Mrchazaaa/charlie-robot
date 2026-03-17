INCLUDE(../../../cmake/macros.cmake)

add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/src/third_party/ABS)

set(CMAKE_CXX_STANDARD 17)

SET(ROBOT_NAME "absspeeddreamsdriver")

SET(ROBOT_INTERFACE LEGACY WELCOME)

SET(ROBOT_SOURCES
    src/ClothoidPath.cpp
    src/ClothoidPath.h
    src/cubic.cpp
    src/cubic.h
    src/absspeeddreamsdriver.cpp
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
)

# Official-only absspeeddreamsdriver instances.
# SET(ROBOT_CLONES absspeeddreamsdriver_36GP absspeeddreamsdriver_67GP absspeeddreamsdriver_lp1 absspeeddreamsdriver_ls1 absspeeddreamsdriver_ls2 absspeeddreamsdriver_mpa1
#                  absspeeddreamsdriver_mpa11 absspeeddreamsdriver_mpa12 absspeeddreamsdriver_rs absspeeddreamsdriver_sc absspeeddreamsdriver_srw absspeeddreamsdriver_trb1)

# The ubiquitous robot module and its clones.
ROBOT_MODULE(NAME ${ROBOT_NAME} VERSION 2.0.0 SOVERSION 1.0.0
             INTERFACE ${ROBOT_INTERFACE}
             SOURCES ${ROBOT_SOURCES}
             CLONENAMES ${ROBOT_CLONES})

target_link_libraries(${ROBOT_NAME} PRIVATE ABS)

# For data associated to the ubiquitous robot module and its clones,
# see data/drivers/absspeeddreamsdriver*/CMakeLists.txt
