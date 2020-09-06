INCLUDE(../../../cmake/macros.cmake)

SET(ROBOT_NAME "charlierobot")

SET(ROBOT_SUBDIRS
    1 2 5 6 7 8 9 10 15 16
    tracks
    trb1-cavallo-360rb
    ls1-archer-r9
    ls1-cavallo-570s1
    ls1-vulture-v6r
    mpa1-hartbill-2002)

ROBOT_DATA(NAME ${ROBOT_NAME}
           FILES ${ROBOT_NAME}.xml default.xml logo.rgb
           SUBDIRS ${ROBOT_SUBDIRS} PATTERNS *.xml logo.rgb *.png *.jpg)
