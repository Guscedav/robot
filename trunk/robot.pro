QT -= gui

TEMPLATE = lib
DEFINES += ROBOT_LIBRARY

CONFIG += c++11

# The following define makes your compiler emit warnings if you use
# any Qt feature that has been marked deprecated (the exact warnings
# depend on your compiler). Please consult the documentation of the
# deprecated API in order to know how to port your code away from it.
DEFINES += QT_DEPRECATED_WARNINGS

# You can also make your code fail to compile if it uses deprecated APIs.
# In order to do so, uncomment the following line.
# You can also select to disable deprecated APIs only up to a certain version of Qt.
#DEFINES += QT_DISABLE_DEPRECATED_BEFORE=0x060000    # disables all the APIs deprecated before Qt 6.0.0

INCLUDEPATH += include/ ../../iio/trunk/include/

SOURCES += \
    src/Matrix.cpp \
    src/Motion.cpp \
    src/Quaternion.cpp \
    src/RobotController.cpp \
    src/RobotModel.cpp \
    src/Task.cpp \
    src/TaskMoveJoint.cpp \
    src/TaskMoveLinear.cpp \
    src/TaskMoveToolWithJoystick.cpp \
    src/TaskSequencer.cpp \
    src/TaskSetDO.cpp \
    src/TaskStop.cpp \
    src/TaskWait.cpp \
    src/TaskWaitDI.cpp \
    src/Tool.cpp \
    src/Transformation.cpp \
    src/Vector.cpp

HEADERS += \
    include/Matrix.h \
    include/Motion.h \
    include/Quaternion.h \
    include/RobotController.h \
    include/RobotModel.h \
    include/Task.h \
    include/TaskMoveJoint.h \
    include/TaskMoveLinear.h \
    include/TaskMoveToolWithJoystick.h \
    include/TaskSequencer.h \
    include/TaskSetDO.h \
    include/TaskStop.h \
    include/TaskWait.h \
    include/TaskWaitDI.h \
    include/Tool.h \
    include/Transformation.h \
    include/Vector.h

# Default rules for deployment.
unix {
    target.path = /usr/lib
}
!isEmpty(target.path): INSTALLS += target
