QT       += core gui axcontainer printsupport serialport

greaterThan(QT_MAJOR_VERSION, 4): QT += widgets

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

SOURCES += \
    Uart_Connect.cpp \
    main.cpp \
    micro_imu_lib.c \
    qcustomplot.cpp \
    widget.cpp

HEADERS += \
    Uart_Connect.h \
    imu_api.h \
    micro_imu_lib.h \
    qcustomplot.h \
    widget.h

FORMS += \
    widget.ui

# Default rules for deployment.
qnx: target.path = /tmp/$${TARGET}/bin
else: unix:!android: target.path = /opt/$${TARGET}/bin
!isEmpty(target.path): INSTALLS += target

RESOURCES += \
    icon.qrc

# MinGW: qcustomplot.cpp 目标文件过大，必须开启 big-obj
QMAKE_CXXFLAGS += -Wa,-mbig-obj
QMAKE_CXXFLAGS_DEBUG += -Wa,-mbig-obj
QMAKE_CXXFLAGS_RELEASE += -Wa,-mbig-obj

# 更保险：只对 qcustomplot.cpp 追加（qmake 支持这种写法）
QMAKE_CXXFLAGS_qcustomplot.cpp += -Wa,-mbig-obj

DEFINES += DISABLE_FP_ASSERT
//RC_ICONS = Fire.ico
