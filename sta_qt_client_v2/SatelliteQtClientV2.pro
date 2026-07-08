QT += core gui network widgets
CONFIG += c++17 link_pkgconfig
PKGCONFIG += gstreamer-1.0 gstreamer-app-1.0 gstreamer-video-1.0

TARGET = SatelliteQtClientV2
TEMPLATE = app

INCLUDEPATH += src

SOURCES += \
    src/main.cpp \
    src/app/AppShell.cpp \
    src/network/TcpClientManager.cpp \
    src/pages/LoginPage.cpp \
    src/pages/ConnectPage.cpp \
    src/pages/DashboardPage.cpp \
    src/pages/DebugLogWindow.cpp \
    src/video/VideoReceiver.cpp \
    src/video/poseimagereceiver.cpp

HEADERS += \
    src/app/AppShell.h \
    src/app/AppState.h \
    src/common/Protocol.h \
    src/network/TcpClientManager.h \
    src/pages/LoginPage.h \
    src/pages/ConnectPage.h \
    src/pages/DashboardPage.h \
    src/pages/DebugLogWindow.h \
    src/video/VideoReceiver.h \
    src/video/poseimagereceiver.h
