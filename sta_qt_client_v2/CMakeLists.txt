cmake_minimum_required(VERSION 3.14)

project(SatelliteQtClientV2 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTOUIC ON)
set(CMAKE_AUTORCC ON)

find_package(QT NAMES Qt6 Qt5 REQUIRED COMPONENTS Widgets Network)
find_package(Qt${QT_VERSION_MAJOR} REQUIRED COMPONENTS Widgets Network)

find_package(PkgConfig REQUIRED)
pkg_check_modules(GST REQUIRED gstreamer-1.0 gstreamer-app-1.0 gstreamer-video-1.0)

set(PROJECT_SOURCES
    src/main.cpp
    src/app/AppShell.cpp
    src/app/AppShell.h
    src/app/AppState.h
    src/common/Protocol.h
    src/network/TcpClientManager.cpp
    src/network/TcpClientManager.h
    src/pages/LoginPage.cpp
    src/pages/LoginPage.h
    src/pages/ConnectPage.cpp
    src/pages/ConnectPage.h
    src/pages/DashboardPage.cpp
    src/pages/DashboardPage.h
    src/pages/DebugLogWindow.cpp
    src/pages/DebugLogWindow.h
    src/video/VideoReceiver.cpp
    src/video/VideoReceiver.h
)

add_executable(${PROJECT_NAME} ${PROJECT_SOURCES})

target_include_directories(${PROJECT_NAME} PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/src
    ${GST_INCLUDE_DIRS}
)

target_link_directories(${PROJECT_NAME} PRIVATE ${GST_LIBRARY_DIRS})

target_compile_options(${PROJECT_NAME} PRIVATE ${GST_CFLAGS_OTHER})

target_link_libraries(${PROJECT_NAME} PRIVATE
    Qt${QT_VERSION_MAJOR}::Widgets
    Qt${QT_VERSION_MAJOR}::Network
    ${GST_LIBRARIES}
)
