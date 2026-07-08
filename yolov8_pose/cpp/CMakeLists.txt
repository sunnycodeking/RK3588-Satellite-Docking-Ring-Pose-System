cmake_minimum_required(VERSION 3.10)

project(rknn_yolov8_ellipse_demo)

if (ENABLE_ASAN)
	message(STATUS "BUILD WITH ADDRESS SANITIZER")
	set (CMAKE_C_FLAGS_DEBUG "${CMAKE_C_FLAGS_DEBUG} -fno-omit-frame-pointer -fsanitize=address")
	set (CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -fno-omit-frame-pointer -fsanitize=address")
	set (CMAKE_LINKER_FLAGS_DEBUG "${CMAKE_LINKER_FLAGS_DEBUG} -fno-omit-frame-pointer -fsanitize=address")
endif ()

set(rknpu_yolov8_file rknpu2/yolov8.cc)

if (TARGET_SOC STREQUAL "rv1106" OR TARGET_SOC STREQUAL "rv1103")
    add_definitions(-DRV1106_1103)
    set(rknpu_yolov8_file rknpu2/yolov8_rv1106_1103.cc)
    #dma
    include_directories(${CMAKE_CURRENT_SOURCE_DIR}/../../../3rdparty/allocator/dma)
endif()

if(TARGET_SOC STREQUAL "rk1808" OR TARGET_SOC STREQUAL "rv1109" OR TARGET_SOC STREQUAL "rv1126")
    add_definitions(-DRKNPU1)
    set(rknpu_yolov8_file rknpu1/yolov8.cc)
endif()

add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/../../../3rdparty/ 3rdparty.out)
add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/../../../utils/ utils.out)

set(CMAKE_INSTALL_RPATH "$ORIGIN/../lib")

file(GLOB SRCS ${CMAKE_CURRENT_SOURCE_DIR}/*.cc)

add_executable(${PROJECT_NAME}
    main.cc
    postprocess.cc
    kcf_tracker.cc  # 新加
    ${rknpu_yolov8_file}
)
set(PKG_CONFIG_PATH $ENV{PKG_CONFIG_PATH})

find_package(PkgConfig REQUIRED)
pkg_check_modules(GST_LIBRARIES REQUIRED
    gstreamer-1.0
    gstreamer-video-1.0
    glib-2.0
    gobject-2.0
    gstreamer-app-1.0
    
)


# 设置OpenCV库路径
set(OPENCV_LIB_DIR "/home/elf/aarch64-buildroot-linux-gnu_sdk-buildroot/aarch64-buildroot-linux-gnu/sysroot/usr/lib/opencv4")

# 查找OpenCV库文件
find_library(OPENCV_CORE_LIB NAMES opencv_core PATHS ${OPENCV_LIB_DIR})
find_library(OPENCV_TRACKING_LIB NAMES opencv_tracking PATHS ${OPENCV_LIB_DIR})
find_library(OPENCV_VIDEO_LIB NAMES opencv_video PATHS ${OPENCV_LIB_DIR})
find_library(OPENCV_HIGHGUI_LIB NAMES opencv_highgui PATHS ${OPENCV_LIB_DIR})
find_library(OPENCV_IMGPROC_LIB NAMES opencv_imgproc PATHS ${OPENCV_LIB_DIR})
find_library(OPENCV_DNN_LIB NAMES opencv_dnn PATHS ${OPENCV_LIB_DIR})
find_library(OPENCV_CALIB3D_LIB NAMES opencv_calib3d PATHS ${OPENCV_LIB_DIR})
find_library(OPENCV_FEATURE_LIB NAMES opencv_features2d PATHS ${OPENCV_LIB_DIR})
find_library(OPENCV_FLANN_LIB NAMES opencv_flann PATHS ${OPENCV_LIB_DIR})
find_library(OPENCV_IMGCODECS_LIB NAMES opencv_imgcodecs PATHS ${OPENCV_LIB_DIR})
find_library(OPENCV_VIDEOIO_LIB NAMES opencv_videoio PATHS ${OPENCV_LIB_DIR})
if(OPENCV_CORE_LIB AND OPENCV_VIDEO_LIB)
    message(STATUS "Found OpenCV core library: ${OPENCV_CORE_LIB}")
    message(STATUS "Found OpenCV tracking library: ${OPENCV_TRACKING_LIB}")
else()
    message(WARNING "OpenCV libraries not found in ${OPENCV_LIB_DIR}")
endif()
target_link_libraries(${PROJECT_NAME}
    ${GST_LIBRARIES}
    imageutils
    fileutils
    imagedrawing    
    ${LIBRKNNRT}
    dl

    # OpenCV库
    ${OPENCV_TRACKING_LIB}
    ${OPENCV_VIDEO_LIB}
    ${OPENCV_HIGHGUI_LIB}
    ${OPENCV_IMGPROC_LIB}
    ${OPENCV_CORE_LIB}
    ${OPENCV_DNN_LIB}
    ${OPENCV_CALIB3D_LIB}
    ${OPENCV_FEATURE_LIB}
    ${OPENCV_FLANN_LIB}
    ${OPENCV_IMGCODECS_LIB} 
    ${OPENCV_VIDEOIO_LIB} 
    /home/elf/aarch64-buildroot-linux-gnu_sdk-buildroot/aarch64-buildroot-linux-gnu/sysroot/usr/lib/libgstapp-1.0.so.0.2209.0
    rknnrt
)

if (CMAKE_SYSTEM_NAME STREQUAL "Android")
    target_link_libraries(${PROJECT_NAME}
    log
)
endif()
set(SYSROOT $ENV{SYSROOT})
message(STATUS " ${SYSROOT}/usr/include")
message(STATUS "!!!!!!!!!!!CMAKE_SYSTEM_NAME: ${CMAKE_SYSTEM_NAME}")
set(OPENCV_INCLUDE_DIRS "${SYSROOT}/usr/include/opencv2")

if (CMAKE_SYSTEM_NAME STREQUAL "Linux")
    set(THREADS_PREFER_PTHREAD_FLAG ON)
    find_package(Threads REQUIRED)
    target_link_libraries(${PROJECT_NAME} Threads::Threads)
endif()
set(GST_INCLUDES 
    ${SYSROOT}/usr/include/gstreamer-1.0     # gst/gst.h所在目录的父级
    ${SYSROOT}/usr/include/glib-2.0          # GLib相关头文件
    ${SYSROOT}/usr/lib/glib-2.0/include      # GLib附加头文件
)
message(STATUS " ${GST_INCLUDES}")
target_include_directories(${PROJECT_NAME} PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${LIBRKNNRT_INCLUDES}
    ${OPENCV_INCLUDE_DIRS}
    ${GST_INCLUDES}

)

install(TARGETS ${PROJECT_NAME} DESTINATION .)
install(FILES ${CMAKE_CURRENT_SOURCE_DIR}/ellipsecopy.jpg DESTINATION .)
install(FILES ${CMAKE_CURRENT_SOURCE_DIR}/../model/ellipse_list.txt DESTINATION model)
file(GLOB RKNN_FILES "${CMAKE_CURRENT_SOURCE_DIR}/../model/*.rknn")
install(FILES ${RKNN_FILES} DESTINATION model)



# cmake_minimum_required(VERSION 3.10)

# project(rknn_yolov8_camera_demo)

# # 关键修复：设置 sysroot 和库路径

# set(CMAKE_SYSROOT $ENV{SYSROOT})
# message(STATUS "Using SYSROOT: ${CMAKE_SYSROOT}")

# # 明确指定库搜索路径

# set(CMAKE_LIBRARY_PATH
#     ${CMAKE_SYSROOT}/usr/lib
#     ${CMAKE_SYSROOT}/usr/lib64
#     ${CMAKE_SYSROOT}/lib
#     ${CMAKE_SYSROOT}/lib64
# )

# # 设置链接器搜索路径

# set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} -Wl,-rpath-link,${CMAKE_SYSROOT}/usr/lib")

# if (ENABLE_ASAN)
#     message(STATUS "BUILD WITH ADDRESS SANITIZER")
#     set (CMAKE_C_FLAGS_DEBUG "${CMAKE_C_FLAGS_DEBUG} -fno-omit-frame-pointer -fsanitize=address")
#     set (CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -fno-omit-frame-pointer -fsanitize=address")
#     set (CMAKE_LINKER_FLAGS_DEBUG "${CMAKE_LINKER_FLAGS_DEBUG} -fno-omit-frame-pointer -fsanitize=address")
# endif ()

# set(rknpu_yolov8_file rknpu2/yolov8.cc)
# add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/../../../3rdparty/ 3rdparty.out)
# add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/../../../utils/ utils.out)
# set(CMAKE_INSTALL_RPATH "$ORIGIN/../lib")

# file(GLOB SRCS ${CMAKE_CURRENT_SOURCE_DIR}/*.cc)

# add_executable(${PROJECT_NAME}
#     main.cc
#     postprocess.cc
#     ${rknpu_yolov8_file}
# )

# # 配置 pkg-config 使用正确的路径

# set(ENV{PKG_CONFIG_PATH} "${CMAKE_SYSROOT}/usr/lib/pkgconfig:${CMAKE_SYSROOT}/usr/share/pkgconfig")
# set(PKG_CONFIG_USE_CMAKE_PREFIX_PATH ON)

# find_package(PkgConfig REQUIRED)
# pkg_check_modules(GST REQUIRED
#     gstreamer-1.0
#     gstreamer-video-1.0
#     gstreamer-app-1.0
#     glib-2.0
#     gobject-2.0
# )

# # 打印调试信息

# message(STATUS "GST_LIBRARIES: ${GST_LIBRARIES}")
# message(STATUS "GST_LIBRARY_DIRS: ${GST_LIBRARY_DIRS}")
# message(STATUS "GST_LDFLAGS: ${GST_LDFLAGS}")

# target_link_libraries(${PROJECT_NAME}

# # 使用变量而非硬编码路径

#     ${GST_LIBRARIES}
   

#     # 其他库
#     imageutils
#     fileutils
#     imagedrawing    
#     ${LIBRKNNRT}
#     dl
#     rknnrt

# )

# message(STATUS "!!!!!!!!!!!CMAKE_SYSTEM_NAME: ${CMAKE_SYSTEM_NAME}")

# if (CMAKE_SYSTEM_NAME STREQUAL "Linux")
#     set(THREADS_PREFER_PTHREAD_FLAG ON)
#     find_package(Threads REQUIRED)
#     target_link_libraries(${PROJECT_NAME} Threads::Threads)
# endif()

# # 设置包含路径

# set(GST_INCLUDES 
#     ${GST_INCLUDE_DIRS}
#     ${CMAKE_SYSROOT}/usr/include/gstreamer-1.0
#     ${CMAKE_SYSROOT}/usr/include/glib-2.0
#     ${CMAKE_SYSROOT}/usr/lib/glib-2.0/include
# )

# target_include_directories(${PROJECT_NAME} PRIVATE
#     ${CMAKE_CURRENT_SOURCE_DIR}
#     ${LIBRKNNRT_INCLUDES}
#     ${GST_INCLUDES}
# )

# install(TARGETS ${PROJECT_NAME} DESTINATION .)
# install(FILES ${CMAKE_CURRENT_SOURCE_DIR}/../model/bus.jpg DESTINATION model)
# install(FILES ${CMAKE_CURRENT_SOURCE_DIR}/../model/coco_80_labels_list.txt DESTINATION model)
# file(GLOB RKNN_FILES "${CMAKE_CURRENT_SOURCE_DIR}/../model/*.rknn")
# install(FILES ${RKNN_FILES} DESTINATION model)