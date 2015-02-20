cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

project(PCL_Demo)

# configure PCL
find_package(PCL 1.7.1 REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

# configure OpenCV
IF(WIN32)
    message("Windows platform detected, generating OpenCV configuration...")
    
    set(OPENCV_BUILD_DIR "C:/OpenCV/build/x64/vc10")
    set(OPENCV_INCLUDE_DIRS "C:/OpenCV/build/include;C:/OpenCV/build/include/opencv;C:/OpenCV/build/include/opencv2")
    set(OPENCV_LIB_DIRS "C:/OpenCV/build/x64/vc10/lib")
    
    # find all of the subdirectories in OPENCV_LIB_DIRS to include
    include_directories(${OPENCV_BUILD_DIR})
    include_directories(${OPENCV_INCLUDE_DIRS})
    link_directories(${OPENCV_LIB_DIRS})
    
    # find all of the libs in OPENCV_LIB_DIRS
    file(GLOB OpenCV_LIBS RELATIVE ${OPENCV_LIB_DIRS} ${OPENCV_LIB_DIRS}/*249.lib)
    message("Found OpenCV libs...")
    message(${OpenCV_LIBS})
ELSE(WIN32)
    find_package (OpenCV REQUIRED)
ENDIF(WIN32)

# suppress deprecation warnings
#set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wno-deprecated-declarations")

add_executable (openni2_snapper openni2_snapper.cpp)
target_link_libraries (openni2_snapper ${PCL_LIBRARIES})

add_executable (load_pcd load_pcd.cpp CloudVisualizer.cpp Utility.cpp)
target_link_libraries (load_pcd ${PCL_LIBRARIES} ${OpenCV_LIBS})
