project (zeromq_example)
cmake_minimum_required(VERSION 2.8)

# configure ZeroMQ
IF(WIN32)
    # set the zeromq directories manually
    message("Windows platform detected, setting ZeroMQ configuration manually...")
    set(ZEROMQ_BUILD_DIR "C:/Program Files/ZeroMQ 4.0.4/bin")
    set(ZEROMQ_INCLUDE_DIRS "C:/Program Files/ZeroMQ 4.0.4/include")
    set(ZEROMQ_LIB_DIRS "C:/Program Files/ZeroMQ 4.0.4/lib")

    # find all of the subdirectories in ZEROMQV_LIB_DIRS to include
    include_directories(${ZEROMQ_BUILD_DIR})
    include_directories(${ZEROMQ_INCLUDE_DIRS})
    link_directories(${ZEROMQ_LIB_DIRS})
    
    # find all of the libs in ZEROMQ_LIB_DIRS
    file(GLOB ZeroMQ_LIBS RELATIVE ${ZEROMQ_LIB_DIRS} ${ZEROMQ_LIB_DIRS}/*.lib)
    message("Found ZeroMQ libs...")
    message(${ZeroMQ_LIBS})
ELSE(WIN32)
    find_package (ZeroMQ REQUIRED)
ENDIF(WIN32)

# configure OpenCV
IF(WIN32)
    # set the opencv directories manually
    message("Windows platform detected, setting OpenCV configuration manually...")
    set(OPENCV_BUILD_DIR "C:/OpenCV/build/x64/vc12")
    set(OPENCV_INCLUDE_DIRS "C:/OpenCV/build/include;C:/OpenCV/build/include/opencv;C:/OpenCV/build/include/opencv2")
    set(OPENCV_LIB_DIRS "C:/OpenCV/build/x64/vc12/lib")

    # find all of the subdirectories in OPENCV_LIB_DIRS to include
    include_directories(${OPENCV_BUILD_DIR})
    include_directories(${OPENCV_INCLUDE_DIRS})
    link_directories(${OPENCV_LIB_DIRS})
    
    # find all of the libs in OPENCV_LIB_DIRS
    file(GLOB OpenCV_LIBS RELATIVE ${OPENCV_LIB_DIRS} ${OPENCV_LIB_DIRS}/*2410.lib)
    message("Found OpenCV libs...")
    message(${OpenCV_LIBS})
ELSE(WIN32)
    find_package (OpenCV REQUIRED)
ENDIF(WIN32)

# create create individual projects
add_executable (zmq_cv_request zmq_cv_request.cpp)
target_link_libraries(zmq_cv_request ${ZeroMQ_LIBS} ${OpenCV_LIBS})

add_executable (zmq_cv_server zmq_cv_server.cpp)
target_link_libraries(zmq_cv_server ${ZeroMQ_LIBS} ${OpenCV_LIBS})
