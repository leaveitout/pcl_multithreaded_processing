cmake_minimum_required(VERSION 3.3)

project(pcl_multi_threaded_processing)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_HOME_DIRECTORY}/bin)

find_package(PCL 1.7 REQUIRED)
find_package(OpenCV REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

set(SOURCE_FILES
        src/main.cpp
        src/App.cpp
        include/App.hpp
        include/Logger.hpp
        src/Logger.cpp
        include/Buffer.hpp
        include/NI2Producer.hpp
        include/Timer.hpp
        include/Definitions.hpp
        include/CloudViewer.hpp
        include/Processor.hpp
        src/NI2ImageViewer.cpp
        include/NI2ImageViewer.hpp
        src/NI2ImageViewer.cpp include/ProcessorProducer.hpp include/CloudImageViewer.hpp include/CloudAndImage.hpp include/MultiCloudAndImage.hpp include/CloudAndImageProducer.hpp include/MultiCloudAndImageProducer.hpp include/MultiCloud.hpp include/MultiCloudProducer.hpp include/MultiCloudViewer.hpp include/TimedCloud.hpp include/MultiTimedCloud.hpp include/MultiTimedCloudProducer.hpp include/CalibratorNode.hpp src/Palette.cpp include/Palette.hpp include/KeypointSorter.cpp include/KeypointSorter.hpp include/SquareDetector.cpp include/SquareDetector.hpp include/CameraId.hpp include/MultiTimedCloudViewer.hpp include/TimedCloudViewer.hpp include/IndexedTimedCloud.hpp include/IndexedTimedCloudViewer.hpp)

include_directories(${CMAKE_HOME_DIRECTORY}/include)

add_executable(pcl_multi_threaded_processing ${SOURCE_FILES})

target_link_libraries(pcl_multi_threaded_processing ${PCL_LIBRARIES})
target_link_libraries(pcl_multi_threaded_processing ${OpenCV_LIBS})
