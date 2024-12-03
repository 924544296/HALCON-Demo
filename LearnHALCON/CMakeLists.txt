# CMakeList.txt : CMake project for LearnHALCON, include source and define
# project specific logic here.
#
cmake_minimum_required (VERSION 3.8)

# Enable Hot Reload for MSVC compilers if supported.
if (POLICY CMP0141)
  cmake_policy(SET CMP0141 NEW)
  set(CMAKE_MSVC_DEBUG_INFORMATION_FORMAT "$<IF:$<AND:$<C_COMPILER_ID:MSVC>,$<CXX_COMPILER_ID:MSVC>>,$<$<CONFIG:Debug,RelWithDebInfo>:EditAndContinue>,$<$<CONFIG:Debug,RelWithDebInfo>:ProgramDatabase>>")
endif()

project ("LearnHALCON")



# halcon
include_directories("D:\\software\\HALCON2411\\HALCON-24.11-Progress-Steady\\include")
link_directories("D:\\software\\HALCON2411\\HALCON-24.11-Progress-Steady\\lib\\x64-win64")
# opencv
#set(OpenCV_DIR "D:/software/opencv/opencv490/build")
#find_package(OpenCV REQUIRED)
#include_directories(${OpenCV_INCLUDE_DIRS})
include_directories("D:/software/opencv/opencv490/build/include"
                    "D:/software/opencv/opencv490/build/include/opencv2")
link_directories("D:/software/opencv/opencv490/build/x64/vc16/lib")


# Add source to this project's executable.
add_executable (LearnHALCON "LearnHALCON.cpp" "LearnHALCON.h" "main.cpp")

if (CMAKE_VERSION VERSION_GREATER 3.12)
  set_property(TARGET LearnHALCON PROPERTY CXX_STANDARD 20)
endif()

# TODO: Add tests and install targets if needed.
# halcon
target_link_libraries (LearnHALCON halcon halconcpp)
# opencv
#target_link_libraries(Detection ${OpenCV_LIBS})
target_link_libraries(LearnHALCON opencv_world490d)