cmake_minimum_required(VERSION 2.8)
project( ImageFunction )


SET(GCC_COVERAGE_COMPILE_FLAGS "-fPIC")
add_definitions(${GCC_COVERAGE_COMPILE_FLAGS})
find_package( OpenCV REQUIRED )
find_package( PythonLibs 2.7 REQUIRED)
find_package( Boost COMPONENTS python REQUIRED )

include_directories( ${PYTHON_INCLUDE_DIRS} )
include_directories( ${Boost_INCLUDE_DIR} )

add_library (ImgFunctionDefs  ImageFunction.cpp)
add_library( ImgProc SHARED ImgLib.cpp)
add_library( cv2  cv2.cpp)



target_link_libraries( ImgProc ${Boost_LIBRARIES} ImgFunctionDefs )
set_Target_properties( ImgProc PROPERTIES PREFIX "" )