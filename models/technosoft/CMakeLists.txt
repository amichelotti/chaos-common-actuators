cmake_minimum_required(VERSION 2.8)
project(common_actuators_technosoft)
LINK_DIRECTORIES(${CMAKE_CURRENT_SOURCE_DIR}/lib)
ADD_DEFINITIONS(-DDEBUG -fpic)

FILE(GLOB src *.cpp)    
IF(BUILD_FORCE_STATIC)
  ADD_LIBRARY(${PROJECT_NAME} STATIC ${src} )
ELSE()
  ADD_LIBRARY(${PROJECT_NAME} SHARED ${src})
ENDIF()

TARGET_LINK_LIBRARIES(${PROJECT_NAME} common_debug TML_lib tmlcomm)
ADD_EXECUTABLE(test_technosoft_slit test/test_slit.cpp)
TARGET_LINK_LIBRARIES(test_technosoft_slit ${PROJECT_NAME} common_actuators common_debug ${chaoslib} tmlcomm boost_regex )


FILE(GLOB model_src *.h)    
 

INSTALL(FILES ${model_src} DESTINATION include/common/actuators/models/technosoft ) 
FILE(GLOB model_lib lib/*)    
INSTALL(FILES ${model_lib} DESTINATION lib/)
INSTALL_TARGETS(/lib ${PROJECT_NAME})
INSTALL_TARGETS(/bin test_technosoft_slit)

