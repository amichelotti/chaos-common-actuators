cmake_minimum_required(VERSION 2.8)
project(common_actuators_technosoft)
FILE(GLOB model_lib ${CMAKE_CURRENT_SOURCE_DIR}/lib/*)    
FILE(COPY ${model_lib} DESTINATION ${CMAKE_INSTALL_PREFIX}/lib)
MESG(STATUS "libraries ${model_lib}")
LINK_DIRECTORIES(${CMAKE_CURRENT_SOURCE_DIR}/lib)
ADD_DEFINITIONS(-DDEBUG -fpic)

FILE(GLOB src *.cpp)    
ADD_LIBRARY(${PROJECT_NAME} SHARED ${src} )

TARGET_LINK_LIBRARIES(${PROJECT_NAME} common_debug TML_lib tmlcomm)

ADD_EXECUTABLE(test_technosoft_slit test/test_slit.cpp)
TARGET_LINK_LIBRARIES(test_technosoft_slit ${PROJECT_NAME} common_actuators common_debug ${chaoslib} tmlcomm chaos_common ${FrameworkLib})


FILE(GLOB model_src *.h)    
 

INSTALL(FILES ${model_src} DESTINATION include/common/actuators/models/technosoft ) 

#INSTALL(FILES ${model_lib} DESTINATION lib/)
INSTALL(TARGETS ${PROJECT_NAME}
        DESTINATION "lib"
        PERMISSIONS OWNER_WRITE OWNER_READ OWNER_EXECUTE GROUP_READ GROUP_WRITE GROUP_EXECUTE WORLD_READ WORLD_EXECUTE)

INSTALL(TARGETS test_technosoft_slit
        DESTINATION "bin"
        PERMISSIONS OWNER_WRITE OWNER_READ OWNER_EXECUTE GROUP_READ GROUP_WRITE GROUP_EXECUTE WORLD_READ WORLD_EXECUTE)
