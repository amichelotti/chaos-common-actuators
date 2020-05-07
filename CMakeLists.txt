cmake_minimum_required(VERSION 2.8)
project(common_actuators)
find_package(Boost MODULE REQUIRED REQUIRED COMPONENTS program_options system)
FILE(GLOB core_src core/*.cpp)    
ADD_LIBRARY(${PROJECT_NAME} STATIC ${core_src})
include_directories(../.. ${Boost_INCLUDE_DIRS})
link_directories(${CMAKE_INSTALL_PREFIX}/lib)

#INSTALL_TARGETS(/lib ${PROJECT_NAME})

INSTALL(TARGETS ${PROJECT_NAME}
        DESTINATION "lib"
        PERMISSIONS OWNER_WRITE OWNER_READ OWNER_EXECUTE GROUP_READ GROUP_WRITE GROUP_EXECUTE WORLD_READ WORLD_EXECUTE)

FILE(GLOB core_src core/*.h)    
INSTALL(FILES ${core_src} DESTINATION include/common/actuators/core)
add_subdirectory(models)





