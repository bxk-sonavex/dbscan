cmake_minimum_required(VERSION 3.24)

set(PROJECT_NAME dbscan)
project(${PROJECT_NAME} CXX)

cmake_policy(SET CMP0074 NEW) # find_package() uses <PackageName>_ROOT variables.

set(HDR_${PROJECT_NAME}
    ${CMAKE_CURRENT_LIST_DIR}/dbscan.hpp)

set(SRC_${PROJECT_NAME}
    ${CMAKE_CURRENT_LIST_DIR}/dbscan.cpp
    ${CMAKE_CURRENT_LIST_DIR}/example.cpp)

# build a target list
set(TARGETS_${PROJECT_NAME} "")

# build AlgoEngine
set(TARGET_NAME ${PROJECT_NAME})
add_executable(${TARGET_NAME})
target_include_directories(${TARGET_NAME} PRIVATE
                           "${CMAKE_CURRENT_LIST_DIR}"
                           "${CMAKE_CURRENT_LIST_DIR}/vendor")
target_sources(${TARGET_NAME} PRIVATE
               ${HDR_${PROJECT_NAME}}
               ${SRC_${PROJECT_NAME}})

set_property(TARGET ${TARGET_NAME} PROPERTY CXX_STANDARD 20)
