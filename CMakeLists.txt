##---------------------------------------------------------------------------------------------------------------------
##  IVIS
##---------------------------------------------------------------------------------------------------------------------
##  Copyright 2019 Manuel Pérez Jiménez (a.k.a. manuoso) manuperezj@gmail.com
##---------------------------------------------------------------------------------------------------------------------
##  Permission is hereby granted, free of charge, to any person obtaining a copy of this software
##  and associated documentation files (the "Software"), to deal in the Software without restriction,
##  including without limitation the rights to use, copy, modify, merge, publish, distribute,
##  sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is
##  furnished to do so, subject to the following conditions:
##
##  The above copyright notice and this permission notice shall be included in all copies or substantial
##  portions of the Software.
##
##  THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING
##  BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
##  NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES
##  OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN
##  CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
##---------------------------------------------------------------------------------------------------------------------

cmake_minimum_required (VERSION 3.8 FATAL_ERROR)
project(ivis VERSION 0.1 LANGUAGES C CXX)

#########################################
###### Loading 3rd party libraries ######
#########################################

if( NOT CMAKE_BUILD_TYPE )
  set( CMAKE_BUILD_TYPE Release CACHE STRING
       "Choose the type of build, options are: None Debug Release RelWithDebInfo MinSizeRel."
       FORCE )
endif()

# set(CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake/modules/" ${CMAKE_MODULE_PATH})

list(APPEND IVIS_DEFINITIONS "")
set(IVIS_COMPILE_OPTIONS "${Qt5Widgets_EXECUTABLE_COMPILE_FLAGS} -Wall -g -O0")

#########################################
## Load PCL
# find_package(PCL QUIET REQUIRED)
# if(${PCL_FOUND})
# 	MESSAGE( STATUS "Adding properly PCL and its dependencies")
# 	add_definitions(-DENABLE_PCL)
# 	add_definitions(${PCL_DEFINITIONS})
# 	list(APPEND IVIS_DEFINITIONS "IVIS_USE_PCL")
# 	if(${PCL_VERSION} VERSION_GREATER 1.9)
# 		list(APPEND IVIS_DEFINITIONS "HAS_PCL_1_9")
# 	else()
# 		list(APPEND IVIS_DEFINITIONS "HAS_PCL_1_8")
# 	endif()
# else()
# 	MESSAGE( FATAL_ERROR "Couldnt Add PCL and/or it's dependencies")
# endif()

#########################################
## Load Marble
find_package(Marble REQUIRED)
list(APPEND IVIS_DEFINITIONS "IVIS_USE_MARBLE")

#########################################
## Load Qt5
find_package(Qt5Widgets REQUIRED)
find_package(Qt5PrintSupport REQUIRED)
find_package(Qt5Core REQUIRED)
MESSAGE( STATUS "Using Qt5")
list(APPEND IVIS_DEFINITIONS "IVIS_USE_QT5")

set(QT5_LIBRARIES Qt5::Widgets Qt5::PrintSupport Qt5::Core)

set(CMAKE_INCLUDE_CURRENT_DIR ON)
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTOUIC ON)

#########################################
# catking
find_package(catkin REQUIRED 
											roscpp 
											geometry_msgs)

if(${catkin_FOUND})
	catkin_package(
		CATKIN_DEPENDS roscpp geometry_msgs
		INCLUDE_DIRS include
	)
	list(APPEND IVIS_DEFINITIONS "IVIS_USE_ROS")
endif(${catkin_FOUND})

#########################################
######			Library 		   ######
#########################################
#add_definitions(-fopenmp)

# Collect all sources
file(GLOB_RECURSE IVIS_SOURCE_FILES "src/*.cpp" "src/*.c")
file(GLOB_RECURSE IVIS_HEADER_FILES "include/ivis/*.h" "include/ivis/*.inl")

add_library(${PROJECT_NAME} SHARED ${IVIS_HEADER_FILES} ${IVIS_SOURCE_FILES})

target_include_directories(${PROJECT_NAME} PRIVATE ${PROJECT_SOURCE_DIR}/include) 

target_compile_definitions(${PROJECT_NAME} PUBLIC ${IVIS_DEFINITIONS})
string(REPLACE " " ";" IVIS_COMPILE_OPTIONS_REPLACED_FLAGS ${IVIS_COMPILE_OPTIONS})

target_compile_options(${PROJECT_NAME} PUBLIC ${IVIS_COMPILE_OPTIONS_REPLACED_FLAGS})
target_compile_features(${PROJECT_NAME} PUBLIC cxx_std_11)

#########################################
# target_link_libraries(${PROJECT_NAME} PUBLIC pthread)

target_include_directories(${PROJECT_NAME} PUBLIC "/usr/include/eigen3")

target_link_libraries(${PROJECT_NAME} LINK_PUBLIC ${QT5_LIBRARIES})
target_include_directories(${PROJECT_NAME} PUBLIC ${PCL_INCLUDE_DIRS})
target_link_libraries(${PROJECT_NAME} LINK_PUBLIC ${PCL_LIBRARIES})
target_link_libraries(${PROJECT_NAME} LINK_PUBLIC Marble)

if(${catkin_FOUND})
	target_include_directories(${PROJECT_NAME} PUBLIC ${catkin_INCLUDE_DIRS})
	target_link_libraries(${PROJECT_NAME} LINK_PUBLIC ${catkin_LIBRARIES})
	add_dependencies(${PROJECT_NAME} ${catkin_EXPORTED_TARGETS})
endif(${catkin_FOUND})

#########################################
######		   	Tools 		   	   ######
#########################################

if(${BUILD_TOOLS})
	add_subdirectory(tools)
endif(${BUILD_TOOLS})

#########################################
######			Examples 		   ######
#########################################

# if(${BUILD_EXAMPLES})
# 	add_subdirectory(examples)
# endif(${BUILD_EXAMPLES})

#########################################
######			INSTALL 		   ######
#########################################

# include(CMakePackageConfigHelpers)

# if(NOT DEFINED CMAKE_INSTALL_PREFIX)
#   set(CMAKE_INSTALL_PREFIX "/usr/local")
# endif() 

# install(TARGETS ivis
#         EXPORT ivisTargets
#         LIBRARY DESTINATION lib
#         ARCHIVE DESTINATION lib
#         INCLUDES DESTINATION ${CMAKE_INSTALL_PREFIX}
#         )

# write_basic_package_version_file(
#     ivisConfigVersion.cmake
#     VERSION ${PACKAGE_VERSION}
#     COMPATIBILITY AnyNewerVersion
#     )

# # This installs the include folder
# install(DIRECTORY include DESTINATION ${CMAKE_INSTALL_PREFIX} FILES_MATCHING PATTERN "*.h")
# install(DIRECTORY include DESTINATION ${CMAKE_INSTALL_PREFIX} FILES_MATCHING PATTERN "*.inl")

# export(TARGETS ivis NAMESPACE ivis:: FILE ivisTargets.cmake)

# # This generates ivisTargets.cmake
# install(EXPORT ivisTargets
#     FILE ivisTargets.cmake
#     NAMESPACE ivis::
#     DESTINATION lib/cmake/ivis
#     )

# configure_file(ivisConfig.cmake.in ivisConfig.cmake @ONLY)
# install(FILES "${CMAKE_CURRENT_BINARY_DIR}/ivisConfig.cmake"
#         DESTINATION lib/cmake/ivis
#         )

# # uninstall target 
# if(NOT TARGET uninstall) 
#   configure_file( 
#       "${CMAKE_CURRENT_SOURCE_DIR}/cmake_uninstall.cmake.in" 
#       "${CMAKE_CURRENT_BINARY_DIR}/cmake_uninstall.cmake" 
#       IMMEDIATE @ONLY) 
 
#   add_custom_target(uninstall COMMAND ${CMAKE_COMMAND} -P ${CMAKE_CURRENT_BINARY_DIR}/cmake_uninstall.cmake) 
# endif() 
