cmake_minimum_required(VERSION 3.0)
# Setup the project.
# Its name is defined to be the controller directory name.
get_filename_component(PROJECT ${CMAKE_SOURCE_DIR} NAME)
project(PndControl VERSION 1.4.0)

# Configure version header file
configure_file(
  ${CMAKE_CURRENT_SOURCE_DIR}/version.h.in
  ${CMAKE_CURRENT_BINARY_DIR}/version.h
  @ONLY
)

# SET(PROJECT_WEBOTS_OLD "webots_interface")
SET(PROJECT_WEBOTS "mpc_controller")
#set(CMAKE_BUILD_TYPE "Release")

# option(webots_sim_def "compile webots sim" off)
set(webots_sim_def FALSE CACHE BOOL "compile webots sim")
set(realrobot TRUE CACHE BOOL "compile real robot control")
set(rosopen FALSE CACHE BOOL "compile with ros")

if (realrobot)
	# for nokov get information
	if (UNIX)
		add_definitions(-D_LINUX)
		SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -pthread")
	endif()
endif()

if(rosopen)
    add_definitions(-DROS_PUB)
endif()

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -pthread")
add_definitions(-Wall -O2 -g)
add_definitions(-w)
ADD_DEFINITIONS(-DQT_NO_KEYWORDS)
# add_definitions(-Wall -std=c++11 -mavx -mfma  -O3  -g )
LIST( APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR} )
if(webots_sim_def)
	# set(SOURCES ${C_SOURCES} ${CPP_SOURCES})
	SET(WEBOTS_HOME /usr/local/webots/)
	link_directories(${WEBOTS_HOME}/lib/controller)
	set (LIBRARIES ${CMAKE_SHARED_LIBRARY_PREFIX}Controller${CMAKE_SHARED_LIBRARY_SUFFIX} ${CMAKE_SHARED_LIBRARY_PREFIX}CppController${CMAKE_SHARED_LIBRARY_SUFFIX})
	include_directories(${WEBOTS_HOME}/include/controller/c ${WEBOTS_HOME}/include/controller/cpp)
	message(STATUS "webots:${WEBOTS_HOME}")

	# add ros packages
	if(rosopen)
		find_package(catkin REQUIRED COMPONENTS roscpp rospy roslib rosconsole std_msgs)
		find_package(roscpp REQUIRED)
		find_package(std_msgs REQUIRED)
		INCLUDE_DIRECTORIES(${roscpp_INCLUDE_DIRS} ${std_msgs_INCLUDE_DIRS})
	endif()

endif()

# Search for the RBDL include directory and library
SET(Simde_INCLUDE_DIR /home/pndrobot/Downloads/simde)
FIND_PACKAGE (RBDL REQUIRED)
FIND_PACKAGE (Eigen3 3.0.0 REQUIRED)
#find_package(proxsuite REQUIRED)
find_package(QT NAMES Qt6 Qt5 COMPONENTS Core REQUIRED)
find_package(Qt${QT_VERSION_MAJOR} COMPONENTS Core REQUIRED)

ADD_SUBDIRECTORY(RobotInterface)
INCLUDE_DIRECTORIES(${PROJECT_SOURCE_DIR}/RobotInterface/include)
LINK_DIRECTORIES(${CMAKE_CURRENT_SOURCE_DIR}/RobotInterface/build)

# Add the include directory to the include paths
INCLUDE_DIRECTORIES ( ${RBDL_INCLUDE_DIR} ${EIGEN3_INCLUDE_DIR} )

link_directories("/usr/local/lib")
INCLUDE_DIRECTORIES ( "/usr/local/include")
link_directories("/usr/lib")
INCLUDE_DIRECTORIES ( "/usr/include")

INCLUDE_DIRECTORIES(${CMAKE_CURRENT_BINARY_DIR})



set(THREADS_PREFER_PTHREAD_FLAG ON)
find_package(Threads REQUIRED)

INCLUDE_DIRECTORIES(${PROJECT_SOURCE_DIR}/nokov/include)
aux_source_directory(${PROJECT_SOURCE_DIR}/nokov/src NokovClinet_DIR)
link_directories(${PROJECT_SOURCE_DIR}/nokov/lib)

INCLUDE_DIRECTORIES(${PROJECT_SOURCE_DIR}/RobotControl/include)
aux_source_directory(${PROJECT_SOURCE_DIR}/RobotControl/src RobotControl_DIR)

INCLUDE_DIRECTORIES(${PROJECT_SOURCE_DIR}/StateMachine/include)
aux_source_directory(${PROJECT_SOURCE_DIR}/StateMachine/src StateMachine_DIR)
if (realrobot)
	include_directories(${PROJECT_SOURCE_DIR}/)
	include_directories(${PROJECT_SOURCE_DIR}/pnd-cpp-sdk/pnd/include)
endif()
INCLUDE_DIRECTORIES(${PROJECT_SOURCE_DIR}/MotionPlan/include)
aux_source_directory(${PROJECT_SOURCE_DIR}/MotionPlan/src MotionPlan_DIR)

INCLUDE_DIRECTORIES(${PROJECT_SOURCE_DIR}/example/include/S2P)
aux_source_directory(${PROJECT_SOURCE_DIR}/example/src/S2P S2P_DIR)
INCLUDE_DIRECTORIES(${PROJECT_SOURCE_DIR}/example/include/wrist_S2P)
aux_source_directory(${PROJECT_SOURCE_DIR}/example/src/wrist_S2P wrist_S2P_DIR)
if (realrobot)
	INCLUDE_DIRECTORIES(${PROJECT_SOURCE_DIR}/example/include/vnIMU)
	INCLUDE_DIRECTORIES(${PROJECT_SOURCE_DIR}/example/include/vnIMU/vn)
	aux_source_directory(${PROJECT_SOURCE_DIR}/example/src/vnIMU vnIMU_DIR)
	aux_source_directory(${PROJECT_SOURCE_DIR}/example/src/vnIMU/vn vnIMU_vn_DIR)
endif()
include_directories(${PROJECT_SOURCE_DIR}/example/include/broccoli)
include_directories(${PROJECT_SOURCE_DIR}/example/include)
aux_source_directory(${PROJECT_SOURCE_DIR}/example/src main_DIR)


if (webots_sim_def)
	include_directories(${PROJECT_SOURCE_DIR}/Webots/controllers/mpc_controller/include)
	aux_source_directory(${PROJECT_SOURCE_DIR}/Webots/controllers/mpc_controller/src WEBOTS_DIR)
	include_directories(${PROJECT_SOURCE_DIR}/Webots/rospub/include)
	aux_source_directory(${PROJECT_SOURCE_DIR}/Webots/rospub/src PUB_DIR)
	# aux_source_directory(${PROJECT_SOURCE_DIR}/webotsim/controllers/webots_interface/src SIM_DIR)
endif()

INCLUDE_DIRECTORIES(${PROJECT_SOURCE_DIR}/StateEstimation/include)
aux_source_directory(${PROJECT_SOURCE_DIR}/StateEstimation/src StateEstimation_DIR)

##------real world----------------------------------------------------
if (realrobot)
	add_executable(	PndControl 
					${NokovClinet_DIR} 
					${RobotControl_DIR} 
					${StateMachine_DIR} 
					${MotionPlan_DIR} 
					${MotorList_DIR} 
					${S2P_DIR} 
					${vnIMU_DIR} ${vnIMU_vn_DIR}
					${main_DIR}
					${StateEstimation_DIR})
	target_link_libraries(PndControl ${LIBRARIES} Threads::Threads RobotInterface)

	# And link the library against the executable
	TARGET_LINK_LIBRARIES (PndControl

		${RBDL_LIBRARY}
		rbdl_urdfreader
		libqpOASES.a

	)
	# target_link_libraries(PndControl PUBLIC proxsuite::proxsuite proxsuite::proxsuite-vectorized)
	TARGET_LINK_LIBRARIES(PndControl  Qt${QT_VERSION_MAJOR}::Core RobotInterface)
	TARGET_LINK_LIBRARIES(PndControl nokov_sdk)
	# target_compile_options(PndControl PUBLIC "-march=native")
endif()

if (webots_sim_def)
	add_definitions(-DWEBOTS)
	##-------------------------------------webots sim ----------------------------------------
	add_executable(	${PROJECT_WEBOTS} 
					${RobotControl_DIR} 
					${StateMachine_DIR} 
					${MotionPlan_DIR} 
					${S2P_DIR} 
					${StateEstimation_DIR} 
					${WEBOTS_DIR}
					)
	target_link_libraries(${PROJECT_WEBOTS} ${LIBRARIES})
    if(rosopen)
		target_sources(${PROJECT_WEBOTS} PRIVATE ${PUB_DIR})
		target_link_libraries(${PROJECT_WEBOTS} ${roscpp_LIBRARIES})
	endif()
	message(STATUS "RBDL_LIBRARY:${RBDL_LIBRARY}")
	# And link the library against the executable
	TARGET_LINK_LIBRARIES (${PROJECT_WEBOTS}
		${RBDL_LIBRARY}
		rbdl_urdfreader
		libqpOASES.a
	)
#	target_link_libraries(${PROJECT_WEBOTS} proxsuite::proxsuite proxsuite::proxsuite-vectorized)
	TARGET_LINK_LIBRARIES(${PROJECT_WEBOTS}  Qt${QT_VERSION_MAJOR}::Core)

	target_compile_options(${PROJECT_WEBOTS} PUBLIC "-march=native")
endif()

##-------------------------------------INSTALL--------------------------------------------------
if (realrobot)
	# real world
	# INSTALL(DIRECTORY build/PndControl DESTINATION bin
    #         PATTERN "build/PndControl"
    #         PERMISSIONS OWNER_EXECUTE OWNER_WRITE OWNER_READ
    #         GROUP_EXECUTE GROUP_READ)
	INSTALL(TARGETS PndControl
			RUNTIME DESTINATION bin)
	INSTALL(DIRECTORY Sources/ DESTINATION bin/Sources
			PATTERN "Sources/*")
	INSTALL(DIRECTORY RobotInterface/JointInterface/pnd-cpp-sdk/pnd/lib/linux_x86_64/ DESTINATION bin
			PATTERN "RobotInterface/JointInterface/pnd-cpp-sdk/pnd/lib/linux_x86_64/*")
 	# INSTALL(FILES Sources/config/joint_abs_config.json DESTINATION bin/Sources/config)
	# INSTALL(FILES Sources/config/joint_pd_config.json DESTINATION bin/Sources/config)
    INSTALL(FILES build/RobotInterface/pnd-cpp-sdk-release/libpndc++.so DESTINATION bin)
	# INSTALL(FILES build/RobotInterface/pnd-cpp-sdk-release/libpndc++.so.0.6.1 DESTINATION bin)
	INSTALL(FILES build/RobotInterface/pnd-cpp-sdk-release/libpndc++.so.1 DESTINATION bin)
	INSTALL(FILES build/RobotInterface/pnd-cpp-sdk-release/libpndc++.so.1.5.1 DESTINATION bin)
    INSTALL(FILES build/RobotInterface/libRobotInterface.so DESTINATION bin)
	INSTALL(FILES nokov/lib/libnokov_sdk.so DESTINATION bin)
	INSTALL(FILES PndControl.sh DESTINATION bin)
	INSTALL(FILES run.sh DESTINATION bin)
endif()
if (webots_sim_def)	
	# webots sim
	# INSTALL(TARGETS ${PROJECT_WEBOTS}
	# 		RUNTIME DESTINATION Webots/controllers/mpc_controller)
	INSTALL(DIRECTORY Sources/ DESTINATION Webots/controllers/mpc_controller/Sources
			PATTERN "Sources/*"
			PERMISSIONS OWNER_EXECUTE OWNER_WRITE OWNER_READ
			GROUP_EXECUTE GROUP_READ)
	ADD_CUSTOM_COMMAND(TARGET ${PROJECT_WEBOTS} POST_BUILD COMMAND ${CMAKE_COMMAND} -E
			copy ${CMAKE_BINARY_DIR}/${PROJECT_WEBOTS} ${CMAKE_SOURCE_DIR}/Webots/controllers/mpc_controller/
			)
endif()

					
