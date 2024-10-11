CMAKE_MINIMUM_REQUIRED(VERSION 3.0)
PROJECT(RobotInterface)

INCLUDE_DIRECTORIES("/usr/local/lib")
INCLUDE_DIRECTORIES("/usr/local/include")

FIND_PACKAGE(QT NAMES Qt6 Qt5 COMPONENTS Core REQUIRED)
FIND_PACKAGE(Qt${QT_VERSION_MAJOR} COMPONENTS Core REQUIRED)
FIND_PACKAGE(Eigen3 REQUIRED)
FIND_PACKAGE(RBDL REQUIRED)

INCLUDE_DIRECTORIES(${EIGEN3_INCLUDE_DIR})
INCLUDE_DIRECTORIES(${RBDL_INCLUDE_DIR})
INCLUDE_DIRECTORIES(JointInterface/include)
INCLUDE_DIRECTORIES(S2P/include)
INCLUDE_DIRECTORIES(wrist_S2P/include)
INCLUDE_DIRECTORIES(StateEstimator/include)
INCLUDE_DIRECTORIES(JointInterface/pnd-cpp-sdk/pnd/include)
INCLUDE_DIRECTORIES(include)
INCLUDE_DIRECTORIES(vnIMU/include)
INCLUDE_DIRECTORIES(vnIMU/include/vn)
INCLUDE_DIRECTORIES(abs)
AUX_SOURCE_DIRECTORY(vnIMU/src vnIMU_DIR)
AUX_SOURCE_DIRECTORY(vnIMU/src/vn vn_DIR)



SET(PND_DIR JointInterface/pnd-cpp-sdk)
SET(pnd_cpp_sdk_build_dir pnd-cpp-sdk-${CMAKE_BUILD_TYPE})
STRING(TOLOWER ${pnd_cpp_sdk_build_dir} pnd_cpp_sdk_build_dir)
GET_FILENAME_COMPONENT(pnd_cpp_sdk_build_dir "${CMAKE_CURRENT_BINARY_DIR}/${pnd_cpp_sdk_build_dir}" REALPATH)

ADD_SUBDIRECTORY(${PND_DIR} ${pnd_cpp_sdk_build_dir})

LINK_DIRECTORIES(JointInterface/pnd-cpp-sdk/pnd/lib/linux_x86_64)

AUX_SOURCE_DIRECTORY(vnIMU/src/vn vn_DIR)

ADD_LIBRARY(${PROJECT_NAME} SHARED
        # StateEstimator/src/StateEstimate.cpp
        # StateEstimator/src/LeggedKalmanFilter.cpp
        JointInterface/src/JointInterface.cpp
        S2P/src/functionIKID_S2P
        wrist_S2P/src/series_parallel
        abs/abs_encoder.cpp
        # vnIMU/src/vnIMU.cpp
        # ${vn_DIR}
        
        RobotInterfaceImpl.cpp
        src/RobotInterface.cpp
        )

TARGET_LINK_LIBRARIES(${PROJECT_NAME}
        PRIVATE pnd pndc++ 
        )

TARGET_LINK_LIBRARIES(${PROJECT_NAME}
        PUBLIC rbdl Qt${QT_VERSION_MAJOR}::Core
        )