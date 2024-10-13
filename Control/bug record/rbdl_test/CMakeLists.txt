cmake_minimum_required(VERSION 3.0)
project(RBDL_TEST)

# 查找RBDL库
find_package(RBDL REQUIRED)

# 查找Eigen库
find_package(Eigen3 REQUIRED)

# 添加可执行文件
add_executable(read_urdf read_urdf.cpp)
add_executable(example example.cpp)

# 链接RBDL库
target_link_libraries(read_urdf rbdl rbdl_urdfreader)
target_link_libraries(example rbdl)

# 包含RBDL和Eigen的头文件目录
target_include_directories(read_urdf PRIVATE ${RBDL_INCLUDE_DIRS} ${EIGEN3_INCLUDE_DIR})
target_include_directories(example PRIVATE ${RBDL_INCLUDE_DIRS} ${EIGEN3_INCLUDE_DIR})