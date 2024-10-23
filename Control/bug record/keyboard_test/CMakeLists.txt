cmake_minimum_required(VERSION 3.10)

# 项目名称
project(main)

# 设置C++标准
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED True)

# 添加源文件
set(SOURCES
    main.cpp
    keyboard.cpp
)

# 添加可执行文件
add_executable(main ${SOURCES})

# 链接 pthread 库
target_link_libraries(main pthread)