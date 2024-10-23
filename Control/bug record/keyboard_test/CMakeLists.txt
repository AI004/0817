# 设置 CMake 的最低版本要求
cmake_minimum_required(VERSION 3.0)

# 设置项目名称和语言
project(main)

# 设置 C 编译器的标准
set(CMAKE_C_STANDARD 99)

# 添加可执行文件
add_executable(main main.cpp)

# 如果需要链接特定的库（例如 termios），可以在这里添加
# target_link_libraries(keyboard_example termios)

# 如果需要包含特定的头文件目录，可以在这里添加
# include_directories(/path/to/headers)