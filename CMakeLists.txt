# 以下5行是ESP-IDF工程的固定模板，必须保持这个顺序，CMake才能正常工作
# 指定CMake最低支持版本为3.16，ESP-IDF v5.x要求
cmake_minimum_required(VERSION 3.16)

# EXTRA_COMPONENT_DIRS：额外组件搜索目录
# 告诉ESP-IDF构建系统，去 component/BSP 和 component/MiddleWare 目录查找自定义组件
set(EXTRA_COMPONENT_DIRS component/BSP component/MiddleWare)

# 开启编译日志彩色输出，终端里编译警告/错误会用颜色区分，方便阅读
add_compile_options(-fdiagnostics-color=always)

# 引入ESP-IDF官方的cmake工程构建脚本，这是ESP-IDF核心必须语句
include($ENV{IDF_PATH}/tools/cmake/project.cmake)

# 定义项目名称为 sample_project，会影响生成固件等文件命名
project(sample_project)
