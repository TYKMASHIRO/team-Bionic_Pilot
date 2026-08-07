# 依赖管理模块
# 规则：第三方依赖集中管理，不得在各子模块中分别下载同一依赖。
#
# 注意：本环境检测到 miniconda 提供了 yaml-cpp/gtest（GCC 12 构建），
# 与系统 GCC 11.4 的 libstdc++ ABI 不兼容（GLIBCXX_3.4.30 缺失）。
# 因此一律用 FetchContent 从源码构建，保证 ABI 与系统编译器一致。

find_package(Threads REQUIRED)
include(FetchContent)

# ---------------------------------------------------------------------------
# yaml-cpp：配置解析（从源码构建）
# ---------------------------------------------------------------------------
set(YAML_CPP_BUILD_TESTS OFF CACHE BOOL "" FORCE)
set(YAML_CPP_BUILD_TOOLS OFF CACHE BOOL "" FORCE)
set(YAML_CPP_INSTALL OFF CACHE BOOL "" FORCE)
FetchContent_Declare(yaml-cpp
    GIT_REPOSITORY https://github.com/jbeder/yaml-cpp.git
    GIT_TAG 0.8.0
    GIT_SHALLOW TRUE)
FetchContent_MakeAvailable(yaml-cpp)
set(YAML_CPP_TARGET yaml-cpp)
if(NOT TARGET ${YAML_CPP_TARGET})
    message(FATAL_ERROR "yaml-cpp target not available")
endif()
message(STATUS "yaml-cpp: built from source")

# ---------------------------------------------------------------------------
# GoogleTest：单元/集成测试框架（从源码构建）
# ---------------------------------------------------------------------------
if(BUILD_TESTING)
    set(INSTALL_GTEST OFF CACHE BOOL "" FORCE)
    set(BUILD_GMOCK ON CACHE BOOL "" FORCE)
    FetchContent_Declare(googletest
        GIT_REPOSITORY https://github.com/google/googletest.git
        GIT_TAG v1.14.0
        GIT_SHALLOW TRUE)
    FetchContent_MakeAvailable(googletest)
    set(GTEST_TARGET GTest::gtest_main)
    message(STATUS "gtest: built from source")
endif()
