# 编译选项模块：GCC 11.4 / Ubuntu 22.04，C++17
# 全部开启警告：-Wall -Wextra -Wpedantic

if(MSVC)
    # MSVC 兼容（本项目主要为 GCC，保留以防将来跨平台）
    add_compile_options(/W4 /permissive-)
else()
    add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# 常用全局定义：隐藏厂商 SDK 中的部分导出宏冲突（如需）
add_compile_definitions($<$<CONFIG:Debug>:_DEBUG>)
