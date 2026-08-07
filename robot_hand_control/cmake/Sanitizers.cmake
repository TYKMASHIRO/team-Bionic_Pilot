# Sanitizer 模块：AddressSanitizer / UndefinedBehaviorSanitizer
# 通过 ENABLE_SANITIZERS=ON 启用，用于 Debug 构建。

if(ENABLE_SANITIZERS)
    if(NOT CMAKE_BUILD_TYPE STREQUAL "Debug")
        message(WARNING "Sanitizers are recommended for Debug builds only.")
    endif()

    if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
        add_compile_options(-fsanitize=address,undefined -fno-omit-frame-pointer)
        add_link_options(-fsanitize=address,undefined)
    else()
        message(WARNING "Sanitizers not supported for compiler: ${CMAKE_CXX_COMPILER_ID}")
    endif()
endif()
