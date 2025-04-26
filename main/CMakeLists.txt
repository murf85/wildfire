add_executable(${projectName} main.cpp)

target_include_directories(${projectName} PUBLIC
    "."
    "include"
)
target_compile_options(${projectName} PUBLIC -std=gnu++2b)
