cmake_minimum_required(VERSION 3.0)

set(target softi2c-test)

file(GLOB_RECURSE sources *.c *.h *.in)

add_definitions( -D_XOPEN_SOURCE=500 )
add_definitions( -D_POSIX_C_SOURCE=199309L )

add_executable(${target} ${sources})
target_include_directories(${target} PUBLIC /usr/local/include)
target_link_libraries(${target} -lsoc)

