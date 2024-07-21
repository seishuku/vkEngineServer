cmake_minimum_required (VERSION 3.8)

include(FetchContent)

set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED True)

project ("vkEngineServer" LANGUAGES C)

set(PROJECT_SOURCES
	camera/camera.c
	math/math.c
	math/matrix.c
	math/quat.c
	math/vec2.c
	math/vec3.c
	math/vec4.c
	network/network.c
	particle/particle.c
	physics/physics.c
	system/memzone.c
	system/threads.c
	utils/list.c
	utils/lz4.c
	vkEngineServer.c
)

if(CMAKE_BUILD_TYPE MATCHES "Debug")
	add_definitions(-D_DEBUG)
endif()

if(CMAKE_SYSTEM_NAME MATCHES "Windows")
	add_definitions(-DWIN32 -D_CRT_SECURE_NO_WARNINGS -D_CONSOLE)
elseif(CMAKE_SYSTEM_NAME MATCHES "Linux")
	add_definitions(-DLINUX -g)
endif()

if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
    if(CMAKE_SYSTEM_PROCESSOR MATCHES "x86_64|amd64|AMD64")
        add_compile_options(
			"-march=x86-64-v3"
			"-ggdb3"
			"-fprofile-instr-generate"
			"/PROFILE"
		)
    else()
        message(WARNING "Unknown CPU architecture ${CMAKE_SYSTEM_PROCESSOR} not targeted.")
    endif()
elseif(CMAKE_CXX_COMPILER_ID MATCHES "MSVC")
    if(CMAKE_SYSTEM_PROCESSOR MATCHES "AMD64")
        add_compile_options("/arch:AVX2")
        else()
            message(WARNING "Unknown CPU architecture ${CMAKE_SYSTEM_PROCESSOR} not targeted.")
    endif()
endif()

add_executable(${CMAKE_PROJECT_NAME} ${PROJECT_SOURCES})

target_link_libraries(
    ${CMAKE_PROJECT_NAME} PUBLIC
)

if(CMAKE_SYSTEM_NAME MATCHES "Windows")
target_link_libraries(${CMAKE_PROJECT_NAME} PUBLIC ws2_32.lib)
endif()

if(CMAKE_SYSTEM_NAME MATCHES "Linux")
target_link_libraries(${CMAKE_PROJECT_NAME} PUBLIC m)
endif()

if(CMAKE_C_COMPILER_ID MATCHES "MSVC")
target_compile_options(${CMAKE_PROJECT_NAME} PUBLIC /experimental:c11atomics)
endif()
