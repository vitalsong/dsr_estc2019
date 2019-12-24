cmake_minimum_required(VERSION 3.1)
project(lab1 LANGUAGES C)
set(CMAKE_C_STANDARD 90)

set(BUILD_ROOT ${CMAKE_CURRENT_SOURCE_DIR}/../..)

set(SOURCES main.c)
add_executable(${PROJECT_NAME} ${SOURCES})

add_definitions(-DUSE_STDPERIPH_DRIVER -DSTM32F40_41xxx -DHSE_VALUE=8000000)

set(INCLUDES
    ${BUILD_ROOT}/stm_spl/CMSIS/inc
    ${BUILD_ROOT}/stm_spl/CMSIS/ST/inc
    ${BUILD_ROOT}/stm_spl/CMSIS/ST/inc
    ${BUILD_ROOT}/stm_spl/STM32F4xx/inc
    ${BUILD_ROOT}/common/include
    )

target_include_directories(${PROJECT_NAME} PRIVATE ${INCLUDES})
