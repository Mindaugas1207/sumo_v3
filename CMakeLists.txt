# Generated Cmake Pico project file

cmake_minimum_required(VERSION 3.13)

set(CMAKE_C_STANDARD 17)
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# Initialise pico_sdk from installed location
# (note this can come from environment, CMake cache etc)

# == DO NOT EDIT THE FOLLOWING LINES for the Raspberry Pi Pico VS Code Extension to work ==
if(WIN32)
    set(USERHOME $ENV{USERPROFILE})
else()
    set(USERHOME $ENV{HOME})
endif()
set(sdkVersion 2.3.1)
set(toolchainVersion 14_2_Rel1)
set(picotoolVersion 2.3.1)
set(picoVscode ${USERHOME}/.pico-sdk/cmake/pico-vscode.cmake)
if (EXISTS ${picoVscode})
    include(${picoVscode})
endif()
# ====================================================================================
set(PICO_BOARD pico2 CACHE STRING "Board type")

# Pull in Raspberry Pi Pico SDK (must be before project)
include(pico_sdk_import.cmake)

project(mini-sumo-v2 C CXX ASM)

set(PICO_CXX_ENABLE_EXCEPTIONS 1)

option(MINI_SUMO_PREFER_PATH_PYTHON "Prefer python from PATH over Pico SDK bundled Python" ON)

if(WIN32 AND MINI_SUMO_PREFER_PATH_PYTHON)
        find_program(MINI_SUMO_PATH_PYTHON NAMES python python3)
        if(MINI_SUMO_PATH_PYTHON)
                set(Python3_EXECUTABLE "${MINI_SUMO_PATH_PYTHON}" CACHE FILEPATH "Python 3 interpreter" FORCE)
        endif()
endif()

find_package(Python3 REQUIRED COMPONENTS Interpreter)
message(STATUS "Using Python interpreter: ${Python3_EXECUTABLE}")

option(MINI_SUMO_AUTO_INSTALL_PILLOW "Automatically pip install Pillow if missing" ON)

execute_process(COMMAND "${Python3_EXECUTABLE}" -c "import PIL"
        RESULT_VARIABLE MINI_SUMO_PILLOW_RESULT OUTPUT_QUIET ERROR_QUIET)
if(NOT MINI_SUMO_PILLOW_RESULT EQUAL 0)
        if(MINI_SUMO_AUTO_INSTALL_PILLOW)
                message(STATUS "Pillow not found, installing with pip...")
                execute_process(COMMAND "${Python3_EXECUTABLE}" -m pip install pillow
                        RESULT_VARIABLE MINI_SUMO_PIP_RESULT)
                execute_process(COMMAND "${Python3_EXECUTABLE}" -c "import PIL"
                        RESULT_VARIABLE MINI_SUMO_PILLOW_RESULT OUTPUT_QUIET ERROR_QUIET)
        endif()
        if(NOT MINI_SUMO_PILLOW_RESULT EQUAL 0)
                message(FATAL_ERROR
                        "Python module 'Pillow' is required for asset generation but is not installed for ${Python3_EXECUTABLE}.\n"
                        "Install it with: \"${Python3_EXECUTABLE}\" -m pip install pillow\n"
                        "or configure with -DMINI_SUMO_AUTO_INSTALL_PILLOW=ON.")
        endif()
endif()

get_filename_component(MINI_SUMO_PYTHON_DIR "${Python3_EXECUTABLE}" DIRECTORY)
set(MINI_SUMO_PYTHON_SITE_PACKAGES "${MINI_SUMO_PYTHON_DIR}/Lib/site-packages")

# Initialise the Raspberry Pi Pico SDK
pico_sdk_init()

# Add executable. Default name is the project name, version 0.1

add_executable(mini-sumo-v2 src/main.cpp src/hardware.cpp src/display.cpp src/remote.cpp src/motion.cpp src/sensors.cpp
        )

set(MINI_SUMO_IMAGE_SOURCE_DIR ${CMAKE_SOURCE_DIR}/resources/images)
set(MINI_SUMO_GENERATED_IMAGE_DIR ${CMAKE_CURRENT_BINARY_DIR}/generated)
set(MINI_SUMO_GENERATED_IMAGE_HEADER ${MINI_SUMO_GENERATED_IMAGE_DIR}/image_assets.h)

set(MINI_SUMO_FONT_SOURCE_DIR ${CMAKE_SOURCE_DIR}/resources/fonts)
set(MINI_SUMO_GENERATED_FONT_DIR ${CMAKE_CURRENT_BINARY_DIR}/generated)
set(MINI_SUMO_GENERATED_FONT_HEADER ${MINI_SUMO_GENERATED_FONT_DIR}/font_assets.h)

file(GLOB MINI_SUMO_IMAGE_SOURCES CONFIGURE_DEPENDS
        "${MINI_SUMO_IMAGE_SOURCE_DIR}/*.pbm"
        "${MINI_SUMO_IMAGE_SOURCE_DIR}/*.pgm"
        "${MINI_SUMO_IMAGE_SOURCE_DIR}/*.ppm"
        "${MINI_SUMO_IMAGE_SOURCE_DIR}/*.png"
        "${MINI_SUMO_IMAGE_SOURCE_DIR}/*.bmp"
        "${MINI_SUMO_IMAGE_SOURCE_DIR}/*.jpg"
        "${MINI_SUMO_IMAGE_SOURCE_DIR}/*.jpeg"
)

file(GLOB MINI_SUMO_FONT_SOURCES CONFIGURE_DEPENDS
        "${MINI_SUMO_FONT_SOURCE_DIR}/*.otf"
        "${MINI_SUMO_FONT_SOURCE_DIR}/*.ttf"
        "${MINI_SUMO_FONT_SOURCE_DIR}/*.ttc"
)

add_custom_command(
        OUTPUT ${MINI_SUMO_GENERATED_IMAGE_HEADER}
        COMMAND ${CMAKE_COMMAND} -E make_directory ${MINI_SUMO_GENERATED_IMAGE_DIR}
        COMMAND ${CMAKE_COMMAND} -E env "PYTHONPATH=${MINI_SUMO_PYTHON_SITE_PACKAGES}" ${Python3_EXECUTABLE} ${CMAKE_CURRENT_LIST_DIR}/tools/generate_image_headers.py
                --output ${MINI_SUMO_GENERATED_IMAGE_HEADER}
                ${MINI_SUMO_IMAGE_SOURCES}
        DEPENDS ${CMAKE_CURRENT_LIST_DIR}/tools/generate_image_headers.py ${MINI_SUMO_IMAGE_SOURCES}
        VERBATIM
)

add_custom_target(mini-sumo-v2-images DEPENDS ${MINI_SUMO_GENERATED_IMAGE_HEADER})

add_custom_command(
        OUTPUT ${MINI_SUMO_GENERATED_FONT_HEADER}
        COMMAND ${CMAKE_COMMAND} -E make_directory ${MINI_SUMO_GENERATED_FONT_DIR}
        COMMAND ${CMAKE_COMMAND} -E env "PYTHONPATH=${MINI_SUMO_PYTHON_SITE_PACKAGES}" ${Python3_EXECUTABLE} ${CMAKE_CURRENT_LIST_DIR}/tools/generate_font_headers.py
                --output ${MINI_SUMO_GENERATED_FONT_HEADER}
                --sizes 6,12
                --tight-sizes 6
                ${MINI_SUMO_FONT_SOURCES}
        DEPENDS ${CMAKE_CURRENT_LIST_DIR}/tools/generate_font_headers.py ${MINI_SUMO_FONT_SOURCES}
        VERBATIM
)

add_custom_target(mini-sumo-v2-fonts DEPENDS ${MINI_SUMO_GENERATED_FONT_HEADER})

pico_set_program_name(mini-sumo-v2 "mini-sumo-v2")
pico_set_program_version(mini-sumo-v2 "0.1")

add_dependencies(mini-sumo-v2 mini-sumo-v2-images)
add_dependencies(mini-sumo-v2 mini-sumo-v2-fonts)

# Generate PIO header
pico_generate_pio_header(mini-sumo-v2 ${CMAKE_CURRENT_LIST_DIR}/src/one_wire.pio)
pico_generate_pio_header(mini-sumo-v2 ${CMAKE_CURRENT_LIST_DIR}/src/ws2812.pio)

# Modify the below lines to enable/disable output over UART/USB
pico_enable_stdio_uart(mini-sumo-v2 0)
pico_enable_stdio_usb(mini-sumo-v2 1)

# Add the standard library to the build
target_link_libraries(mini-sumo-v2
        pico_stdlib
        )

# Add the standard include files to the build
target_include_directories(mini-sumo-v2 PRIVATE
        ${CMAKE_CURRENT_LIST_DIR}
        ${MINI_SUMO_GENERATED_IMAGE_DIR}
        ${CMAKE_SOURCE_DIR}/include
        ${CMAKE_SOURCE_DIR}/include/hal/rp2350
        ${CMAKE_SOURCE_DIR}/include/drivers
        ${CMAKE_SOURCE_DIR}/include/drivers/interfaces
        ${CMAKE_SOURCE_DIR}/include/drivers/hardware
        ${CMAKE_SOURCE_DIR}/include/drivers/graphics
        ${CMAKE_SOURCE_DIR}/include/drivers/graphics/components
        ${CMAKE_SOURCE_DIR}/include/app
)

# Add any user requested libraries
target_link_libraries(mini-sumo-v2 
        hardware_i2c
        hardware_pio
        hardware_pwm
        hardware_flash 
        hardware_sync 
        pico_multicore
        )

pico_add_extra_outputs(mini-sumo-v2)

if(NOT CMAKE_SIZE AND CMAKE_C_COMPILER)
        get_filename_component(PICO_TOOLCHAIN_BIN_DIR ${CMAKE_C_COMPILER} DIRECTORY)
        find_program(CMAKE_SIZE arm-none-eabi-size HINTS ${PICO_TOOLCHAIN_BIN_DIR})
endif()

if(NOT CMAKE_NM AND CMAKE_C_COMPILER)
        get_filename_component(PICO_TOOLCHAIN_BIN_DIR ${CMAKE_C_COMPILER} DIRECTORY)
        find_program(CMAKE_NM arm-none-eabi-nm HINTS ${PICO_TOOLCHAIN_BIN_DIR})
endif()

file(TO_CMAKE_PATH "${CMAKE_SOURCE_DIR}" MINI_SUMO_PROJECT_ROOT)

set(MINI_SUMO_SYMBOL_REPORT_SCRIPT ${CMAKE_CURRENT_BINARY_DIR}/mini-sumo-symbol-report.cmake)

file(GENERATE OUTPUT ${MINI_SUMO_SYMBOL_REPORT_SCRIPT} CONTENT [=[
cmake_policy(SET CMP0053 NEW)

execute_process(
        COMMAND "${CMAKE_NM}" --demangle --line-numbers --print-size --size-sort --reverse-sort --radix=d "${ELF_FILE}"
        RESULT_VARIABLE nm_result
        OUTPUT_VARIABLE nm_output
        ERROR_VARIABLE nm_error
        OUTPUT_STRIP_TRAILING_WHITESPACE
)

if(NOT nm_result EQUAL 0)
        message(WARNING "Failed to generate symbol size report: ${nm_error}")
        return()
endif()

string(REPLACE "\r\n" "\n" nm_output "${nm_output}")
string(REPLACE "\n" ";" nm_lines "${nm_output}")

message("Top 10 largest project symbols")
message("    Size  Type  Name")

set(symbol_count 0)
foreach(nm_line IN LISTS nm_lines)
        if(nm_line STREQUAL "")
                continue()
        endif()

        string(REPLACE "\\" "/" nm_line_normalized "${nm_line}")
        string(FIND "${nm_line_normalized}" "${PROJECT_ROOT}" project_root_index)
        if(project_root_index LESS 0)
                continue()
        endif()

        string(SUBSTRING "${nm_line_normalized}" 0 ${project_root_index} nm_metadata)
        string(STRIP "${nm_metadata}" nm_metadata)

        string(REGEX MATCH "^[0-9]+ +([0-9]+) +([A-Za-z]) +(.*)$" nm_match "${nm_metadata}")
        if(NOT nm_match)
                continue()
        endif()

        set(symbol_size "${CMAKE_MATCH_1}")
        set(symbol_type "${CMAKE_MATCH_2}")
        set(symbol_name "${CMAKE_MATCH_3}")

        if(symbol_name MATCHES "^_GLOBAL__sub_I_" OR symbol_name MATCHES "^__static_initialization_and_destruction_0")
                continue()
        endif()

        string(REGEX REPLACE "^0+" "" symbol_size "${symbol_size}")
        if(symbol_size STREQUAL "")
                set(symbol_size "0")
        endif()

        string(LENGTH "${symbol_size}" symbol_size_length)
        math(EXPR symbol_size_padding "8 - ${symbol_size_length}")
        if(symbol_size_padding GREATER 0)
                string(REPEAT " " ${symbol_size_padding} symbol_size_prefix)
        else()
                set(symbol_size_prefix "")
        endif()

        math(EXPR symbol_count "${symbol_count} + 1")
        message("${symbol_size_prefix}${symbol_size}  ${symbol_type}     ${symbol_name}")

        if(symbol_count GREATER_EQUAL 10)
                break()
        endif()
endforeach()

if(symbol_count EQUAL 0)
        message("No project-local symbols with source paths were found.")
endif()
]=])

if(CMAKE_SIZE)
        add_custom_target(mini-sumo-v2-size-report ALL
                        COMMAND ${CMAKE_COMMAND} -E echo "Firmware size summary for $<TARGET_FILE_NAME:mini-sumo-v2>"
                        COMMAND ${CMAKE_SIZE} $<TARGET_FILE:mini-sumo-v2>
                        COMMAND ${CMAKE_COMMAND}
                                -DCMAKE_NM=${CMAKE_NM}
                                -DELF_FILE=$<TARGET_FILE:mini-sumo-v2>
                                -DPROJECT_ROOT=${MINI_SUMO_PROJECT_ROOT}
                                -P ${MINI_SUMO_SYMBOL_REPORT_SCRIPT}
                        DEPENDS mini-sumo-v2
                        VERBATIM
        )
else()
        message(STATUS "CMAKE_SIZE is not set; post-build memory usage reporting is disabled.")
endif()

