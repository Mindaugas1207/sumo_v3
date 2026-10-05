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
