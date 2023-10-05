cmake_minimum_required(VERSION 3.14...3.25)

# Project name, language, version and description
project(QPSK-Libraries LANGUAGES C VERSION 1.0.0 DESCRIPTION "QPSK modulator and demodulator")

# Enable building of shared libraries
option(BUILD_SHARED_LIBS "Build shared libraries (.dll/.so) instead of static ones (.lib/.a)" ON)

# Enable EXPORT_ALL_SYMBOLS
set(CMAKE_WINDOWS_EXPORT_ALL_SYMBOLS YES CACHE BOOL "Export all symbols")

# Add source code
add_library("qpsk_modulator"
    src/qpsk_modulator.c
    src/bytes_symbols_converter.c
    src/rrc_filter.c
    src/butterworth_filter.c
)

# Add source code
add_library("qpsk_demodulator"
    src/qpsk_demodulator.c
    src/bytes_symbols_converter.c
    src/pll.c
    src/rrc_filter.c
    src/butterworth_filter.c
)

# Add "include" directory
target_include_directories("qpsk_modulator" PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)
target_include_directories("qpsk_demodulator" PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)

# Set the version properties
set_target_properties("qpsk_modulator" PROPERTIES VERSION ${PROJECT_VERSION})
set_target_properties("qpsk_modulator" PROPERTIES SOVERSION ${PROJECT_VERSION_MAJOR})
set_target_properties("qpsk_demodulator" PROPERTIES VERSION ${PROJECT_VERSION})
set_target_properties("qpsk_demodulator" PROPERTIES SOVERSION ${PROJECT_VERSION_MAJOR})

# Set output files name
set_target_properties("qpsk_modulator" PROPERTIES OUTPUT_NAME "qpsk_modulator_${PROJECT_VERSION}")
set_target_properties("qpsk_demodulator" PROPERTIES OUTPUT_NAME "qpsk_demodulator_${PROJECT_VERSION}")

# Auto-ignore build directory
if(NOT EXISTS ${PROJECT_BINARY_DIR}/.gitignore)
	file(WRITE ${PROJECT_BINARY_DIR}/.gitignore "*")
endif()
