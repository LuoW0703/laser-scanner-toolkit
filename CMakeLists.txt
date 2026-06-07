cmake_minimum_required(VERSION 3.16)
project(LaserScannerToolkit VERSION 1.0.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

include(GNUInstallDirs)
include(CTest)

option(LSC_BUILD_DEMOS "Build demo executables" ON)
option(LSC_BUILD_VALIDATION "Build validation executables" ON)
option(LSC_BUILD_GUI "Build the optional Qt5 GUI" ON)
option(LSC_BUILD_TESTS "Build lightweight unit tests" OFF)
option(LSC_BUILD_BENCHMARKS "Build lightweight performance benchmarks" OFF)
option(LSC_BUILD_FUZZERS "Build fuzz targets (Clang/libFuzzer)" OFF)
option(LSC_BUILD_PYTHON "Build optional Python bindings with pybind11" OFF)
option(LSC_REGISTER_VALIDATION_TESTS "Register long validation executables as CTest tests" OFF)
option(LSC_ENABLE_WARNINGS "Enable strict compiler warnings" ON)
option(LSC_WARNINGS_AS_ERRORS "Treat project warnings as errors" OFF)
option(LSC_ENABLE_ASAN "Enable AddressSanitizer" OFF)
option(LSC_ENABLE_UBSAN "Enable UndefinedBehaviorSanitizer" OFF)
option(LSC_ENABLE_COVERAGE "Enable coverage instrumentation (GCC/Clang)" OFF)

if(MSVC)
    add_compile_options(/utf-8)
endif()

if(MINGW)
    add_compile_options(-finput-charset=UTF-8 -fexec-charset=UTF-8)
endif()

if(LSC_ENABLE_COVERAGE)
    if(MSVC)
        message(WARNING "Coverage instrumentation is not supported by MSVC; ignoring LSC_ENABLE_COVERAGE")
    else()
        add_compile_options(--coverage)
        add_link_options(--coverage)
    endif()
endif()

function(lsc_apply_project_options target)
    if(LSC_ENABLE_WARNINGS)
        if(MSVC)
            target_compile_options(${target} PRIVATE /W4)
            if(LSC_WARNINGS_AS_ERRORS)
                target_compile_options(${target} PRIVATE /WX)
            endif()
        else()
            target_compile_options(${target} PRIVATE -Wall -Wextra -Wpedantic)
            if(LSC_WARNINGS_AS_ERRORS)
                target_compile_options(${target} PRIVATE -Werror)
            endif()
        endif()
    endif()

    if(LSC_ENABLE_ASAN)
        if(MSVC)
            target_compile_options(${target} PRIVATE /fsanitize=address)
        else()
            target_compile_options(${target} PRIVATE -fsanitize=address -fno-omit-frame-pointer)
            target_link_options(${target} PRIVATE -fsanitize=address)
        endif()
    endif()

    if(LSC_ENABLE_UBSAN AND NOT MSVC)
        target_compile_options(${target} PRIVATE -fsanitize=undefined -fno-omit-frame-pointer)
        target_link_options(${target} PRIVATE -fsanitize=undefined)
    elseif(LSC_ENABLE_UBSAN AND MSVC)
        message(WARNING "UndefinedBehaviorSanitizer is not supported by MSVC; ignoring LSC_ENABLE_UBSAN")
    endif()
endfunction()

set(EIGEN_DIR "${CMAKE_SOURCE_DIR}/third_party/eigen")
if(NOT EXISTS "${EIGEN_DIR}/Eigen/Core")
    message(FATAL_ERROR "Eigen3 not found at ${EIGEN_DIR}. Clone with --recursive or initialize submodules.")
endif()

find_package(OpenCV QUIET)
find_package(Qt5 COMPONENTS Widgets OpenGL QUIET)
if(LSC_BUILD_PYTHON)
    find_package(pybind11 QUIET)
endif()

if(OpenCV_FOUND)
    file(GLOB_RECURSE LSC_SOURCES CONFIGURE_DEPENDS "src/*.cpp")

    set(LSC_OPENCV_RUNTIME_DLLS)
    if(WIN32 AND OpenCV_DIR)
        set(LSC_OPENCV_RUNTIME_CANDIDATES
            "${OpenCV_DIR}/bin"
            "${OpenCV_DIR}/../bin"
            "${OpenCV_DIR}/../../bin"
            "${OpenCV_DIR}/x64/vc15/bin"
            "${OpenCV_DIR}/x64/vc16/bin"
        )
        foreach(candidate IN LISTS LSC_OPENCV_RUNTIME_CANDIDATES)
            if(EXISTS "${candidate}")
                file(GLOB LSC_OPENCV_RUNTIME_DLLS "${candidate}/opencv_*.dll")
                if(LSC_OPENCV_RUNTIME_DLLS)
                    break()
                endif()
            endif()
        endforeach()
    endif()

    add_library(lsc STATIC ${LSC_SOURCES})
    add_library(LaserScannerToolkit::lsc ALIAS lsc)

    target_include_directories(lsc
        PUBLIC
            $<BUILD_INTERFACE:${CMAKE_SOURCE_DIR}/include>
            $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>
    )
    target_include_directories(lsc SYSTEM PUBLIC
        $<BUILD_INTERFACE:${EIGEN_DIR}>
        ${OpenCV_INCLUDE_DIRS}
    )

    target_link_libraries(lsc PUBLIC ${OpenCV_LIBRARIES})
    lsc_apply_project_options(lsc)

    function(add_lsc_executable target source_dir)
        add_executable(${target} "${source_dir}/${target}.cpp")
        target_link_libraries(${target} PRIVATE lsc)
        lsc_apply_project_options(${target})
        if(LSC_OPENCV_RUNTIME_DLLS)
            add_custom_command(TARGET ${target} POST_BUILD
                COMMAND ${CMAKE_COMMAND} -E copy_if_different
                    ${LSC_OPENCV_RUNTIME_DLLS}
                    $<TARGET_FILE_DIR:${target}>
                COMMENT "Copy OpenCV runtime DLLs for ${target}")
        endif()
        install(TARGETS ${target} RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR})
    endfunction()

    if(LSC_BUILD_DEMOS)
        add_lsc_executable(demo_full_pipeline demo)
        add_lsc_executable(demo_camera_calib demo)
        add_lsc_executable(demo_light_plane_calib demo)
        add_lsc_executable(demo_processing demo)
    endif()

    if(LSC_BUILD_VALIDATION)
        add_lsc_executable(validate_calibration validation)
        add_lsc_executable(validate_reconstruction validation)
        add_lsc_executable(validate_repeatability validation)
        add_lsc_executable(validate_robustness validation)

        if(BUILD_TESTING AND LSC_REGISTER_VALIDATION_TESTS)
            add_test(NAME validate_calibration COMMAND validate_calibration --smoke)
            add_test(NAME validate_reconstruction COMMAND validate_reconstruction --smoke)
            add_test(NAME validate_repeatability COMMAND validate_repeatability --smoke)
            add_test(NAME validate_robustness COMMAND validate_robustness --smoke)
        endif()
    endif()

    if(LSC_BUILD_TESTS)
        add_executable(test_core tests/test_core.cpp)
        target_link_libraries(test_core PRIVATE lsc)
        lsc_apply_project_options(test_core)
        if(LSC_OPENCV_RUNTIME_DLLS)
            add_custom_command(TARGET test_core POST_BUILD
                COMMAND ${CMAKE_COMMAND} -E copy_if_different
                    ${LSC_OPENCV_RUNTIME_DLLS}
                    $<TARGET_FILE_DIR:test_core>
                COMMENT "Copy OpenCV runtime DLLs for test_core")
        endif()

        if(BUILD_TESTING)
            add_test(NAME test_core COMMAND test_core)
        endif()
    endif()

    if(LSC_BUILD_BENCHMARKS)
        add_executable(bench_filter tests/bench_filter.cpp)
        target_link_libraries(bench_filter PRIVATE lsc)
        lsc_apply_project_options(bench_filter)
        if(BUILD_TESTING)
            add_test(NAME bench_filter COMMAND bench_filter)
            set_tests_properties(bench_filter PROPERTIES TIMEOUT 30 LABELS benchmark)
        endif()
    endif()

    if(LSC_BUILD_FUZZERS)
        if(CMAKE_CXX_COMPILER_ID MATCHES "Clang")
            add_executable(fuzz_ply tests/fuzz_ply.cpp)
            target_link_libraries(fuzz_ply PRIVATE lsc)
            target_compile_options(fuzz_ply PRIVATE -fsanitize=fuzzer,address)
            target_link_options(fuzz_ply PRIVATE -fsanitize=fuzzer,address)
        else()
            message(WARNING "LSC_BUILD_FUZZERS requires Clang/libFuzzer; fuzz targets will not be built")
        endif()
    endif()

    if(LSC_BUILD_PYTHON)
        if(pybind11_FOUND)
            pybind11_add_module(lsc_python python/bindings.cpp)
            target_link_libraries(lsc_python PRIVATE lsc)
            set_target_properties(lsc_python PROPERTIES OUTPUT_NAME lsc)
            lsc_apply_project_options(lsc_python)
        else()
            message(WARNING "pybind11 not found; Python bindings will not be built")
        endif()
    endif()

    install(TARGETS lsc
        EXPORT LaserScannerToolkitTargets
        ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
        LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
        RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
    )
    install(DIRECTORY include/ DESTINATION ${CMAKE_INSTALL_INCLUDEDIR})
else()
    message(STATUS "OpenCV not found; skipping lsc library, demos, and validation targets")
endif()

if(LSC_BUILD_GUI AND Qt5_FOUND)
    message(STATUS "Qt5 found; building GUI application")

    set(GUI_SOURCES
        gui/main.cpp
        gui/EvidenceView.cpp
        gui/ImageView.cpp
        gui/MainWindow.cpp
        gui/PointCloudView.cpp
        gui/PipelineRunner.cpp
        gui/ResultParser.cpp
    )

    add_executable(laser_scanner_gui ${GUI_SOURCES})
    target_link_libraries(laser_scanner_gui PRIVATE Qt5::Widgets Qt5::OpenGL)

    set_target_properties(laser_scanner_gui PROPERTIES
        AUTOMOC ON
        WIN32_EXECUTABLE ON
        OUTPUT_NAME "LaserScannerToolkit"
    )

    if(TARGET demo_full_pipeline)
        add_custom_command(TARGET laser_scanner_gui POST_BUILD
            COMMAND ${CMAKE_COMMAND} -E copy_if_different
                $<TARGET_FILE:demo_full_pipeline>
                $<TARGET_FILE_DIR:laser_scanner_gui>
            COMMENT "Copy demo_full_pipeline to GUI output dir")
    endif()

    if(LSC_BUILD_TESTS)
        add_executable(test_result_parser
            gui/test_result_parser.cpp
            gui/ResultParser.cpp
        )
        target_include_directories(test_result_parser PRIVATE gui)
        target_link_libraries(test_result_parser PRIVATE Qt5::Core)
        lsc_apply_project_options(test_result_parser)
        if(BUILD_TESTING)
            add_test(NAME test_result_parser COMMAND test_result_parser)
        endif()
    endif()

    install(TARGETS laser_scanner_gui RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR})
elseif(LSC_BUILD_GUI)
    message(STATUS "Qt5 not found; GUI application will not be built")
endif()
