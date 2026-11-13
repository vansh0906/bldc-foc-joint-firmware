cmake_minimum_required(VERSION 3.16)
project(bldc_foc_joint C)

set(CMAKE_C_STANDARD 99)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_C_EXTENSIONS OFF)
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release)
endif()
option(SANITIZE "Build with AddressSanitizer + UBSan" OFF)

set(FW_SOURCES
    firmware/src/app.c firmware/src/calib.c firmware/src/ctrl.c firmware/src/encoder.c
    firmware/src/foc.c firmware/src/foc_math.c firmware/src/jbus.c firmware/src/params.c
    firmware/src/protect.c)
set(STRICT_FLAGS -Wall -Wextra -Wpedantic -Wconversion -Wsign-conversion -Wshadow
    -Wdouble-promotion -Wcast-qual -Wstrict-prototypes -Wmissing-prototypes -Werror)

if(CMAKE_CROSSCOMPILING)
    # ---------------- target: STM32G431 + FreeRTOS ----------------
    set(FREERTOS_KERNEL_PATH "" CACHE PATH "FreeRTOS-Kernel checkout (fetched from GitHub if empty)")
    if(NOT FREERTOS_KERNEL_PATH)
        include(FetchContent)
        FetchContent_Declare(freertos_src GIT_REPOSITORY https://github.com/FreeRTOS/FreeRTOS-Kernel.git
                             GIT_TAG V11.2.0 GIT_SHALLOW TRUE)
        FetchContent_Populate(freertos_src)
        set(FREERTOS_KERNEL_PATH ${freertos_src_SOURCE_DIR})
    endif()
    set(RTOS ${FREERTOS_KERNEL_PATH})
    add_executable(firmware.elf ${FW_SOURCES}
        port/stm32g431/main.c port/stm32g431/hal_stm32g4.c port/stm32g431/startup.c port/stm32g431/mem.c
        ${RTOS}/tasks.c ${RTOS}/list.c ${RTOS}/queue.c ${RTOS}/portable/GCC/ARM_CM4F/port.c)
    target_include_directories(firmware.elf PRIVATE port/stm32g431/libc firmware/inc port/stm32g431 ${RTOS}/include
                               ${RTOS}/portable/GCC/ARM_CM4F)
    target_compile_options(firmware.elf PRIVATE -Os -g -fno-math-errno)   # sqrtf -> VSQRT instruction
    # strict flags on our code; main.c includes the kernel headers, so warnings there are not errors
    set_source_files_properties(${FW_SOURCES} port/stm32g431/hal_stm32g4.c port/stm32g431/startup.c
        PROPERTIES COMPILE_OPTIONS "${STRICT_FLAGS}")
    set_source_files_properties(port/stm32g431/main.c PROPERTIES COMPILE_OPTIONS "-Wall;-Wextra")
    if(CMAKE_C_COMPILER_ID STREQUAL "GNU")
        # stop GCC from turning the memset/memcpy loops back into calls to themselves
        set_source_files_properties(port/stm32g431/mem.c PROPERTIES COMPILE_OPTIONS "-fno-tree-loop-distribute-patterns")
    endif()
    target_link_options(firmware.elf PRIVATE -T${CMAKE_SOURCE_DIR}/port/stm32g431/stm32g431.ld
                        -Wl,-Map=${CMAKE_BINARY_DIR}/firmware.map)
    target_link_libraries(firmware.elf PRIVATE ${ARM_LINK_LIBS})
    add_custom_command(TARGET firmware.elf POST_BUILD
        COMMAND ${ARM_OBJCOPY} -O binary firmware.elf firmware.bin
        COMMAND ${ARM_SIZE} firmware.elf)
    return()
endif()

if(SANITIZE)
    add_compile_options(-fsanitize=address,undefined -fno-omit-frame-pointer -fno-sanitize-recover=all)
    add_link_options(-fsanitize=address,undefined)
endif()

# the firmware, built for the host with the same strict flags as for the target
add_library(joint_fw STATIC ${FW_SOURCES})
target_include_directories(joint_fw PUBLIC firmware/inc)
target_compile_options(joint_fw PRIVATE ${STRICT_FLAGS})
set_target_properties(joint_fw PROPERTIES POSITION_INDEPENDENT_CODE ON)

# test equipment: motor/inverter/bus model + simulator, as a shared library for Python
add_library(jointsim SHARED sim/plant.c sim/sim.c)
target_include_directories(jointsim PUBLIC sim)
target_link_libraries(jointsim PUBLIC joint_fw m)
target_compile_definitions(jointsim PRIVATE _GNU_SOURCE)

add_executable(unit_tests tests/unit/test_main.c)
target_link_libraries(unit_tests PRIVATE jointsim)
target_compile_definitions(unit_tests PRIVATE _GNU_SOURCE)

add_executable(pty_sim sim/pty_sim.c)          # simulated joints behind a pseudo-terminal
target_link_libraries(pty_sim PRIVATE jointsim)
if(CMAKE_SYSTEM_NAME STREQUAL "Linux")
    target_link_libraries(pty_sim PRIVATE util)
    target_compile_definitions(pty_sim PRIVATE _GNU_SOURCE)
endif()

enable_testing()
add_test(NAME unit_tests COMMAND unit_tests)
