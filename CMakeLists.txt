cmake_minimum_required(VERSION 3.5)
project(BlueSkyPilot)

set(CMAKE_C_COMPILER arm-none-eabi-gcc)
set(CMAKE_OBJCOPY arm-none-eabi-objcopy)
set(CMAKE_SIZE arm-none-eabi-size)

add_definitions(
-DSTM32F40_41xxx
-DUSE_STDPERIPH_DRIVER
-DARM_MATH_CM4
)

#set(CMAKE_BUILD_TYPE "Debug")
#set(CMAKE_BUILD_TYPE "Release")
set(MCU_FLAGS "-mcpu=cortex-m4 -mfloat-abi=hard -mfpu=fpv4-sp-d16")
set(CMAKE_C_FLAGS "${MCU_FLAGS} -w -Wno-unknown-pragmas")
set(CMAKE_C_FLAGS_DEBUG "-O0 -g2 -ggdb")
set(CMAKE_C_FLAGS_RELEASE "-O3")

include_directories(
    STMLIB
    STMLIB/inc
    STMLIB/CORE
    STMLIB/USB
    STMLIB/USB/STM32_USB_Device_Library/Class/cdc/inc
    STMLIB/USB/STM32_USB_Device_Library/Core/inc
    STMLIB/USB/STM32_USB_OTG_Driver/inc
    STMLIB/SDIO
    FreeRTOS/Source/include
    FreeRTOS/Source/portable/GCC/ARM_CM4F
    FatFs
    MAVLINK
    MAVLINK/common
    SRC/CONTROL 
    SRC/DRIVER 
    SRC/LOG 
    SRC/MATH 
    SRC/MESSAGE 
    SRC/MODULE 
    SRC/NAVIGATION 
    SRC/SENSOR
    SRC/SYSTEM
    SRC/TASK
    ${CMAKE_CURRENT_BINARY_DIR}
)

set_property(SOURCE STMLIB/startup/GCC/startup_stm32f40_41xxx.s PROPERTY LANGUAGE C)

add_library(stm32_lib
    STMLIB/startup/GCC/startup_stm32f40_41xxx.s
    STMLIB/src/misc.c 
    STMLIB/src/stm32f4xx_adc.c 
    STMLIB/src/stm32f4xx_can.c 
    STMLIB/src/stm32f4xx_dma.c 
    STMLIB/src/stm32f4xx_flash.c 
    STMLIB/src/stm32f4xx_rcc.c 
    STMLIB/src/stm32f4xx_gpio.c 
    STMLIB/src/stm32f4xx_tim.c 
    STMLIB/src/stm32f4xx_spi.c 
    STMLIB/src/stm32f4xx_pwr.c 
    STMLIB/src/stm32f4xx_sdio.c 
    STMLIB/src/stm32f4xx_usart.c 
    STMLIB/src/stm32f4xx_syscfg.c 
    STMLIB/system_stm32f4xx.c
    STMLIB/USB/usb_bsp.c 
    STMLIB/USB/usbd_desc.c
    STMLIB/USB/STM32_USB_Device_Library/Class/cdc/src/usbd_cdc_core.c
    STMLIB/USB/STM32_USB_Device_Library/Core/src/usbd_ioreq.c
    STMLIB/USB/STM32_USB_Device_Library/Core/src/usbd_req.c
    STMLIB/USB/STM32_USB_Device_Library/Core/src/usbd_core.c
    STMLIB/USB/STM32_USB_OTG_Driver/src/usb_core.c
    STMLIB/USB/STM32_USB_OTG_Driver/src/usb_dcd_int.c
    STMLIB/USB/STM32_USB_OTG_Driver/src/usb_dcd.c
)

add_library(freertos
    FreeRTOS/Source/tasks.c 
    FreeRTOS/Source/list.c 
    FreeRTOS/Source/queue.c 
    FreeRTOS/Source/portable/GCC/ARM_CM4F/port.c 
    FreeRTOS/Source/portable/MemMang/heap_4.c
)

add_library(fatfs
    FatFs/diskio.c 
    FatFs/ff.c 
    FatFs/option/ccsbcs.c
    STMLIB/SDIO/stm32f4_sdio_sd_LowLevel.c
    STMLIB/SDIO/stm32f4_sdio_sd.c
)
target_link_libraries(fatfs
    stm32_lib
)

file(GLOB SRC_DRIVER SRC/DRIVER/*.c)
file(GLOB SRC_CONTROL SRC/CONTROL/*.c)
file(GLOB SRC_LOG SRC/LOG/*.c)
file(GLOB SRC_MATH SRC/MATH/*.c)
file(GLOB SRC_MESSAGE SRC/MESSAGE/*.c)
file(GLOB SRC_MODULE SRC/MODULE/*.c)
file(GLOB SRC_NAVIGATION SRC/NAVIGATION/*.c)
file(GLOB SRC_SENSOR SRC/SENSOR/*.c)
file(GLOB SRC_SYSTEM SRC/SYSTEM/*.c)
file(GLOB SRC_TASK SRC/TASK/*.c)

add_library(${PROJECT_NAME}
    ${SRC_DRIVER}
    ${SRC_CONTROL}
    ${SRC_LOG}
    ${SRC_MATH}
    ${SRC_MESSAGE}
    ${SRC_MODULE}
    ${SRC_NAVIGATION}
    ${SRC_SENSOR}
    ${SRC_SYSTEM}
    ${SRC_TASK}
)
target_link_libraries(${PROJECT_NAME} -lm
    stm32_lib
    fatfs
    freertos
)

set(LINKER_SCRIPT ${CMAKE_CURRENT_SOURCE_DIR}/STMLIB/STM32F405RGTx_FLASH.ld)
set(CMAKE_EXE_LINKER_FLAGS
"--specs=nano.specs -specs=nosys.specs -nostartfiles -T${LINKER_SCRIPT} -Wl,-Map=${PROJECT_BINARY_DIR}/${PROJECT_NAME}.map,--cref -Wl,--gc-sections"
)

add_executable(${PROJECT_NAME}.elf SRC/main.c)
target_link_libraries(${PROJECT_NAME}.elf
    ${PROJECT_NAME}
)

set(ELF_FILE ${PROJECT_BINARY_DIR}/${PROJECT_NAME}.elf)
set(HEX_FILE ${PROJECT_BINARY_DIR}/${PROJECT_NAME}.hex)
set(BIN_FILE ${PROJECT_BINARY_DIR}/${PROJECT_NAME}.bin)

add_custom_command(TARGET "${PROJECT_NAME}.elf" POST_BUILD
    COMMAND ${CMAKE_OBJCOPY} -Obinary ${ELF_FILE} ${BIN_FILE}
    COMMAND ${CMAKE_OBJCOPY} -Oihex  ${ELF_FILE} ${HEX_FILE}
    COMMENT "Building ${PROJECT_NAME}.bin and ${PROJECT_NAME}.hex"

    COMMAND ${CMAKE_COMMAND} -E copy ${HEX_FILE} "${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}.hex"
    COMMAND ${CMAKE_COMMAND} -E copy ${BIN_FILE} "${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}.bin"

    COMMAND ${CMAKE_SIZE} --format=berkeley ${PROJECT_NAME}.elf ${PROJECT_NAME}.hex
    COMMENT "Invoking: Cross ARM GNU Print Size"
)
