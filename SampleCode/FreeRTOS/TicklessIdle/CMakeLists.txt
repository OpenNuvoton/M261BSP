

set(PRJ tickless_idle)


add_executable(${PRJ} 
    main.c 
    ParTest.c
    port.c

    ${BSP_DIR}/ThirdParty/FreeRTOS/Source/timers.c
    ${BSP_DIR}/ThirdParty/FreeRTOS/Source/list.c
    ${BSP_DIR}/ThirdParty/FreeRTOS/Source/queue.c
    ${BSP_DIR}/ThirdParty/FreeRTOS/Source/tasks.c
    ${BSP_DIR}/ThirdParty/FreeRTOS/Source/portable/MemMang/heap_4.c

    ${BSP_DIR}/ThirdParty/FreeRTOS/Demo/Common/Minimal/dynamic.c
    ${BSP_DIR}/ThirdParty/FreeRTOS/Demo/Common/Minimal/recmutex.c
    ${BSP_DIR}/ThirdParty/FreeRTOS/Demo/Common/Minimal/blocktim.c
    ${BSP_DIR}/ThirdParty/FreeRTOS/Demo/Common/Minimal/countsem.c
    ${BSP_DIR}/ThirdParty/FreeRTOS/Demo/Common/Minimal/QueueSet.c
    ${BSP_DIR}/ThirdParty/FreeRTOS/Demo/Common/Minimal/QueueOverwrite.c

)
target_compile_definitions(${PRJ}
    PUBLIC
    
)
target_link_libraries(${PRJ} libstartup libdriver)
target_include_directories(${PRJ} 
 PUBLIC
    ${CMAKE_CURRENT_LIST_DIR}
    ${BSP_DIR}/ThirdParty/FreeRTOS/Source/include
    ${BSP_DIR}/ThirdParty/FreeRTOS/Source/portable
    ${BSP_DIR}/ThirdParty/FreeRTOS/Source/portable/GCC/ARM_CM0
    ${BSP_DIR}/ThirdParty/FreeRTOS/Demo/Common/include

)

target_link_options(${PRJ} 
   PUBLIC 
    --cpu=Cortex-M23
    --library_type=microlib 
    --ro-base 0x00000000
    --rw-base 0x20000000
    --entry Reset_Handler
    --first __Vectors
    --strict
    --map 
    --datacompressor=off 
    --info=inline 
    --entry Reset_Handler 
    --summary_stderr 
    --info totals
    --map 
    --load_addr_map_info 
    --xref 
    --callgraph 
    --symbols
)

# Download to board
set(DOWNLOAD_BOARD TRUE)

# Convert to binary and copy to target device
if(${DOWNLOAD_BOARD})

    add_custom_command(TARGET ${PRJ} POST_BUILD
       COMMAND ${ARM_FROMELF} --bin --output e:/${PRJ}.bin ${PRJ}.elf
       VERBATIM
    )

endif()
