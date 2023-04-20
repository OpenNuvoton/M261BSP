set(_SRC  
    timers.c
    list.c
    queue.c
    tasks.c
    portable/GCC/ARM_CM0/port.c
    portable/MemMang/heap_4.c
)


add_library(libfreertos ${_SRC})
target_include_directories(libfreertos 
 PUBLIC
    ${CMAKE_CURRENT_LIST_DIR}
    ${BSP_DIR}/ThirdParty/FreeRTOS/Source/include
    ${BSP_DIR}/ThirdParty/FreeRTOS/Source/portable
    ${BSP_DIR}/ThirdParty/FreeRTOS/Source/portable/GCC/ARM_CM0
    ${BSP_DIR}/ThirdParty/FreeRTOS/Demo/Common/include

)


