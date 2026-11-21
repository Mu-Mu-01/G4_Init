# 仅自有 HAL 扩展；厂商 Drivers/ 由 CubeMX 目标继续管理。
add_library(dpll_stm32_driver STATIC Src/adc_Doublebuffer.c Src/dac_Doublebuffer.c
    Src/timer_frequency_stm32h7.c ../Src/timer_frequency.c)
target_include_directories(dpll_stm32_driver PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/Inc
    ${CMAKE_CURRENT_SOURCE_DIR}/../Inc)
target_link_libraries(dpll_stm32_driver PUBLIC stm32cubemx)
