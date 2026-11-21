add_library(dpll_timer_frequency STATIC
    "${PROJECT_SOURCE_DIR}/myDrivers/Src/timer_frequency.c")
target_include_directories(dpll_timer_frequency PUBLIC
    "${PROJECT_SOURCE_DIR}/myDrivers/Inc")
target_link_libraries(dpll_timer_frequency PRIVATE m)

add_library(signal_port_stm32g4 STATIC Src/signal_port_stm32g4.c)
target_include_directories(signal_port_stm32g4 PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}/Inc")
target_link_libraries(signal_port_stm32g4 PUBLIC signal_interface stm32cubemx
    PRIVATE dpll_timer_frequency)
