add_library(signal_port_stm32h7 STATIC Src/signal_port_stm32h7.c)
target_include_directories(signal_port_stm32h7 PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/Inc)
target_link_libraries(signal_port_stm32h7 PUBLIC signal_interface
    PRIVATE dpll_stm32_driver stm32cubemx)
