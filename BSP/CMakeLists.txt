if(TARGET signal_port_stm32g4)
    set(bsp_signal_source Src/bsp_signal_stm32g4.c)
    set(bsp_signal_port signal_port_stm32g4)
else()
    set(bsp_signal_source Src/bsp_signal.c)
    set(bsp_signal_port signal_port_stm32h7)
endif()

add_library(dpll_bsp STATIC ${bsp_signal_source})
target_include_directories(dpll_bsp PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/Inc)
target_link_libraries(dpll_bsp PUBLIC signal_interface
    PRIVATE ${bsp_signal_port} stm32cubemx)
