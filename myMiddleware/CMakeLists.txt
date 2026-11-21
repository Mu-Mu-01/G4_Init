# 数学模块与 NCO 服务分目标，只有 NCO 服务需要运行环境接口。
add_library(dpll_algorithms STATIC
    Src/dpll_pi.c Src/dpll_phase_calibration.c Src/iq_phase.c
    Src/nco_frequency_multiplier.c Src/nco_phase_interp.c Src/nco_waveform.c
)
target_include_directories(dpll_algorithms PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/Inc)
target_link_libraries(dpll_algorithms PRIVATE m)

add_library(dpll_nco STATIC Src/nco_dac.c)
target_include_directories(dpll_nco PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/Inc)
target_link_libraries(dpll_nco PUBLIC dpll_algorithms signal_interface)
