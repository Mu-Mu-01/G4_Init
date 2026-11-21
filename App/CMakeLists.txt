add_library(dpll_app STATIC Src/dpll_app.c)
target_include_directories(dpll_app PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/Inc)
target_link_libraries(dpll_app PUBLIC dpll_nco signal_interface)
