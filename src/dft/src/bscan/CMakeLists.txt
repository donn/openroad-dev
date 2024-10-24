add_library(dft_bscan_lib
  # Keep sorted
  ScanBoundary.cpp
)

target_link_libraries(dft_bscan_lib
  PRIVATE
    odb
    dbSta
    OpenSTA
    utl_lib
    dft_utils_lib
    dft_config_lib
    dft_reset_domain_lib
    dft_clock_domain_lib
)

target_include_directories(dft_bscan_lib
  PUBLIC
    ${CMAKE_CURRENT_LIST_DIR}
)
