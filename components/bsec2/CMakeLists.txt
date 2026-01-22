idf_component_register(
  SRCS
    "src/bsec2.cpp"
  INCLUDE_DIRS
    "src"
    "src/inc"
  REQUIRES
    BME68x_Sensor_library
)

# Link the prebuilt Bosch algorithm library for the current target
set(BSEC_LIB_DIR "${CMAKE_CURRENT_LIST_DIR}/src/${IDF_TARGET}")
set(BSEC_LIB     "${BSEC_LIB_DIR}/libalgobsec.a")

if(EXISTS "${BSEC_LIB}")
  message(STATUS "Using BSEC prebuilt: ${BSEC_LIB}")
  target_link_libraries(${COMPONENT_LIB} INTERFACE "${BSEC_LIB}")
else()
  message(FATAL_ERROR "BSEC prebuilt lib not found for IDF_TARGET='${IDF_TARGET}'. Expected: ${BSEC_LIB}")
endif()
