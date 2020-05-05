#  argos3core_simulator
#  argos3plugin_simulator_genericrobot)

if(ARGOS_BUILD_FOR_SIMULATOR)
    add_library(epuck_beacon MODULE epuck_beacon.h
                                            epuck_beacon.cpp)
    target_link_libraries(epuck_beacon
                          argos3plugin_${ARGOS_BUILD_FOR}_epuck)


    else()
    add_executable(epuck_beacon
      epuck_beacon.h
      epuck_beacon.cpp)
    target_link_libraries(epuck_beacon
                          argos3plugin_${ARGOS_BUILD_FOR}_epuck)

endif(ARGOS_BUILD_FOR_SIMULATOR)
