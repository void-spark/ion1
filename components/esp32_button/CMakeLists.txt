if(IDF_VERSION_MAJOR GREATER_EQUAL 6)
    idf_component_register(SRC_DIRS src
        REQUIRES log driver esp_timer esp_driver_gpio
        INCLUDE_DIRS include)
elseif(IDF_VERSION_MAJOR GREATER_EQUAL 4)
    idf_component_register(SRC_DIRS src
        REQUIRES log driver esp_timer
        INCLUDE_DIRS include)
else()
    set(COMPONENT_SRCDIRS src)
    set(COMPONENT_ADD_INCLUDEDIRS include)
    set(COMPONENT_REQUIRES log driver esp_timer)
    register_component()
endif()
