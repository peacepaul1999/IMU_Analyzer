# ESP-IDF also includes this file in script mode to scan requirements;
# CONFIGURE_DEPENDS and directory properties are invalid there.
if(NOT CMAKE_SCRIPT_MODE_FILE)
    set(WEB_DIR "${CMAKE_CURRENT_LIST_DIR}/../web")
    file(GLOB_RECURSE WEB_TS_SOURCES CONFIGURE_DEPENDS "${WEB_DIR}/ts/*.ts")
    set_property(DIRECTORY APPEND PROPERTY CMAKE_CONFIGURE_DEPENDS
        ${WEB_TS_SOURCES}
        "${WEB_DIR}/package.json"
        "${WEB_DIR}/package-lock.json"
    )

    find_program(NPM_EXECUTABLE
        NAMES npm.cmd npm
        HINTS "$ENV{ProgramFiles}/nodejs"
    )
    if(NOT NPM_EXECUTABLE)
        message(FATAL_ERROR "npm is required to bundle the web TypeScript. Install Node.js/npm and configure again.")
    endif()

    execute_process(
        COMMAND "${NPM_EXECUTABLE}" run build
        WORKING_DIRECTORY "${WEB_DIR}"
        RESULT_VARIABLE WEB_BUILD_RESULT
        COMMAND_ECHO STDOUT
    )
    if(NOT WEB_BUILD_RESULT EQUAL 0)
        message(FATAL_ERROR "Web TypeScript build failed (exit code ${WEB_BUILD_RESULT}).")
    endif()

    file(COPY "${WEB_DIR}/dist/main.js" DESTINATION "${CMAKE_CURRENT_LIST_DIR}/../data")
endif()

idf_component_register(
    SRCS 
    "main.c" 
    "imu_web.c"

    INCLUDE_DIRS 
    "."

    REQUIRES 
    MY_I2C 
    sensors 
    Wifi 
    System_Info 
    driver 
    esp_timer 
    esp_http_server 
    nvs_flash

    EMBED_FILES 
    "../data/index.html" 
    "../data/style.css" 
    "../data/main.js"
)
