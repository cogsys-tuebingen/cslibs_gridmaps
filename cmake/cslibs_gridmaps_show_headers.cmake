function("${PROJECT_NAME}_show_headers")
    file(GLOB_RECURSE ALL_HEADERS RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} FOLLOW_SYMLINKS *.hpp *.h)
    if(ALL_HEADERS)
        message("[${PROJECT_NAME}]: Adding headers '${ALL_HEADERS}'.")
        add_custom_target(${PROJECT_NAME}_show_header SOURCES ${ALL_HEADERS})
    else()
        message("${PROJECT_NAME}: No headers to show found!")
    endif()
endfunction()
