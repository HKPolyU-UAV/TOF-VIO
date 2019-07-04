# search for royale SDK
# find all `share` directories in `royale` directory
execute_process(COMMAND "${PROJECT_SOURCE_DIR}/cmake/findPicoFlexxSDK.py" "${PROJECT_SOURCE_DIR}/royale/" RESULT_VARIABLE ERROR_CODE OUTPUT_VARIABLE PATHS_STRING)

if(ERROR_CODE EQUAL 0)
  if(PATHS_STRING)
    string(REPLACE "\n" ";" PATHS_LIST ${PATHS_STRING})

    # store CXX flags to override the settings from royale later
    # The royale sdk cmake file overrides the variable CMAKE_CXX_FLAGS
    set(CMAKE_CXX_FLAGS_OLD "${CMAKE_CXX_FLAGS}")

    find_package(royale REQUIRED
      PATHS ${PATHS_LIST}
      NO_DEFAULT_PATH
    )

    # also remove CMAKE_CXX_FLAGS from the cache, 
    # or the changes done by find_package(royale) could show up later again
    unset(CMAKE_CXX_FLAGS CACHE)
    # get the untouched CXX_FLAGS
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS_OLD}")
  endif()
endif()

FIND_PACKAGE_HANDLE_STANDARD_ARGS(royale
  REQUIRED_VARS royale_LIB_DIR royale_LIBRARIES royale_INCLUDE_DIRS
  FAIL_MESSAGE "Could not find royale SDK! please make sure to extract the royale SDK to ${PROJECT_SOURCE_DIR}/royale/"
)
