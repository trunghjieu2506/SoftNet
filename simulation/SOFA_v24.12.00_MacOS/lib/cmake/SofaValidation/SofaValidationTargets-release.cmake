#----------------------------------------------------------------
# Generated CMake target import file for configuration "Release".
#----------------------------------------------------------------

# Commands may need to know the format version.
set(CMAKE_IMPORT_FILE_VERSION 1)

# Import target "SofaValidation" for configuration "Release"
set_property(TARGET SofaValidation APPEND PROPERTY IMPORTED_CONFIGURATIONS RELEASE)
set_target_properties(SofaValidation PROPERTIES
  IMPORTED_LOCATION_RELEASE "${_IMPORT_PREFIX}/lib/libSofaValidation.24.12.00.dylib"
  IMPORTED_SONAME_RELEASE "@rpath/libSofaValidation.24.12.00.dylib"
  )

list(APPEND _cmake_import_check_targets SofaValidation )
list(APPEND _cmake_import_check_files_for_SofaValidation "${_IMPORT_PREFIX}/lib/libSofaValidation.24.12.00.dylib" )

# Commands beyond this point should not need to know the version.
set(CMAKE_IMPORT_FILE_VERSION)
