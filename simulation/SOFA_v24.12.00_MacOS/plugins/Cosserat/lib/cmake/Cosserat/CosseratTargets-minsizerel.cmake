#----------------------------------------------------------------
# Generated CMake target import file for configuration "MinSizeRel".
#----------------------------------------------------------------

# Commands may need to know the format version.
set(CMAKE_IMPORT_FILE_VERSION 1)

# Import target "CosseratBindings" for configuration "MinSizeRel"
set_property(TARGET CosseratBindings APPEND PROPERTY IMPORTED_CONFIGURATIONS MINSIZEREL)
set_target_properties(CosseratBindings PROPERTIES
  IMPORTED_LINK_DEPENDENT_LIBRARIES_MINSIZEREL "Python::Python"
  IMPORTED_LOCATION_MINSIZEREL "${_IMPORT_PREFIX}/lib/python3/site-packages/Cosserat.cpython-312-darwin.so"
  IMPORTED_SONAME_MINSIZEREL "@rpath/python3/site-packages///Cosserat.cpython-312-darwin.so"
  )

list(APPEND _cmake_import_check_targets CosseratBindings )
list(APPEND _cmake_import_check_files_for_CosseratBindings "${_IMPORT_PREFIX}/lib/python3/site-packages/Cosserat.cpython-312-darwin.so" )

# Import target "Cosserat" for configuration "MinSizeRel"
set_property(TARGET Cosserat APPEND PROPERTY IMPORTED_CONFIGURATIONS MINSIZEREL)
set_target_properties(Cosserat PROPERTIES
  IMPORTED_LOCATION_MINSIZEREL "${_IMPORT_PREFIX}/lib/libCosserat.21.12.0.dylib"
  IMPORTED_SONAME_MINSIZEREL "@rpath/libCosserat.21.12.0.dylib"
  )

list(APPEND _cmake_import_check_targets Cosserat )
list(APPEND _cmake_import_check_files_for_Cosserat "${_IMPORT_PREFIX}/lib/libCosserat.21.12.0.dylib" )

# Commands beyond this point should not need to know the version.
set(CMAKE_IMPORT_FILE_VERSION)
