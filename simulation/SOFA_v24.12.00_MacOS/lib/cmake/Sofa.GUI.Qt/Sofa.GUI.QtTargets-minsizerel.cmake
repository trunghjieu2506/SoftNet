#----------------------------------------------------------------
# Generated CMake target import file for configuration "MinSizeRel".
#----------------------------------------------------------------

# Commands may need to know the format version.
set(CMAKE_IMPORT_FILE_VERSION 1)

# Import target "Sofa.GUI.Qt" for configuration "MinSizeRel"
set_property(TARGET Sofa.GUI.Qt APPEND PROPERTY IMPORTED_CONFIGURATIONS MINSIZEREL)
set_target_properties(Sofa.GUI.Qt PROPERTIES
  IMPORTED_LINK_DEPENDENT_LIBRARIES_MINSIZEREL "tinyxml2::tinyxml2"
  IMPORTED_LOCATION_MINSIZEREL "${_IMPORT_PREFIX}/lib/libSofa.GUI.Qt.24.12.00.dylib"
  IMPORTED_SONAME_MINSIZEREL "@rpath/libSofa.GUI.Qt.24.12.00.dylib"
  )

list(APPEND _cmake_import_check_targets Sofa.GUI.Qt )
list(APPEND _cmake_import_check_files_for_Sofa.GUI.Qt "${_IMPORT_PREFIX}/lib/libSofa.GUI.Qt.24.12.00.dylib" )

# Commands beyond this point should not need to know the version.
set(CMAKE_IMPORT_FILE_VERSION)
