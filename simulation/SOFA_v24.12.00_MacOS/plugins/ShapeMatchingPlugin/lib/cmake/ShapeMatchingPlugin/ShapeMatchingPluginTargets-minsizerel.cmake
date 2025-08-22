#----------------------------------------------------------------
# Generated CMake target import file for configuration "MinSizeRel".
#----------------------------------------------------------------

# Commands may need to know the format version.
set(CMAKE_IMPORT_FILE_VERSION 1)

# Import target "ShapeMatchingPlugin" for configuration "MinSizeRel"
set_property(TARGET ShapeMatchingPlugin APPEND PROPERTY IMPORTED_CONFIGURATIONS MINSIZEREL)
set_target_properties(ShapeMatchingPlugin PROPERTIES
  IMPORTED_LOCATION_MINSIZEREL "${_IMPORT_PREFIX}/lib/libShapeMatchingPlugin.24.12.00.dylib"
  IMPORTED_SONAME_MINSIZEREL "@rpath/libShapeMatchingPlugin.24.12.00.dylib"
  )

list(APPEND _cmake_import_check_targets ShapeMatchingPlugin )
list(APPEND _cmake_import_check_files_for_ShapeMatchingPlugin "${_IMPORT_PREFIX}/lib/libShapeMatchingPlugin.24.12.00.dylib" )

# Commands beyond this point should not need to know the version.
set(CMAKE_IMPORT_FILE_VERSION)
