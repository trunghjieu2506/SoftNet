#----------------------------------------------------------------
# Generated CMake target import file for configuration "MinSizeRel".
#----------------------------------------------------------------

# Commands may need to know the format version.
set(CMAKE_IMPORT_FILE_VERSION 1)

# Import target "image_gui" for configuration "MinSizeRel"
set_property(TARGET image_gui APPEND PROPERTY IMPORTED_CONFIGURATIONS MINSIZEREL)
set_target_properties(image_gui PROPERTIES
  IMPORTED_LOCATION_MINSIZEREL "${_IMPORT_PREFIX}/lib/libimage_gui.24.12.00.dylib"
  IMPORTED_SONAME_MINSIZEREL "@rpath/libimage_gui.24.12.00.dylib"
  )

list(APPEND _cmake_import_check_targets image_gui )
list(APPEND _cmake_import_check_files_for_image_gui "${_IMPORT_PREFIX}/lib/libimage_gui.24.12.00.dylib" )

# Commands beyond this point should not need to know the version.
set(CMAKE_IMPORT_FILE_VERSION)
