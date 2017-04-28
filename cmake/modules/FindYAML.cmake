# Locate YAML 
#
# This module defines
#  YAML_FOUND, if false, do not try to link to yaml-cpp
#  YAML_LIBRARY, where to find yaml-cpp
#  YAML_INCLUDE_DIR, where to find yaml.h

# find the LibSerial include directory
find_path(YAML_INCLUDE_DIR yaml.h
          PATH_SUFFIXES include
          PATHS /usr/local/yaml-cpp)

# find the LibSerial library
find_library(YAML_LIBRARY
             NAMES libyaml-cpp.so
             PATH_SUFFIXES lib
             PATHS /usr/local/)

 # handle the QUIETLY and REQUIRED arguments and set YAML_FOUND to TRUE if all listed variables are TRUE
include(FindPackageHandleStandardArgs)
FIND_PACKAGE_HANDLE_STANDARD_ARGS(YAML DEFAULT_MSG YAML_INCLUDE_DIR YAML_LIBRARY)
mark_as_advanced(YAML_INCLUDE_DIR YAML_LIBRARY)
