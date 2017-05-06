# This module finds an installed GLPK package.
#
# It sets the following variables:
#  GLPK_FOUND              - Set to true if GLPK is found.
#  GLPK_INCLUDE_DIR        - GLPK include directory.
#  GLPK_LIBRARY            - GLPK library file.
FIND_PATH(GLPK_INCLUDE_DIR
  glpk.h
  PATHS /usr/include/ /usr/include/glpk $ENV{GLPK_DIR}/include
  DOC "Directory where GLPK header files are stored")
find_library(GLPK_LIBRARY NAMES glpk PATHS /usr/lib $ENV{GLPK_DIR}/lib LIBRARY_PATH LD_LIBRARY_PATH)

# handle the QUIETLY and REQUIRED arguments and set GLPK_FOUND to TRUE if 
# all listed variables are TRUE
INCLUDE(FindPackageHandleStandardArgs)
FIND_PACKAGE_HANDLE_STANDARD_ARGS(GLPK DEFAULT_MSG GLPK_LIBRARY GLPK_INCLUDE_DIR)

MESSAGE(STATUS "Found GLPK at ${GLPK_LIBRARY}")
MARK_AS_ADVANCED(GLPK_INCLUDE_DIR GLPK_LIBRARY GLPK_LIBRARY_DIR)