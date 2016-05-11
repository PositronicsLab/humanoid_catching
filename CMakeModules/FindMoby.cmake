# Find Ravelin header and library.
#

# This module defines the following uncached variables:
#  RAVELIN_FOUND, if false, do not try to use Ravelin.
#  RAVELIN_INCLUDE_DIRS, where to find Ravelin/Ravelin_a.h.
#  RAVELIN_LIBRARIES, the libraries to link against to use the Ravelin library
#  RAVELIN_LIBRARY_DIRS, the directory where the Ravelin library is found.

find_path(
  MOBY_INCLUDE_DIR
  Moby/AABB.h 
  PATHS /usr/local/include /usr/include
)

if( MOBY_INCLUDE_DIR )
  find_library(
    MOBY_LIBRARY
    NAMES libMoby Moby
    PATHS /usr/local/lib /usr/lib
  )
  if( MOBY_LIBRARY )
    set(MOBY_LIBRARY_DIR "")
    get_filename_component(MOBY_LIBRARY_DIRS ${MOBY_LIBRARY} PATH)
    # Set uncached variables as per standard.
    set(MOBY_FOUND ON)
    set(MOBY_INCLUDE_DIRS ${MOBY_INCLUDE_DIR})
    set(MOBY_LIBRARIES ${MOBY_LIBRARY})
  endif(MOBY_LIBRARY)
else(MOBY_INCLUDE_DIR)
  message(FATAL_ERROR "FindMoby: Could not find AABB.h")
endif(MOBY_INCLUDE_DIR)
	    
if(MOBY_FOUND)
  if(NOT MOBY_FIND_QUIETLY)
    message(STATUS "FindMoby: Found both AABB.h and libMoby.a")
  endif(NOT MOBY_FIND_QUIETLY)
else(MOBY_FOUND)
  if(MOBY_FIND_REQUIRED)
    message(FATAL_ERROR "FindMOby: Could not find AABB.h and/or libMoby.a")
  endif(MOBY_FIND_REQUIRED)
endif(MOBY_FOUND)
