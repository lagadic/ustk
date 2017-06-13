# Find the native fftw includes and libraries
#
# FFTW_FOUND - True if fftw found.
# FFTW_INCLUDE_DIRS - where to find fftw3.h
# FFTW_LIBRARIES - List of libraries when using FFTW.

if(FFTW_INCLUDE_DIRS)
  # Already in cache, be silent
  set(FFTW_FIND_QUIETLY TRUE)
endif(FFTW_INCLUDE_DIRS)

if(WIN32)
  find_path(FFTW_INCLUDE_DIRS fftw3.h
    $ENV{FFTW_HOME}
  )
  find_library(FFTW_LIBRARIES NAMES libfftw3-3
    $ENV{FFTW_HOME}
  )
else()
  if($ENV{FFTW_HOME})
    find_path(FFTW_INCLUDE_DIRS fftw3.h
      "$ENV{FFTW_HOME}/include"
    )

    find_library(FFTW_LIBRARIES NAMES "fftw3"
      PATHS "$ENV{FFTW_HOME}/lib"
    )
  else()
    find_path(FFTW_INCLUDE_DIRS fftw3.h)
    find_library(FFTW_LIBRARIES NAMES fftw3)
  endif()
endif()

# handle the QUIETLY and REQUIRED arguments and set FFTW_FOUND to TRUE if
# all listed variables are TRUE
include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(FFTW DEFAULT_MSG FFTW_LIBRARIES FFTW_INCLUDE_DIRS)
mark_as_advanced(FFTW_LIBRARIES FFTW_INCLUDE_DIRS)
