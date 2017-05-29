# Find the native fftw includes and libraries
#
# FFTW_FOUND - True if fftw found.
# FFTW_INCLUDE_DIRS - where to find fftw3.h
# FFTW_LIBRARIES - List of libraries when using FFTW.

if(FFTW_INCLUDE_DIRS)
  # Already in cache, be silent
  set(FFTW_FIND_QUIETLY TRUE)
endif(FFTW_INCLUDE_DIRS)

find_path(FFTW_INCLUDE_DIRS fftw3.h)
find_library(FFTW_LIBRARIES NAMES fftw3 fftw3_omp)

# handle the QUIETLY and REQUIRED arguments and set FFTW_FOUND to TRUE if
# all listed variables are TRUE
include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(FFTW DEFAULT_MSG FFTW_LIBRARIES FFTW_INCLUDE_DIRS)
mark_as_advanced(FFTW_LIBRARIES FFTW_INCLUDE_DIRS)
