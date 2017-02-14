project(visp-tutorial-ustk)

cmake_minimum_required(VERSION 2.6)

find_package(VISP REQUIRED)

if(MSVC)
  if(NOT VISP_SHARED)
    foreach(flag_var
            CMAKE_C_FLAGS CMAKE_C_FLAGS_DEBUG CMAKE_C_FLAGS_RELEASE
            CMAKE_C_FLAGS_MINSIZEREL CMAKE_C_FLAGS_RELWITHDEBINFO
            CMAKE_CXX_FLAGS CMAKE_CXX_FLAGS_DEBUG CMAKE_CXX_FLAGS_RELEASE
            CMAKE_CXX_FLAGS_MINSIZEREL CMAKE_CXX_FLAGS_RELWITHDEBINFO)
      if(${flag_var} MATCHES "/MD")
        string(REGEX REPLACE "/MD" "/MT" ${flag_var} "${${flag_var}}")
      endif()
      if(${flag_var} MATCHES "/MDd")
        string(REGEX REPLACE "/MDd" "/MTd" ${flag_var} "${${flag_var}}")
      endif()
    endforeach(flag_var)

    set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} /NODEFAULTLIB:atlthunk.lib /NODEFAULTLIB:msvcrt.lib /NODEFAULTLIB:msvcrtd.lib")
    set(CMAKE_EXE_LINKER_FLAGS_DEBUG "${CMAKE_EXE_LINKER_FLAGS_DEBUG} /NODEFAULTLIB:libcmt.lib")
    set(CMAKE_EXE_LINKER_FLAGS_RELEASE "${CMAKE_EXE_LINKER_FLAGS_RELEASE} /NODEFAULTLIB:libcmtd.lib")
  endif()
endif()

visp_add_subdirectory(sonosite REQUIRED_DEPS visp_ustk_core visp_ustk_confidence_map)
visp_add_subdirectory(scanConversion REQUIRED_DEPS visp_ustk_core visp_ustk_io)
visp_add_subdirectory(ultrasonix REQUIRED_DEPS visp_ustk_core visp_ustk_grabber)
visp_add_subdirectory(needleDetection REQUIRED_DEPS visp_ustk_core visp_ustk_io visp_ustk_needle_detection)
visp_add_subdirectory(gui REQUIRED_DEPS visp_ustk_core visp_ustk_io visp_ustk_gui)
