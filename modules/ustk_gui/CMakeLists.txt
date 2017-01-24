# TODO: Check if vtkGUISupportQt found
# TODO: Check if ${QT_LIBRARIES} for Qt4 sould be imported using IMPORTED_LOCATION_
# TODO: Check if VTK_VERSION_BUILD or VTK_VERSION_PATH on vtk-6.x and vtk-5.x
# TODO: find how to suppress include(${VTK_USE_FILE}). For the moment, if not used on OSX, produces a segfault
# TODO: find a way to remove all the warnings due to vtk headers

VP_OPTION(USE_VTK "VTK;COMPONENTS;vtkCommonCore;vtkFiltersSources;vtkIOImage;vtkInteractionImage;vtkGUISupportQt;vtkRenderingImage" "" "Include vtk support" "" ON)

if(USE_VTK)
  if(NOT VTK_VERSION)
    set(VTK_VERSION "${VTK_MAJOR_VERSION}.${VTK_MINOR_VERSION}.${VTK_BUILD_VERSION}")
  endif()
  if(${VTK_VERSION} VERSION_LESS "6.0.0")
    message("vtk-${VTK_VERSION} detected but too old to be used by ustk. You should install at least vtk-6.0.0 to use ustk_gui module.")
    set(USE_VTK OFF CACHE BOOL "Enables vtk usage" FORCE)
  endif()
endif()

#message("VTK_LIBRARIES: ${VTK_LIBRARIES}")
if(USE_VTK)
  if("${VTK_QT_VERSION}" STREQUAL "")
    message("vtk found, but turned off since vtk was not built with Qt")
    set(USE_VTK OFF CACHE BOOL "Turn vtk support off" FORCE)
#  elseif(VTK_QT_VERSION VERSION_GREATER "4")
#    find_package(VTK QUIET COMPONENTS vtkGUISupportQt)
  endif()
endif()

if(USE_VTK)
  if(VTK_VERSION VERSION_LESS "7.1.0")
    set(USTK_HAVE_VTK_VERSION "(${VTK_VERSION_MAJOR}<<16 | ${VTK_VERSION_MINOR}<<8 | ${VTK_VERSION_PATCH})") # for vpConfig.h
  else()
    set(USTK_HAVE_VTK_VERSION "(${VTK_VERSION_MAJOR}<<16 | ${VTK_VERSION_MINOR}<<8 | ${VTK_VERSION_BUILD})") # since 7.1.0 for vpConfig.h
  endif()

  include(${VTK_USE_FILE})

  list(APPEND opt_incs ${VTK_INCLUDE_DIRS})

  # list(APPEND opt_libs ${VTK_LIBRARIES})
  # Using VTK_LIBRARIES works to build visp/ustk library, examples, demos and test thanks to the components,
  # but not tutorials when they are build outside ViSP as they are stand alone CMake projects that use
  # ViSP as a 3rd party.
  # To be clear VTK_LIBRARIES contains VTK 3rd party such as vtkalglib and not /usr/local/Cellar/vtk/6.3.0/lib/libvtkalglib-6.3.1.dylib
  # full path as requested to use ViSP as 3rd party. This is the case for all VTK libraries we depend on.
  # The build of works with VTK_LIBRARIES since in that case thanks to vtkalglib properties, CMake
  # is able to find the real name and location of the libraries.
  # But when ViSP is used as a 3rd party where it should import VTK libraries, it doesn't work with
  # VTK_LIBRARIES.
  # The solution here is to get the full location of VTK_LIBRARIES libraries thanks to the properties and link
  # with these names.
  # An other way could be to include vtkConfig.cmake, but in that case, visp-config and visp.pc
  # will not be able to give the names of VTK libraries when used without CMake.

  set(config_ "NONE" "RELEASE" "DEBUG" "RELEASEWITHDEBINFO" "RELWITHDEBINFO")
  foreach(lib_ ${VTK_LIBRARIES})
    if(${lib_} MATCHES "^(vtk.*)")
      foreach(imp_config_ ${config_})
        get_target_property(lib_property_${imp_config_}_ ${lib_} IMPORTED_LOCATION_${imp_config_})

        # Under Unix, there is no specific suffix for PCL/VTK libraries.         # Under Windows, we add the "optimized", "debug" specific keywords
        if(WIN32 AND EXISTS "${lib_property_${imp_config_}_}" AND "${imp_config_}" MATCHES "RELEASE") # also valid for RELEASEWITHDEBINFO
          list(APPEND opt_libs optimized "${component_property_${imp_config_}_}")
        elseif(WIN32 AND EXISTS "${lib_property_${imp_config_}_}" AND "${imp_config_}" MATCHES "DEBUG")
          list(APPEND opt_libs debug     "${lib_property_${imp_config_}_}")
        elseif(EXISTS "${lib_property_${imp_config_}_}")
          list(APPEND opt_libs "${lib_property_${imp_config_}_}")
        endif()
      endforeach()
    else() # not a vtk library
      list(APPEND opt_libs "${lib_}")
    endif()
  endforeach()

  # Instruct CMake to run moc automatically when needed.
  set(CMAKE_AUTOMOC ON)

  # Find Qt version corresponding to the Qt version used to compile VTK (4 or 5)
  if(VTK_QT_VERSION VERSION_EQUAL "5") # Qt5 not working yet
    find_package(Qt5 COMPONENTS Widgets REQUIRED QUIET)
    qt5_wrap_ui(UI_Srcs ${UIs})

    list(APPEND opt_incs "${Qt5Widgets_INCLUDE_DIRS}")

    # list(APPEND opt_libs "${Qt5Widgets_LIBRARIES}")
    # Using Qt5Widgets_LIBRARIES works to build visp/ustk library, examples, demos and test thanks to the components,
    # but not tutorials when they are build outside ViSP as they are stand alone CMake projects that use
    # ViSP as a 3rd party.

    foreach(lib_ ${Qt5Widgets_LIBRARIES})
      foreach(imp_config_ ${config_})
        get_target_property(lib_property_${imp_config_}_ ${lib_} IMPORTED_LOCATION_${imp_config_})
        # Under Unix, there is no specific suffix for PCL/VTK libraries.         # Under Windows, we add the "optimized", "debug" specific keywords
        if(WIN32 AND EXISTS "${lib_property_${imp_config_}_}" AND "${imp_config_}" MATCHES "RELEASE") # also valid for RELEASEWITHDEBINFO
          list(APPEND opt_libs optimized "${component_property_${imp_config_}_}")
        elseif(WIN32 AND EXISTS "${lib_property_${imp_config_}_}" AND "${imp_config_}" MATCHES "DEBUG")
          list(APPEND opt_libs debug     "${lib_property_${imp_config_}_}")
        elseif(EXISTS "${lib_property_${imp_config_}_}")
          list(APPEND opt_libs "${lib_property_${imp_config_}_}")
        endif()
        get_target_property(lib_deps_${imp_config_}_ ${lib_} IMPORTED_LINK_INTERFACE_LIBRARIES_${imp_config_})

        if(lib_deps_${imp_config_}_)
          foreach(deps_ ${lib_deps_${imp_config_}_})
            get_target_property(deps_location_ ${deps_} IMPORTED_LOCATION_${imp_config_})
            if(EXISTS "${deps_location_}")
              if(WIN32 AND "${imp_config_}" MATCHES "RELEASE")
                list(APPEND opt_libs optimized ${deps_location_})
              elseif(WIN32 AND "${imp_config_}" MATCHES "DEBUG")
                list(APPEND opt_libs debug ${deps_location_})
              else()
                list(APPEND opt_libs ${deps_location_})
              endif()
            endif()
          endforeach()
        endif()

      endforeach()
    endforeach()
    set(USTK_HAVE_VTK_QT5 TRUE)

  elseif(VTK_QT_VERSION VERSION_EQUAL "4")
    find_package(Qt4 REQUIRED QtGui QtCore)
    # Use what VTK built with
    set(QT_QMAKE_EXECUTABLE ${VTK_QT_QMAKE_EXECUTABLE} CACHE FILEPATH "")
    set(QT_MOC_EXECUTABLE ${VTK_QT_MOC_EXECUTABLE} CACHE FILEPATH "")
    set(QT_UIC_EXECUTABLE ${VTK_QT_UIC_EXECUTABLE} CACHE FILEPATH "")

    list(APPEND opt_incs "${QT_QTGUI_INCLUDE_DIR}")
    #list(APPEND opt_incs "${QT_INCLUDE_DIR}/QtCore") # seems not useful
    list(APPEND opt_libs ${QT_QTGUI_LIBRARIES})
    list(APPEND opt_libs ${QT_QTCORE_LIBRARIES})

    set(USTK_HAVE_VTK_QT4 TRUE)

  endif()

  # Create ustk_gui module, only if vtk found
  vp_add_module(ustk_gui visp_ustk_io)
  vp_glob_module_sources()
  vp_module_include_directories(${opt_incs})
  vp_create_module(${opt_libs})

  set(USTK_HAVE_VTK_QT TRUE)

  vp_add_config_file("cmake/templates/usGuiConfig.h.in")
endif()
