vp_add_module(ustk_gui visp_ustk_core)

if(NOT VTK_FOUND)
  VP_OPTION(USE_VTK       VTK        ""    "Include vtk support"          "" ON)
endif()

find_package(VTK COMPONENTS
  vtkCommonCore
  vtkFiltersSources
  vtkGUISupportQt
  vtkIOImage
  vtkInteractionImage
)

if("${VTK_QT_VERSION}" STREQUAL "")
  message(FATAL_ERROR "VTK was not built with Qt")
endif()

if(VTK_FOUND AND USE_VTK)
  include(${VTK_USE_FILE})
else()
  message(WARNING "VTK not found for ustk gui module")
endif()

# Instruct CMake to run moc automatically when needed.
set(CMAKE_AUTOMOC ON)

#Find Qt version corresponding to the Qt version used to compile VTK (4 or 5)
if(VTK_QT_VERSION VERSION_GREATER "4") #Qt5 not tested yet
  find_package(Qt5 COMPONENTS Widgets REQUIRED QUIET)
  qt5_wrap_ui(UI_Srcs ${UIs})

  #ORIGNIAL
  #add_executable(QtVTKRenderWindows ${Srcs} ${Hdrs} ${UI_Srcs} ${MOC_Hdrs})
  #qt5_use_modules(QtVTKRenderWindows Core Gui Widgets)
  #target_link_libraries(QtVTKRenderWindows ${VTK_LIBRARIES})

  #VISP version (not tested yet)
  vp_glob_module_sources()
  vp_module_include_directories(${QT_INCLUDE_DIRS} ${VTK_INCLUDE_DIRS})
  vp_create_module(${QT_LIBRARIES} ${VTK_LIBRARIES})
  
else()
  find_package(Qt4 REQUIRED)
  include(${QT_USE_FILE})
  # Use what VTK built with
  #set(QT_QMAKE_EXECUTABLE ${VTK_QT_QMAKE_EXECUTABLE} CACHE FILEPATH "")
  #set(QT_MOC_EXECUTABLE ${VTK_QT_MOC_EXECUTABLE} CACHE FILEPATH "")
  #(QT_UIC_EXECUTABLE ${VTK_QT_UIC_EXECUTABLE} CACHE FILEPATH "")
  #(UI_Srcs ${UIs})
 
  # ORIGNIAL
  # add_executable(QtVTKRenderWindows ${Srcs} ${Hdrs} ${UI_Srcs} ${MOC_Hdrs})
  #target_link_libraries(QtVTKRenderWindows ${QT_LIBRARIES} ${VTK_LIBRARIES})

  #VISP version
  vp_glob_module_sources()
  vp_module_include_directories(${QT_INCLUDE_DIR} ${VTK_INCLUDE_DIRS})
  vp_create_module(${QT_LIBRARIES} ${VTK_LIBRARIES})
endif()
