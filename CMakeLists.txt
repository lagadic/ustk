project(USTK CXX)

cmake_minimum_required(VERSION 2.8.3)

#-----------------------------------------------------------------------------
# VISP version number.  An even minor number corresponds to releases.
set(USTK_VERSION_MAJOR "1")
set(USTK_VERSION_MINOR "0")
set(USTK_VERSION_PATCH "0")
set(USTK_VERSION "${USTK_VERSION_MAJOR}.${USTK_VERSION_MINOR}.${USTK_VERSION_PATCH}")

set(USTK_DOC_DIR "${USTK_BINARY_DIR}/doc")


#----------------------------------------------------------------------
# Try to find doxygen for documentation generation
# Use "make ustk_doc" target to generate the documentation
#----------------------------------------------------------------------
find_package(Doxygen)
if(DOXYGEN_FOUND)
  ## we need latex for doxygen because of the formulas
  find_package(LATEX)
  if(NOT LATEX_COMPILER)
    message(STATUS "latex command LATEX_COMPILER not found but usually required. You will probably get warnings and user interaction on doxy run.")
  endif()
  if(NOT MAKEINDEX_COMPILER)
    message(STATUS "makeindex command MAKEINDEX_COMPILER not found but usually required.")
  endif()
  if(NOT DVIPS_CONVERTER)
    message(STATUS "dvips command DVIPS_CONVERTER not found but usually required.")
  endif()

  # set vars used in doxygen config file
  # - DOXYGEN_STRIP_FROM_INC_PATH corresponding to STRIP_FROM_INC_PATH in the doxy file
  set(DOXYGEN_STRIP_FROM_INC_PATH "")

  file(GLOB_RECURSE __path "${USTK_SOURCE_DIR}/modules" "${USTK_SOURCE_DIR}/modules/*/CMakeLists.txt")
  foreach(p ${__path})
    string(REPLACE "CMakeLists.txt" "include" __inc_path ${p})
    set(DOXYGEN_STRIP_FROM_INC_PATH "${DOXYGEN_STRIP_FROM_INC_PATH} \\ \n\t\t\t \"${__inc_path}\"")
  endforeach()

  configure_file(${USTK_SOURCE_DIR}/doc/config-doxygen.in
    ${USTK_DOC_DIR}/config-doxygen
    @ONLY )

  # set vars used in mainpage.doc.in
  # - VISP_MAINPAGE_EXTENSION
  set(VISP_MAINPAGE_EXTENSION "")
  set(mainpage_ext_file_ "${USTK_SOURCE_DIR}/doc/mainpage_extension.doc")
  if(EXISTS ${mainpage_ext_file_})
    file(READ ${mainpage_ext_file_} mainpage_ext_content_)
    set(VISP_MAINPAGE_EXTENSION "${VISP_MAINPAGE_EXTENSION}\n${mainpage_ext_content_}")
  endif()

  configure_file(${USTK_SOURCE_DIR}/doc/mainpage.doc.in
    ${USTK_DOC_DIR}/mainpage.doc
    @ONLY )
endif()

# ----------------------------------------------------------------------------
#   Doxygen documentation target, for "make visp_doc" and "make html-doc" (to keep compat with previous versions)
# ----------------------------------------------------------------------------
if(DOXYGEN_FOUND)
  add_custom_target(ustk_doc ${DOXYGEN_EXECUTABLE} ${USTK_DOC_DIR}/config-doxygen)
  if(ENABLE_SOLUTION_FOLDERS)
    set_target_properties(visp_doc PROPERTIES FOLDER "extra")
  endif()
endif()
