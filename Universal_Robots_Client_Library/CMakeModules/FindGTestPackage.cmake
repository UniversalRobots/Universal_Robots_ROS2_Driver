# SetupGTestTarget
# -------------------------
#
# Finds the gtest package.  Setups a gtest target, so that consumers can use gtest.
#
# setup_gtest_target(<target>)
#
# ::
#
# <target> - Defines the variable the target should be stored in.
#
# This function tries to find the googletest framework.
# There are 3 stages:
#   1. Try to find a proper installed googletest package. (e.g. through "make install")
#   2. Try to find a source version from a debian package. (Usually under /usr/src)
#   3. If enabled then try to download the source from the repository directly.
#
# Depending on the found version the variable is set to the correct target.
#
# ::
#

option(DOWNLOAD_GTEST "Download googletest framework if not found on system." OFF)

function(find_gtest_package target)

  # 1) Try to find a proper installed gtest suite

  find_package(GTest)

  if(GTEST_FOUND)
    if(NOT TARGET GTest::GTest)
      if(NOT TARGET GTest)
	if(DEFINED GTEST_LIBRARY)
          # -lpthread neccessary for Ubuntu 14.04
          set(${target} ${GTEST_LIBRARY} -lpthread PARENT_SCOPE)
	else()

	  message(FATAL_ERROR "The found gtest package does neither contain GTest nor GTest::GTest. Please make sure GTest are installed correctly.")

	endif()
      else()
	# -lpthread neccessary for Ubuntu 14.04
	set(${target} GTest -lpthread PARENT_SCOPE)

        if(NOT CMAKE_REQUIRED_QUIET)
          message(STATUS "GTest target found as: GTest")
        endif()
      endif()   
    else()
      set(${target} "GTest::GTest" PARENT_SCOPE)

      if(NOT CMAKE_REQUIRED_QUIET)
        message(STATUS "GTest target found as: GTest::GTest")
      endif()
    endif()

  else()

    # 2) Try to find the source version (on Ubuntu)

    find_path(GTEST_SRC_DIR src/gtest.cc
      HINTS
        ENV GTEST_DIR
      PATHS
        /usr/src/googletest/googletest
        /usr/src/gtest
      DOC "Path to the source version of gtest, if available" )

    if(GTEST_SRC_DIR)

      if(NOT TARGET gtest)
        add_subdirectory(${GTEST_SRC_DIR} "${CMAKE_CURRENT_BINARY_DIR}/gtest" EXCLUDE_FROM_ALL)
        # provide the target for caller 
        set(${target} "gtest" PARENT_SCOPE)

        if(NOT CMAKE_REQUIRED_QUIET)
          message(STATUS "GTest target will be build from source: " ${GTEST_SRC_DIR})
        endif()

      endif()
        
    else()

      # 3) Try to Download and unpack googletest if enabled
      # see https://crascit.com/2015/07/25/cmake-gtest/ for an explanation of the complete process

      if(DOWNLOAD_GTEST)

	# try to guess where the .in for the download is
        find_path(GTEST_DL_TMPL_DIR FindGTest.CMakeLists.txt.in
          PATHS
            ${CMAKE_SOURCE_DIR}/CMakeModules/
            ${CMAKE_CURRENT_SOURCE_DIR}/CMakeModules/
            ${CMAKE_CURRENT_SOURCE_DIR}/../CMakeModules/)

        configure_file("${GTEST_DL_TMPL_DIR}/FindGTest.CMakeLists.txt.in" "${CMAKE_BINARY_DIR}/gtest-download/CMakeLists.txt")
        execute_process(COMMAND "${CMAKE_COMMAND}" -G "${CMAKE_GENERATOR}" .
          WORKING_DIRECTORY "${CMAKE_BINARY_DIR}/gtest-download")
        # this is not the build itself, it just downloads the gtest repository 
        execute_process(COMMAND "${CMAKE_COMMAND}" --build .
          WORKING_DIRECTORY "${CMAKE_BINARY_DIR}/gtest-download"
          RESULT_VARIABLE GTEST_BUILD_RESULT)

        if(${GTEST_BUILD_RESULT} EQUAL 0)

          set(gtest_force_shared_crt ON CACHE BOOL "" FORCE)

          add_subdirectory("${CMAKE_BINARY_DIR}/gtest-src" "${CMAKE_BINARY_DIR}/gtest-build" EXCLUDE_FROM_ALL)
          # provide the target for caller 
          set(${target} "gtest" PARENT_SCOPE)

        else()

          message(FATAL_ERROR "Could not find gtest package/source nor did downloading from repository succeed. Make sure you have gtest installed.")

        endif()

      else()

	message(FATAL_ERROR "Could not find gtest package nor gtest source. USE DOWNLOAD_GTEST option to build from source.")

      endif()

    endif()

  endif()
endfunction(find_gtest_package)
