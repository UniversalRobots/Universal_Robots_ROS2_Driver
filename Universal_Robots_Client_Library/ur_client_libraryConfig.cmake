include(CMakeFindDependencyMacro)

if(NOT TARGET ur_client_library::urcl)
  include("${CMAKE_CURRENT_LIST_DIR}/urclTargets.cmake")
endif()

# This is for catkin compatibility. Better use target_link_libraries(<my_target> ur_client_library::ur_client_library)
set(ur_client_library_LIBRARIES ur_client_library::urcl)
get_target_property(ur_client_library_INCLUDE_DIRS ur_client_library::urcl INTERFACE_INCLUDE_DIRECTORIES)

