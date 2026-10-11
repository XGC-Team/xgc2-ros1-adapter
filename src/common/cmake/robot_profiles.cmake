# Validates the repository-owned profiles of one robot type against the
# protobuf registry and produces
#   - the contract tables compiled into its type server (HEADER), and
#   - the profile catalog installed for Core.
#
#   xgc2_add_robot_profiles(
#     TARGET <custom target>        built with ALL
#     DEFINITION_ID <provider id>   e.g. xgc2-scout-mini-ros1-adapter
#     NAMESPACE <c++ namespace>     of InstalledContract() in the header
#     HEADER <path>                 generated header
#     PROFILES <yaml>...            first one is the primary (physical) profile
#   )
get_filename_component(XGC2_ROBOT_TOOLS_DIR
  "${CMAKE_CURRENT_LIST_DIR}/../../../tools" ABSOLUTE)
get_filename_component(XGC2_ROBOT_PROFILE_ROOT
  "${CMAKE_CURRENT_LIST_DIR}/../../../profiles" ABSOLUTE)

function(xgc2_add_robot_profiles)
  cmake_parse_arguments(PROFILE "" "TARGET;DEFINITION_ID;NAMESPACE;HEADER"
                        "PROFILES" ${ARGN})
  find_package(PythonInterp 3 REQUIRED)
  if(NOT EXISTS "${XGC2_PROTOBUF_REGISTRY_JSON}")
    message(FATAL_ERROR "xgc2-protobuf-dev did not expose the XGC2 message registry")
  endif()
  set(schema "${XGC2_ROBOT_PROFILE_ROOT}/schema/robot-adapter-profile-v4.schema.json")
  set(profile_args)
  foreach(file IN LISTS PROFILE_PROFILES)
    if(NOT EXISTS "${file}")
      message(FATAL_ERROR "robot profile ${file} is missing")
    endif()
    list(APPEND profile_args --profile-file "${file}")
  endforeach()
  set(catalog
    "${CMAKE_CURRENT_BINARY_DIR}/robot-adapter-profiles/${PROFILE_DEFINITION_ID}.json")
  get_filename_component(header_directory "${PROFILE_HEADER}" DIRECTORY)
  add_custom_command(
    OUTPUT "${PROFILE_HEADER}" "${catalog}"
    COMMAND "${CMAKE_COMMAND}" -E make_directory "${header_directory}"
    COMMAND "${PYTHON_EXECUTABLE}" "${XGC2_ROBOT_TOOLS_DIR}/generate_contract_metadata.py"
            --registry "${XGC2_PROTOBUF_REGISTRY_JSON}" ${profile_args}
            --profile-schema "${schema}"
            --definition-id "${PROFILE_DEFINITION_ID}"
            --cpp-namespace "${PROFILE_NAMESPACE}"
            --output "${PROFILE_HEADER}"
    COMMAND "${PYTHON_EXECUTABLE}" "${XGC2_ROBOT_TOOLS_DIR}/generate_profile_catalog.py"
            --registry "${XGC2_PROTOBUF_REGISTRY_JSON}" ${profile_args}
            --profile-schema "${schema}"
            --definition-id "${PROFILE_DEFINITION_ID}"
            --output "${catalog}"
    DEPENDS
      "${XGC2_ROBOT_TOOLS_DIR}/generate_contract_metadata.py"
      "${XGC2_ROBOT_TOOLS_DIR}/generate_profile_catalog.py"
      "${XGC2_PROTOBUF_REGISTRY_JSON}" "${schema}" ${PROFILE_PROFILES}
    COMMENT "Validating the ${PROFILE_DEFINITION_ID} robot profiles"
    VERBATIM
  )
  add_custom_target(${PROFILE_TARGET} ALL DEPENDS "${PROFILE_HEADER}" "${catalog}")
  # The profiles are installed with the package that owns them, next to the
  # catalog Core reads. The caller may run before catkin_package(), so the
  # package share directory is spelled out.
  install(FILES "${catalog}" DESTINATION /usr/share/xgc2/robot-adapter-profiles)
  install(FILES ${PROFILE_PROFILES}
    DESTINATION share/${PROJECT_NAME}/profiles/ros1)
  install(FILES "${schema}"
    DESTINATION share/${PROJECT_NAME}/profiles/schema)
endfunction()
