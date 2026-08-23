get_filename_component(XGC2_ROS1_ROBOT_COMMON_ROOT
  "${CMAKE_CURRENT_LIST_DIR}/.." ABSOLUTE)

function(xgc2_add_ros1_robot_common target_name)
  if(NOT XGC2_PROTOBUF_PROTO_ROOT OR
     NOT EXISTS "${XGC2_PROTOBUF_PROTO_ROOT}/xgc/robot/v1/message.proto")
    message(FATAL_ERROR "xgc2-protobuf-dev does not provide robot-domain schemas")
  endif()

  set(protocol_files
    xgc/robot/v1/message.proto
    xgc/semantic/aerial/v1/control.proto
    xgc/semantic/aerial/v1/diagnostic.proto
    xgc/semantic/aerial/v1/flight.proto
    xgc/semantic/aerial/v1/setpoint.proto
    xgc/semantic/common/v1/control.proto
    xgc/semantic/common/v1/telemetry.proto
    xgc/semantic/common/v1/types.proto
    xgc/semantic/ground/v1/chassis.proto
    xgc/semantic/ground/v1/control.proto
    xgc/semantic/ground/v1/locomotion.proto
  )
  set(generated_dir "${CMAKE_CURRENT_BINARY_DIR}/${target_name}_generated")
  set(protocol_inputs)
  set(protocol_sources)
  set(protocol_headers)
  foreach(relative_file IN LISTS protocol_files)
    list(APPEND protocol_inputs "${XGC2_PROTOBUF_PROTO_ROOT}/${relative_file}")
    string(REGEX REPLACE "\\.proto$" ".pb.cc" generated_source
                         "${relative_file}")
    string(REGEX REPLACE "\\.proto$" ".pb.h" generated_header
                         "${relative_file}")
    list(APPEND protocol_sources "${generated_dir}/${generated_source}")
    list(APPEND protocol_headers "${generated_dir}/${generated_header}")
  endforeach()

  add_custom_command(
    OUTPUT ${protocol_sources} ${protocol_headers}
    COMMAND "${CMAKE_COMMAND}" -E make_directory "${generated_dir}"
    COMMAND "${Protobuf_PROTOC_EXECUTABLE}"
            "--proto_path=${XGC2_PROTOBUF_PROTO_ROOT}"
            "--cpp_out=${generated_dir}"
            ${protocol_inputs}
    DEPENDS ${protocol_inputs}
    COMMENT "Generating robot-domain protobuf C++ bindings"
    VERBATIM
  )

  add_library(${target_name} STATIC
    ${protocol_sources}
    "${XGC2_ROS1_ROBOT_COMMON_ROOT}/src/ground_health.cpp"
    "${XGC2_ROS1_ROBOT_COMMON_ROOT}/src/robot_domain.cpp"
    "${XGC2_ROS1_ROBOT_COMMON_ROOT}/src/runtime_support.cpp"
  )
  target_compile_features(${target_name} PUBLIC cxx_std_14)
  target_compile_options(${target_name} PRIVATE -Wall -Wextra -Wpedantic)
  target_include_directories(${target_name}
    PUBLIC
      "${XGC2_ROS1_ROBOT_COMMON_ROOT}/include"
      "${generated_dir}"
  )
  target_link_libraries(${target_name}
    PUBLIC
      xgc2::adapter_runtime_client
      protobuf::libprotobuf
  )
endfunction()

function(xgc2_add_robot_runtime_manifests target_name definition_id version
         label description profile_file profile_schema)
  set(generator
    "${XGC2_ROS1_ROBOT_COMMON_ROOT}/../tools/generate_runtime_manifests.py")
  set(verifier
    "${XGC2_ROS1_ROBOT_COMMON_ROOT}/../tools/verify_runtime_manifests.py")
  set(output_root "${CMAKE_CURRENT_BINARY_DIR}/runtime-manifests")
  set(adapter_manifest
    "${output_root}/adapter-definitions/${definition_id}.json")
  set(process_manifest
    "${output_root}/process-definitions/${definition_id}.json")
  set(profile_catalog
    "${output_root}/robot-adapter-profiles/${definition_id}.json")
  add_custom_command(
    OUTPUT "${adapter_manifest}" "${process_manifest}" "${profile_catalog}"
    COMMAND "${PYTHON_EXECUTABLE}" "${generator}"
            --executable "$<TARGET_FILE:${target_name}>"
            --ros-package "${PROJECT_NAME}"
            --ros-executable "${target_name}"
            --registry "${XGC2_PROTOBUF_REGISTRY_JSON}"
            --profile-file "${profile_file}"
            --profile-schema "${profile_schema}"
            --definition-id "${definition_id}"
            --version "${version}"
            --label "${label}"
            --description "${description}"
            --adapter-output "${adapter_manifest}"
            --process-output "${process_manifest}"
            --profile-output "${profile_catalog}"
    COMMAND "${PYTHON_EXECUTABLE}" "${verifier}"
            --executable "$<TARGET_FILE:${target_name}>"
            --ros-package "${PROJECT_NAME}"
            --ros-executable "${target_name}"
            --definition-id "${definition_id}"
            --registry "${XGC2_PROTOBUF_REGISTRY_JSON}"
            --profile-file "${profile_file}"
            --profile-schema "${profile_schema}"
            --adapter-manifest "${adapter_manifest}"
            --process-manifest "${process_manifest}"
            --profile-catalog "${profile_catalog}"
    DEPENDS
      ${target_name}
      "${generator}"
      "${verifier}"
      "${XGC2_ROS1_ROBOT_COMMON_ROOT}/../tools/generate_contract_metadata.py"
      "${XGC2_PROTOBUF_REGISTRY_JSON}"
      "${profile_file}"
      "${profile_schema}"
    COMMENT "Generating exact ${definition_id} Adapter Runtime manifests"
    VERBATIM
  )
  add_custom_target(${target_name}_runtime_manifests ALL
    DEPENDS "${adapter_manifest}" "${process_manifest}" "${profile_catalog}")

  set(XGC2_INSTALLED_EXECUTABLE
    "${CMAKE_INSTALL_PREFIX}/${CATKIN_PACKAGE_BIN_DESTINATION}/${target_name}")
  set(XGC2_ROS_PACKAGE "${PROJECT_NAME}")
  set(XGC2_ROS_EXECUTABLE "${target_name}")
  set(XGC2_ADAPTER_DEFINITION_ID "${definition_id}")
  set(XGC2_ADAPTER_VERSION "${version}")
  set(XGC2_ADAPTER_LABEL "${label}")
  set(XGC2_ADAPTER_DESCRIPTION "${description}")
  set(XGC2_PROFILE_FILE "${profile_file}")
  set(XGC2_PROFILE_SCHEMA "${profile_schema}")
  set(XGC2_RUNTIME_MANIFEST_GENERATOR "${generator}")
  set(XGC2_RUNTIME_MANIFEST_VERIFIER "${verifier}")
  set(install_script
    "${CMAKE_CURRENT_BINARY_DIR}/install_${definition_id}_runtime_manifests.cmake")
  configure_file(
    "${XGC2_ROS1_ROBOT_COMMON_ROOT}/cmake/install_runtime_manifests.cmake.in"
    "${install_script}" @ONLY)
  install(SCRIPT "${install_script}")
endfunction()
