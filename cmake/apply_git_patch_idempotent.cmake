if(NOT DEFINED SOURCE_DIR OR NOT DEFINED PATCH_FILE)
  message(FATAL_ERROR "SOURCE_DIR and PATCH_FILE are required")
endif()

if(NOT EXISTS "${PATCH_FILE}")
  message(FATAL_ERROR "Patch file not found: ${PATCH_FILE}")
endif()

if(NOT EXISTS "${SOURCE_DIR}")
  message(FATAL_ERROR "Source dir not found: ${SOURCE_DIR}")
endif()

if(NOT DEFINED PATCH_LABEL)
  set(PATCH_LABEL "${PATCH_FILE}")
endif()

execute_process(
  COMMAND git -C "${SOURCE_DIR}" apply --check "${PATCH_FILE}"
  RESULT_VARIABLE _check_result
  OUTPUT_QUIET
  ERROR_QUIET
)

if(_check_result EQUAL 0)
  message(STATUS "Applying patch: ${PATCH_LABEL}")
  execute_process(
    COMMAND git -C "${SOURCE_DIR}" apply "${PATCH_FILE}"
    RESULT_VARIABLE _apply_result
    OUTPUT_VARIABLE _apply_out
    ERROR_VARIABLE _apply_err
  )
  if(NOT _apply_result EQUAL 0)
    message(FATAL_ERROR "Failed to apply patch ${PATCH_LABEL}:\n${_apply_out}\n${_apply_err}")
  endif()
  return()
endif()

execute_process(
  COMMAND git -C "${SOURCE_DIR}" apply --reverse --check "${PATCH_FILE}"
  RESULT_VARIABLE _reverse_check_result
  OUTPUT_QUIET
  ERROR_QUIET
)

if(_reverse_check_result EQUAL 0)
  message(STATUS "Patch already applied: ${PATCH_LABEL}")
  return()
endif()

message(FATAL_ERROR "Patch does not apply cleanly and is not already applied: ${PATCH_LABEL}")
