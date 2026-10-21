function(exec_dir dir prefix output_dir as_test strip_output)
  set(libs ${ARGN})

  file(GLOB_RECURSE source RELATIVE ${dir} ${dir}/*.cpp)
  foreach(src_file ${source})
    string(REPLACE ".cpp" "" exec_base ${src_file})
    string(REPLACE "/" "." exec_name ${exec_base})
    set(exec_name ${prefix}${exec_name})
    add_executable(${exec_name} ${dir}/${src_file})
    target_link_libraries(${exec_name} PRIVATE ${libs})
    set_target_properties(${exec_name} PROPERTIES
      RUNTIME_OUTPUT_DIRECTORY ${output_dir}
    )
    if(as_test)
      add_test(NAME ${exec_name} COMMAND $<TARGET_FILE:${exec_name}>)
      set_tests_properties(${exec_name} PROPERTIES TIMEOUT 30)
    endif()
    if (CMAKE_SYSTEM_NAME STREQUAL "Linux" AND ${strip_output})
      add_custom_command(TARGET ${exec_name}
        COMMAND ${CMAKE_STRIP} $<TARGET_FILE:${exec_name}>
        COMMENT "Shrink executable size by stripping debug symbols"
      )
    endif()
  endforeach()
endfunction()