set(generated "${CMAKE_CURRENT_BINARY_DIR}/s265.rc"
              "${CMAKE_CURRENT_BINARY_DIR}/s265.pc"
              "${CMAKE_CURRENT_BINARY_DIR}/s265.def"
              "${CMAKE_CURRENT_BINARY_DIR}/s265_config.h")

foreach(file ${generated})
  if(EXISTS ${file})
     file(REMOVE ${file})
  endif()
endforeach(file)
