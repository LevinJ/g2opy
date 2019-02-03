LIST(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/EXTERNAL/pybind11/tools)

include(pybind11Tools)

message("PYTHON_INCLUDE_DIRS_sba=" ${PYTHON_INCLUDE_DIRS})
message("PYTHON_LIBRARIES_sba=" ${PYTHON_LIBRARIES})

ADD_LIBRARY(types_sba ${G2O_LIB_TYPE}
  types_sba.h     types_six_dof_expmap.h
  types_sba.cpp   types_six_dof_expmap.cpp
  g2o_types_sba_api.h types_direct_method.h
)

SET_TARGET_PROPERTIES(types_sba PROPERTIES OUTPUT_NAME ${LIB_PREFIX}types_sba)
target_include_directories(types_sba PRIVATE ${PYTHON_INCLUDE_DIRS})
TARGET_LINK_LIBRARIES(types_sba core types_slam3d ${PYTHON_LIBRARIES})

INSTALL(TARGETS types_sba
  RUNTIME DESTINATION ${CMAKE_INSTALL_PREFIX}/bin
  LIBRARY DESTINATION ${CMAKE_INSTALL_PREFIX}/lib
  ARCHIVE DESTINATION ${CMAKE_INSTALL_PREFIX}/lib
)

FILE(GLOB headers "${CMAKE_CURRENT_SOURCE_DIR}/*.h" "${CMAKE_CURRENT_SOURCE_DIR}/*.hpp")

INSTALL(FILES ${headers} DESTINATION ${CMAKE_INSTALL_PREFIX}/include/g2o/types/sba)
