cmake_minimum_required( VERSION 2.8 )

project( icra2015_multiblock_experiment )

set( LIBS 
  ${GAZEBO_LIBRARIES} 
) 

add_library( multiblock-controller SHARED multiblock_controller.cpp)
target_link_libraries( multiblock-controller ${GAZEBO_LIBRARIES} )

# Copy data files
file( COPY ${CMAKE_CURRENT_SOURCE_DIR}/models DESTINATION ${CMAKE_CURRENT_BINARY_DIR} )
file( COPY ${CMAKE_CURRENT_SOURCE_DIR}/one-block.world DESTINATION ${CMAKE_CURRENT_BINARY_DIR} )
file( COPY ${CMAKE_CURRENT_SOURCE_DIR}/two-block.world DESTINATION ${CMAKE_CURRENT_BINARY_DIR} )
file( COPY ${CMAKE_CURRENT_SOURCE_DIR}/ten-block.world DESTINATION ${CMAKE_CURRENT_BINARY_DIR} )
file( COPY ${CMAKE_CURRENT_SOURCE_DIR}/eleven-block.world DESTINATION ${CMAKE_CURRENT_BINARY_DIR} )

# Copy setup file and substitute correct path into file
SET( SEARCH_REGEX "%BUILD_DIR%" )
file( READ ${CMAKE_CURRENT_SOURCE_DIR}/setup.sh SETUP_FILE_TEXT )
STRING( REGEX REPLACE "${SEARCH_REGEX}" "${CMAKE_CURRENT_BINARY_DIR}" MODIFIED_FILE_TEXT "${SETUP_FILE_TEXT}" )
FILE( WRITE ${CMAKE_CURRENT_BINARY_DIR}/setup.sh "${MODIFIED_FILE_TEXT}" )

