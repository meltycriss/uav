SET(PROTO_PATH ${CMAKE_CURRENT_SOURCE_DIR}/../src/proto)
FILE(GLOB proto_files ${PROTO_PATH}/*.proto)
FOREACH(proto_file ${proto_files})
  STRING(REGEX REPLACE "[^/]proto" "" proto_file_name ${proto_file})
  LIST(APPEND PROTO_SRCS "${proto_file_name}_pb2.py")
  ADD_CUSTOM_COMMAND(
    OUTPUT "${proto_file_name}_pb2.py"
    COMMAND protoc --proto_path=${PROTO_PATH}
    --python_out=${CMAKE_CURRENT_BINARY_DIR} ${proto_file}
    DEPENDS ${proto_file}
    )
ENDFOREACH(proto_file)

# dummy target to force add_custom_command above run
add_custom_target(dummy_target ALL
  DEPENDS
  ${PROTO_SRCS}
  )

FILE(COPY irispy DESTINATION ${CMAKE_CURRENT_BINARY_DIR})
FILE(COPY examples DESTINATION ${CMAKE_CURRENT_BINARY_DIR})
FILE(COPY vis.py DESTINATION ${CMAKE_CURRENT_BINARY_DIR})
FILE(COPY disp.py DESTINATION ${CMAKE_CURRENT_BINARY_DIR})
