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
FILE(COPY ./vis2d_no_bg.py DESTINATION ${CMAKE_CURRENT_BINARY_DIR})
FILE(COPY ./vis2d_with_bg.py DESTINATION ${CMAKE_CURRENT_BINARY_DIR})
FILE(COPY ./vis3d_all_in_one.py DESTINATION ${CMAKE_CURRENT_BINARY_DIR})
FILE(COPY ./vis3d_seperate.py DESTINATION ${CMAKE_CURRENT_BINARY_DIR})
FILE(COPY ./vis3d_all_in_one_bg.py DESTINATION ${CMAKE_CURRENT_BINARY_DIR})
FILE(COPY ./vis3d_seperate_bg.py DESTINATION ${CMAKE_CURRENT_BINARY_DIR})
FILE(COPY ./vis2d_iris.py DESTINATION ${CMAKE_CURRENT_BINARY_DIR})
FILE(COPY ./vis3d_iris.py DESTINATION ${CMAKE_CURRENT_BINARY_DIR})
FILE(COPY ./vis2d_orca.py DESTINATION ${CMAKE_CURRENT_BINARY_DIR})
FILE(COPY ./vis3d_orca.py DESTINATION ${CMAKE_CURRENT_BINARY_DIR})
