cmake_minimum_required(VERSION 3.8)
project(agv_map_3d)
find_package(ament_cmake REQUIRED)
find_package(cartographer REQUIRED)
add_executable(pbstream_to_ply src/pbstream_to_ply.cpp)
target_compile_features(pbstream_to_ply PRIVATE cxx_std_17)
target_link_libraries(pbstream_to_ply PRIVATE cartographer)
install(TARGETS pbstream_to_ply DESTINATION lib/${PROJECT_NAME})
install(PROGRAMS scripts/map_3d_server.py DESTINATION lib/${PROJECT_NAME})
if(BUILD_TESTING)
  enable_testing()
  find_package(Python3 REQUIRED COMPONENTS Interpreter)
  add_executable(map3d_test_fixture tests/make_fixture.cpp)
  target_compile_features(map3d_test_fixture PRIVATE cxx_std_17)
  target_link_libraries(map3d_test_fixture PRIVATE cartographer)
  add_test(NAME map3d_export_coordinates
    COMMAND ${Python3_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/tests/test_export.py
      $<TARGET_FILE:map3d_test_fixture> $<TARGET_FILE:pbstream_to_ply>)
  add_test(NAME map3d_asset_api
    COMMAND ${Python3_EXECUTABLE} -m unittest discover
      -s ${CMAKE_CURRENT_SOURCE_DIR}/tests -p test_asset_server.py -v)
endif()
ament_package()
