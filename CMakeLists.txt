cmake_minimum_required(VERSION 3.8)
project(publi_deskewing_uncertainty)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(libpointmatcher REQUIRED)

include_directories(
  ${libpointmatcher_INCLUDE_DIRS}
)

add_executable(compute_mapping_error src/compute_mapping_error.cpp)
target_link_libraries(compute_mapping_error
  ${libpointmatcher_LIBRARIES}
)

install(TARGETS compute_mapping_error
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY params
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY scripts
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY cfg
  DESTINATION share/${PROJECT_NAME}
)

ament_package()

