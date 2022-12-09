cmake_minimum_required(VERSION 3.2)
project(mujoco_wasm)

# add_subdirectory(division)
set(SOURCE_FILES main.cc)

add_executable(mujoco_wasm ${SOURCE_FILES})

target_link_libraries(mujoco_wasm ccd elasticity lodepng mujoco tinyxml2 qhullstatic_r)
install(TARGETS mujoco_wasm DESTINATION ${DIVISIBLE_INSTALL_BIN_DIR})
