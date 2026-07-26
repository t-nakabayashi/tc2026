cmake_minimum_required(VERSION 3.8)
project(livox_sdk2_vendor)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

find_package(ament_cmake REQUIRED)

# -----------------------------------------------------------------------------
# Livox-SDK2 (third_party submodule) + GCC 13 対応パッチ適用
#
# upstream の Livox-SDK2 は <cstdint> の include 漏れにより GCC 13
# (Ubuntu 24.04 / ROS 2 Jazzy) でビルドできない。upstream PR #111 相当の
# パッチを当てて解消する。詳細は third_party/patches 配下を参照。
#
# また upstream の手順は /usr/local へ sudo install する前提だが、本パッケージは
# SDK を colcon の install 空間へ配置する。これにより livox_ros_driver2 の
# find_library(liblivox_lidar_sdk_shared.so ...) が CMAKE_PREFIX_PATH 経由で
# 解決され、sudo が不要になる。
# -----------------------------------------------------------------------------

set(LIVOX_SDK2_DIR       ${CMAKE_CURRENT_SOURCE_DIR}/third_party/Livox-SDK2)
set(LIVOX_SDK2_BUILD_DIR ${CMAKE_CURRENT_BINARY_DIR}/Livox-SDK2-patched)
set(LIVOX_SDK2_PATCH
  ${CMAKE_CURRENT_SOURCE_DIR}/third_party/patches/0001-add-cstdint-include-for-gcc13.patch)
get_filename_component(LIVOX_SDK2_GIT_CEILING_DIR ${CMAKE_CURRENT_SOURCE_DIR}/../.. ABSOLUTE)

if(NOT EXISTS ${LIVOX_SDK2_DIR}/CMakeLists.txt)
  message(FATAL_ERROR
    "Livox-SDK2 submodule is not initialized at ${LIVOX_SDK2_DIR}.\n"
    "Run: git submodule update --init --recursive")
endif()

# third_party の submodule を直接変更せず、ビルド用コピーにパッチを適用する。
file(REMOVE_RECURSE ${LIVOX_SDK2_BUILD_DIR})
file(COPY ${LIVOX_SDK2_DIR}/ DESTINATION ${LIVOX_SDK2_BUILD_DIR}
  PATTERN ".git" EXCLUDE
)

# 冪等: --check が成功 (=未適用) ならパッチを当てる。
# 失敗時は reverse --check で適用済みか確認し、どちらでもなければ不整合として停止する。
execute_process(
  COMMAND ${CMAKE_COMMAND} -E env
    GIT_CEILING_DIRECTORIES=${LIVOX_SDK2_GIT_CEILING_DIR}
    git apply --check ${LIVOX_SDK2_PATCH}
  WORKING_DIRECTORY ${LIVOX_SDK2_BUILD_DIR}
  RESULT_VARIABLE _livox_sdk2_patch_can_apply
  OUTPUT_QUIET ERROR_QUIET
)
if(_livox_sdk2_patch_can_apply EQUAL 0)
  message(STATUS "Applying Livox-SDK2 patch (0001-add-cstdint-include-for-gcc13)")
  execute_process(
    COMMAND ${CMAKE_COMMAND} -E env
      GIT_CEILING_DIRECTORIES=${LIVOX_SDK2_GIT_CEILING_DIR}
      git apply ${LIVOX_SDK2_PATCH}
    WORKING_DIRECTORY ${LIVOX_SDK2_BUILD_DIR}
    RESULT_VARIABLE _livox_sdk2_patch_result
  )
  if(NOT _livox_sdk2_patch_result EQUAL 0)
    message(FATAL_ERROR "Failed to apply Livox-SDK2 patch ${LIVOX_SDK2_PATCH}")
  endif()
else()
  execute_process(
    COMMAND ${CMAKE_COMMAND} -E env
      GIT_CEILING_DIRECTORIES=${LIVOX_SDK2_GIT_CEILING_DIR}
      git apply --reverse --check ${LIVOX_SDK2_PATCH}
    WORKING_DIRECTORY ${LIVOX_SDK2_BUILD_DIR}
    RESULT_VARIABLE _livox_sdk2_patch_already_applied
    OUTPUT_QUIET ERROR_QUIET
  )
  if(_livox_sdk2_patch_already_applied EQUAL 0)
    message(STATUS "Livox-SDK2 patch already applied")
  else()
    message(FATAL_ERROR "Failed to apply Livox-SDK2 patch ${LIVOX_SDK2_PATCH}")
  endif()
endif()

# upstream のルート CMakeLists.txt 相当の設定。
# サンプル (samples/) は実機運用に不要なため sdk_core のみをビルドする。
if(UNIX)
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -pthread")
endif()

add_subdirectory(
  ${LIVOX_SDK2_BUILD_DIR}/sdk_core
  ${CMAKE_CURRENT_BINARY_DIR}/Livox-SDK2-build
)

ament_package()
