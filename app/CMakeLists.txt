# Copyright (C) 2026 Olsc <OlscStudio@outlook.com>
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#      http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

cmake_minimum_required(VERSION 3.22.1)
project("glb_renderer")

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

include_directories(
    ${CMAKE_CURRENT_SOURCE_DIR}/src/main/cpp/glb
    ${CMAKE_CURRENT_SOURCE_DIR}/src/main/cpp/Thirdparty/cgltf
)

add_library(glb_renderer SHARED
    src/main/cpp/glb/glb_model.cpp
    src/main/cpp/glb/glb_renderer.cpp
    src/main/cpp/glb/glb_jni.cpp
)

target_link_libraries(glb_renderer
    GLESv2
    EGL
    android
    log
    m
)
