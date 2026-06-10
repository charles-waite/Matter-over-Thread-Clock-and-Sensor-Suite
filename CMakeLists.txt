cmake_minimum_required(VERSION 3.16)

set(PROJECT_VER "0.2.2")
set(PROJECT_VER_NUMBER 202)

set(EXTRA_COMPONENT_DIRS components)

include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(esp_clock)
