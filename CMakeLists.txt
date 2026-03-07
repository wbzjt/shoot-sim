cmake_minimum_required(VERSION 3.22)
project(FRCShooterTools LANGUAGES CXX)

option(BUILD_BESTSHOOTSIM "Build stage-1 shooter simulation tool" ON)
option(BUILD_CALLI "Build stage-2 calibration tool" ON)

if(BUILD_BESTSHOOTSIM)
  add_subdirectory(bestshootsim)
endif()

if(BUILD_CALLI)
  add_subdirectory(calli)
endif()

