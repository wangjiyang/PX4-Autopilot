# Android NDK cross toolchain (arm64-v8a)
#
# Used by boards/droid/phone to build the PX4 posix binary so it can run
# inside an Android app process (packaged as a native library / exec'd
# from the app's nativeLibraryDir).
#
# Requires ANDROID_NDK_HOME pointing at an NDK installation (r26+).
# Relies on CMake's built-in NDK support (CMAKE_SYSTEM_NAME=Android), which
# also enables the existing "NOT ANDROID" guards in platforms/posix (no
# librt / libpthread on bionic).

if(NOT DEFINED ENV{ANDROID_NDK_HOME})
	message(FATAL_ERROR "ANDROID_NDK_HOME must be set to an Android NDK path")
endif()

set(CMAKE_SYSTEM_NAME Android)
set(CMAKE_SYSTEM_VERSION 30)
set(CMAKE_ANDROID_ARCH_ABI arm64-v8a)
set(CMAKE_ANDROID_NDK $ENV{ANDROID_NDK_HOME})
set(CMAKE_ANDROID_STL_TYPE c++_static)

# PX4 uses VLAs in C++ in several places; NDK clang 18+ warns (and PX4
# builds with -Werror), while gcc and older clang accepted them silently.
set(CMAKE_C_FLAGS_INIT "-Wno-vla-cxx-extension")
set(CMAKE_CXX_FLAGS_INIT "-Wno-vla-cxx-extension")

set(CMAKE_FIND_ROOT_PATH_MODE_PROGRAM NEVER)
set(CMAKE_FIND_ROOT_PATH_MODE_LIBRARY ONLY)
set(CMAKE_FIND_ROOT_PATH_MODE_INCLUDE ONLY)
