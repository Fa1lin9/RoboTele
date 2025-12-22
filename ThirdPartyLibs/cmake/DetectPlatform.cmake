# ------------------------------------------------------------
#   Platform detection
# ------------------------------------------------------------
if (CMAKE_SYSTEM_NAME MATCHES "Linux")
    set(CURRENT_OS linux)
    # message(STATUS "The current platform is: Linux")
elseif (CMAKE_SYSTEM_NAME MATCHES "Windows")
    set(CURRENT_OS windows)
    # message(STATUS "The current platform is: Windows")
else()
    message(FATAL_ERROR "Unsupported platform: ${CMAKE_SYSTEM_NAME}")
endif()

# ------------------------------------------------------------
#   Architecture detection
# ------------------------------------------------------------
if (CMAKE_SYSTEM_PROCESSOR MATCHES "aarch64|ARM64")
    set(CURRENT_ARCH aarch64)
    # message(STATUS "The current architecture is: ARM64")
elseif (CMAKE_SYSTEM_PROCESSOR MATCHES "x86_64|AMD64")
    set(CURRENT_ARCH x86_64)
    # message(STATUS "The current architecture is: x86_64")
else()
    message(FATAL_ERROR "Unsupported architecture: ${CMAKE_SYSTEM_PROCESSOR}")
endif()
