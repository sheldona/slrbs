# ============================================================================
# CPMBootstrap.cmake
# ----------------------------------------------------------------------------
# Bootstraps CPM.cmake for subsequent recipe includes.
# ============================================================================

set(CPM_DOWNLOAD_VERSION 0.38.1)
set(CPM_DOWNLOAD_LOCATION "${CMAKE_BINARY_DIR}/cmake/CPM_${CPM_DOWNLOAD_VERSION}.cmake")
if(NOT EXISTS ${CPM_DOWNLOAD_LOCATION})
    message(STATUS "Downloading CPM.cmake v${CPM_DOWNLOAD_VERSION} to ${CPM_DOWNLOAD_LOCATION}")
    file(DOWNLOAD
            "https://github.com/cpm-cmake/CPM.cmake/releases/download/v${CPM_DOWNLOAD_VERSION}/CPM.cmake"
            ${CPM_DOWNLOAD_LOCATION}
            SHOW_PROGRESS
    )
endif()
include(${CPM_DOWNLOAD_LOCATION})