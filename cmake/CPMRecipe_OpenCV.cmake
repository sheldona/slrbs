# ============================================================================
# CPMRecipe_OpenCV.cmake
# ----------------------------------------------------------------------------
# OpenCV (computer vision) - optional
# ============================================================================

if(USE_OPENCV)
    CPMAddPackage(
            NAME               OpenCV
            GITHUB_REPOSITORY  opencv/opencv
            GIT_TAG            4.8.0
            OPTIONS            "BUILD_EXAMPLES=OFF" "BUILD_TESTS=OFF" "WITH_CUDA=OFF"
    )
endif()