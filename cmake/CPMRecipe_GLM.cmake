# ============================================================================
# CPMRecipe_GLM.cmake
# ----------------------------------------------------------------------------
# GLM (header-only math library)
# ============================================================================

if(NOT TARGET glm AND NOT TARGET glm-header-only)
    find_package(glm QUIET)
    if(NOT glm_FOUND)
        CPMAddPackage(
                NAME               glm
                GITHUB_REPOSITORY  g-truc/glm
                GIT_TAG            master
        )
    endif()
endif()