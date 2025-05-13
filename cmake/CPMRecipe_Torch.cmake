# ============================================================================
# CPMRecipe_Torch.cmake
# ----------------------------------------------------------------------------
# LibTorch (PyTorch C++ API) - optional
# ============================================================================

if(USE_TORCH)
    CPMAddPackage(
            NAME               Torch
            URL                https://download.pytorch.org/libtorch/cpu/libtorch-cxx11-abi-shared-with-deps-2.0.1%2Bcpu.zip
            DOWNLOAD_ONLY      YES
    )
    find_package(Torch REQUIRED)
endif()