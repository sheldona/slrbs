# ─────────────────────────────────────────────────────────────────────────────
#  User-configurable options
# ─────────────────────────────────────────────────────────────────────────────
option(BUILD_TESTING        "Enable testing"                          ON)

# ---- viewer selection ------------------------------------------------
option(USE_QT               "Enable Qt support"                       OFF)
option(USE_POLYSCOPE        "Enable Polyscope-based viewer"           ON)

# ---- optional libs ---------------------------------------------------
option(USE_SYSTEM_QT        "Use system Qt5/6 installation"           OFF)
option(USE_OPENCV           "Enable OpenCV support"                   OFF)
option(USE_TORCH            "Enable LibTorch support"                 OFF)
option(USE_OPENMP           "Enable OpenMP"                           ON)
option(USE_OPENMP_SIMD      "Enable OpenMP SIMD directives"           ON)
option(USE_SIMD_INTRINSICS  "Enable low‑level SIMD intrinsics"        ON)
option(USE_PYBIND           "Enable Python bindings via pybind11"     OFF)
option(USE_CCACHE           "Enable C/C++ compiler caching with ccache" ON)
option(USE_TCNN             "Enable tiny‑cuda‑nn neural network support" OFF)