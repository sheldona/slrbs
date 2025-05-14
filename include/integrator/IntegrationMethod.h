#pragma once

/// All supported integration schemes.
enum class IntegrationMethod {
    EXPLICIT_EULER,
    SYMPLECTIC_EULER,
    VERLET,
    RK4,
    IMPLICIT_EULER,
    NEWTON
};
