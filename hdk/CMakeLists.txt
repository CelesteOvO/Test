set(SIM_HinaPE_TARGET SIM_Test_Fluid)

file(GLOB_RECURSE SIM_HinaPE_HEADER *.h)
file(GLOB_RECURSE SIM_HinaPE_SOURCE *.cpp)

add_library(
        ${SIM_HinaPE_TARGET}
        SHARED
        ${SIM_HinaPE_HEADER}
        ${SIM_HinaPE_SOURCE}

        Common/Particles.h
        Common/kernels.h
        Common/emitter.h
        Common/neighbors.h
        Common/geometry.h

        Base/SIM_Hina_Particles.cpp
        Base/SIM_Hina_Particles.h

        PBF/SIM_Hina_Particles_PBF.cpp
        PBF/SIM_Hina_Particles_PBF.h
        PBF/GAS_Hina_Solver_PBF.cpp
        PBF/GAS_Hina_Solver_PBF.h
        PBF/PBF.cpp
        PBF/PBF.h

        Rigid/GAS_Hina_Solver_Rigid.cpp
        Rigid/GAS_Hina_Solver_Rigid.h
        Rigid/SIM_Hina_RigidBody.cpp
        Rigid/SIM_Hina_RigidBody.h

        GAS_Test_CFL_SubStep.cpp
        GAS_Test_CFL_SubStep.h
        SIM_Hina_Generator.h
        SemiAnalytical/SAB.cpp
        SemiAnalytical/SAB.h
        SemiAnalytical/AlignedBox.h
        SemiAnalytical/SIM_Hina_SemiAnalyticalBoundary.cpp
        SemiAnalytical/SIM_Hina_SemiAnalyticalBoundary.h
        SemiAnalytical/GAS_Hina_SemiAnalyticalSolver.cpp
        SemiAnalytical/GAS_Hina_SemiAnalyticalSolver.h
        SemiAnalytical/LinearBVH.cpp
        SemiAnalytical/LinearBVH.h
)

# Link Houdini Toolkit
target_link_libraries(
        ${SIM_HinaPE_TARGET}
        PUBLIC
        Houdini
        HinaPE
        HinaPE_CUDA
)
target_include_directories(
        ${SIM_HinaPE_TARGET}
        PUBLIC
        ${CMAKE_CURRENT_BINARY_DIR}
        ${CMAKE_CURRENT_SOURCE_DIR}
)
target_compile_options(
        ${SIM_HinaPE_TARGET}
        PRIVATE
        -D_LIBCPP_ENABLE_CXX17_REMOVED_UNARY_BINARY_FUNCTION
)
houdini_configure_target(${SIM_HinaPE_TARGET})
