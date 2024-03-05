set(HINA_TARGET SIM_Test_Fluid)

file(GLOB_RECURSE HINAPE_EADER *.h)
file(GLOB_RECURSE HINAPE_SOURCE *.cpp)

add_library(
        ${HINA_TARGET}
        SHARED
        ${HINAPE_HEADER}
        ${HINAPE_SOURCE}

        Common/Particles.h
        Common/kernels.h
        Common/emitter.h
        Common/neighbors.h
        PBF/PBF.cpp
        PBF/PBF.h

        Base/SIM_Hina_Particles.cpp
        Base/SIM_Hina_Particles.h

        PBF/SIM_Hina_Particles_PBF.cpp
        PBF/SIM_Hina_Particles_PBF.h
        PBF/GAS_Hina_Solver_PBF.cpp
        PBF/GAS_Hina_Solver_PBF.h

        GAS_Test_CFL_SubStep.cpp
        GAS_Test_CFL_SubStep.h
        SIM_Hina_Generator.h
)

# Link Houdini Toolkit
target_link_libraries(
        ${HINA_TARGET}
        PUBLIC
        Houdini
        CUDA_CubbyFlow
        CUDA_HinaPE
        TreeNSearch
)
if (MSVC)
    target_link_libraries(
            ${HINA_TARGET}
            PUBLIC
            CUDA_NeighborSearch
    )
endif ()
target_include_directories(
        ${HINA_TARGET}
        PUBLIC
        ${CMAKE_CURRENT_BINARY_DIR}
        ${CMAKE_CURRENT_SOURCE_DIR}
)
target_compile_options(
        ${HINA_TARGET}
        PRIVATE
        -D_LIBCPP_ENABLE_CXX17_REMOVED_UNARY_BINARY_FUNCTION
)
houdini_configure_target(${HINA_TARGET})
