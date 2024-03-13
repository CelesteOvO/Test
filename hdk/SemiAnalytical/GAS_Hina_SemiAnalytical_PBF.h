//
// Created by LiYifan on 2024/3/12.
//

#ifndef HINAPE_HOUDINI_GAS_HINA_SEMIANALYTICAL_PBF_H
#define HINAPE_HOUDINI_GAS_HINA_SEMIANALYTICAL_PBF_H

#include <SIM_Hina_Generator.h>
#include "SAB.h"
#include "SIM_Hina_SemiAnalyticalBoundary.h"
#include <PBF/SIM_Hina_Particles_PBF.h>
#include <PBF/PBF.h>

GAS_HINA_SUBSOLVER_CLASS(
        SemiAnalytical_PBF,
        HINA_GETSET_PARAMETER(FluidDomain, GETSET_DATA_FUNCS_V3)
        HINA_GETSET_PARAMETER(TopOpen, GETSET_DATA_FUNCS_B)
        HINA_GETSET_PARAMETER(TargetSpacing, GETSET_DATA_FUNCS_F)
        HINA_GETSET_PARAMETER(KernelRadius, GETSET_DATA_FUNCS_F)
        HINA_GETSET_PARAMETER(TargetDensity, GETSET_DATA_FUNCS_F)
        HINA_GETSET_PARAMETER(Gravity, GETSET_DATA_FUNCS_V3)
        HINA_GETSET_PARAMETER(MaxNumOfParticles, GETSET_DATA_FUNCS_I)
        HINA_GETSET_PARAMETER(IsOneShot, GETSET_DATA_FUNCS_B)
        HINA_GETSET_PARAMETER(EmitStart, GETSET_DATA_FUNCS_V3)
        HINA_GETSET_PARAMETER(EmitEnd, GETSET_DATA_FUNCS_V3)
        HINA_GETSET_PARAMETER(Kernel, GETSET_DATA_FUNCS_I)

        std::shared_ptr<SemiAnalyticalSolver> SemiAnalyticalSolverPtr;
        std::shared_ptr<PbfSolver> PBFSolverPtr;
        bool fluid_inited, boundary_inited;
        bool fluid_emitted;

        void init_fluid_data(SIM_Hina_Particles_PBF*, SIM_Object *);
        void emit_fluid_data(SIM_Hina_Particles_PBF*);
        void init_boundary_data(SIM_Hina_SemiAnalyticalBoundary*, SIM_Object *);
)

#endif //HINAPE_HOUDINI_GAS_HINA_SEMIANALYTICAL_PBF_H
