//
// Created by LiYifan on 2024/3/7.
//

#ifndef HINAPE_HOUDINI_GAS_HINA_SEMIANALYTICALSOLVER_H
#define HINAPE_HOUDINI_GAS_HINA_SEMIANALYTICALSOLVER_H

#include <SIM_Hina_Generator.h>
#include "SAB.h"
#include "SIM_Hina_SemiAnalyticalBoundary.h"

GAS_HINA_SUBSOLVER_CLASS(
        SemiAnalyticalSolver,

        std::shared_ptr<SemiAnalyticalSolver> SemiAnalyticalSolverPtr;
        bool inited;

        void init_data(SIM_Hina_SemiAnalyticalBoundary*, SIM_Object *);
)

#endif //HINAPE_HOUDINI_GAS_HINA_SEMIANALYTICALSOLVER_H
