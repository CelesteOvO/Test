//
// Created by LiYifan on 2024/3/5.
//

#ifndef HINAPE_HOUDINI_SIM_HINA_PARTICLES_PBF_H
#define HINAPE_HOUDINI_SIM_HINA_PARTICLES_PBF_H

#include <Base/SIM_Hina_Particles.h>
#include "SemiAnalytical/AlignedBox.h"

SIM_HINA_DERIVED_GEOMETRY_CLASS(
        Particles_PBF,
        Particles,
        void commit() override;

        ScalarArrayCPU *lambda;
        VectorArrayCPU *pred_x, *delta_p, *a_ext;
        std::vector<AlignedBox> *pointAABB;
)

#endif //HINAPE_HOUDINI_SIM_HINA_PARTICLES_PBF_H
