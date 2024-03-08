//
// Created by LiYifan on 2024/3/7.
//

#ifndef HINAPE_HOUDINI_SIM_HINA_SEMIANALYTICALBOUNDARY_H
#define HINAPE_HOUDINI_SIM_HINA_SEMIANALYTICALBOUNDARY_H

#include <SIM_Hina_Generator.h>
#include "AlignedBox.h"

using real = float;
using Vector = UT_Vector3T<real>;
using ScalarArrayCPU = std::vector<real>;
using VectorArrayCPU = std::vector<Vector>;

SIM_HINA_GEOMETRY_CLASS(
        SemiAnalyticalBoundary,
        virtual void load_gdp(const GU_Detail *gdp);
        virtual void commit();

        std::map<GA_Offset, GA_Size> offset2index;
        std::map<GA_Size, GA_Offset> index2offset;

        VectorArrayCPU *vertices;
        std::vector<std::vector<size_t>> *faces;
        std::vector<AlignedBox> *triangleAABBs;
)

auto FetchAllSemiAnalyticalBoundaries(SIM_Object *fluid_obj) -> std::vector<SIM_Hina_SemiAnalyticalBoundary *>;
void InitAllSemiAnalyticalBoundaries(SIM_Object *fluid_obj);

#endif //HINAPE_HOUDINI_SIM_HINA_SEMIANALYTICALBOUNDARY_H
