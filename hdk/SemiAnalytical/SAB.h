//
// Created by LiYifan on 2024/3/6.
//

#ifndef HINAPE_HOUDINI_SAB_H
#define HINAPE_HOUDINI_SAB_H

#include "AlignedBox.h"
#include <vector>
#include <UT/UT_Vector3.h>

using real = float;
using Vector = UT_Vector3T<real>;
using ScalarArrayCPU = std::vector<real>;
using VectorArrayCPU = std::vector<Vector>;

struct TriangleSet
{
    VectorArrayCPU vertices;
    std::vector<std::vector<size_t>> faces;
    std::vector<AlignedBox> triangleAABBs;
};

struct LabeledTriangleMesh {
    //std::string objectId;
    TriangleSet triangleSet;
};

struct SemiAnalyticalBoundaryCPU
{
    std::vector<LabeledTriangleMesh> meshes;
};

struct SemiAnalyticalParamCPU
{
};

struct SemiAnalyticalSolver : public SemiAnalyticalParamCPU
{
public:
    VectorArrayCPU vertices;
    std::vector<std::vector<size_t>> faces;
    std::vector<AlignedBox> triangleAABBs;

    struct ContactPair {
        AlignedBox* box_src;
        AlignedBox* box_tar;
    };
    std::vector<ContactPair> contactPairs;
public:
    void doCollisionWithLinearBVH(std::vector<AlignedBox> &aabb_src, std::vector<AlignedBox> &aabb_tar);
};

#endif //HINAPE_HOUDINI_SAB_H
