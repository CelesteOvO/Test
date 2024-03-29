//
// Created by LiYifan on 2024/3/6.
//

#ifndef HINAPE_HOUDINI_SAB_H
#define HINAPE_HOUDINI_SAB_H

#include "AlignedBox.h"
#include "CollisionDetectionBroadPhase.h"
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
    SemiAnalyticalSolver();
    void Solve(real dt, std::vector<AlignedBox>& aabb_point, std::vector<AlignedBox>& aabb_triangle) const;
public:
    VectorArrayCPU vertices;
    std::vector<std::vector<size_t>> faces;
    std::vector<AlignedBox> triangleAABBs;

    std::shared_ptr<CollisionDetectionBroadPhase> mBroadPhaseCD;
public:
};

#endif //HINAPE_HOUDINI_SAB_H
