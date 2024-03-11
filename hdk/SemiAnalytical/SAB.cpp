//
// Created by LiYifan on 2024/3/6.
//

#include <SIM/SIM_ObjectArray.h>
#include "SAB.h"
#include "PBF/PBF.h"

SemiAnalyticalSolver::SemiAnalyticalSolver(std::vector<AlignedBox> &pointAABBs) {
    vertices.clear();
    faces.clear();
    triangleAABBs.clear();

    mBroadPhaseCD = std::make_shared<CollisionDetectionBroadPhase>(pointAABBs, triangleAABBs);
}

void SemiAnalyticalSolver::Solve(real dt) {
    mBroadPhaseCD->doCollisionWithLinearBVH(); // 得到contact list
}
