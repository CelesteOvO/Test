//
// Created by LiYifan on 2024/3/6.
//

#include <SIM/SIM_ObjectArray.h>
#include "SAB.h"
#include "PBF/PBF.h"

SemiAnalyticalSolver::SemiAnalyticalSolver()
{
    mBroadPhaseCD = std::make_shared<CollisionDetectionBroadPhase>();
}

void SemiAnalyticalSolver::Solve(real dt, std::vector<AlignedBox>& aabb_point, std::vector<AlignedBox>& aabb_triangle) const
{
    mBroadPhaseCD->updateAABB(aabb_point, aabb_triangle);
    //mBroadPhaseCD->printAABB();
    mBroadPhaseCD->doCollisionWithLinearBVH(); // 得到contact list*/
}
