//
// Created by LiYifan on 2024/3/11.
//

#ifndef HINAPE_HOUDINI_COLLISIONDETECTIONBROADPHASE_H
#define HINAPE_HOUDINI_COLLISIONDETECTIONBROADPHASE_H

#include <iostream>
#include <vector>
#include <UT/UT_Vector3.h>
#include "AlignedBox.h"
#include "LinearBVH.h"

using real = float;

class CollisionDetectionBroadPhase
{
public:
    CollisionDetectionBroadPhase() = default;
    ~CollisionDetectionBroadPhase() = default;
    std::vector<AlignedBox> aabb_src;
    std::vector<AlignedBox> aabb_tar;
    std::vector<std::vector<size_t>> contactPairs;

    std::vector<size_t> mCounter;
public:
    void doCollisionWithLinearBVH();
    void CDBP_RequestIntersectionNumberBVH(LinearBVH bvh, bool selfCollision);
    void CDBP_RequestIntersectionIdsBVH(LinearBVH bvh, bool selfCollision);
public:
    void updateAABB(std::vector<AlignedBox>& aabb_src, std::vector<AlignedBox>& aabb_tar);
    /// For Test
    void printAABB();
};

#endif //HINAPE_HOUDINI_COLLISIONDETECTIONBROADPHASE_H
