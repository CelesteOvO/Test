//
// Created by LiYifan on 2024/3/11.
//

#include <numeric>
#include "CollisionDetectionBroadPhase.h"
#include "LinearBVH.h"

CollisionDetectionBroadPhase::CollisionDetectionBroadPhase(std::vector<AlignedBox> &aabb_src,
                                                           std::vector<AlignedBox> &aabb_tar) : aabb_src(aabb_src),
                                                                                                aabb_tar(aabb_tar)
{
    mCounter.resize(aabb_src.size());
    contactPairs.resize(aabb_src.size() * aabb_tar.size());
}

void CollisionDetectionBroadPhase::doCollisionWithLinearBVH()
{
    LinearBVH bvh;
    bvh.construct(aabb_tar);

    mCounter.resize(aabb_src.size());
    CDBP_RequestIntersectionNumberBVH(bvh, false);

    size_t total = std::accumulate(mCounter.begin(), mCounter.end(), 0);
    contactPairs.resize(total);

    CDBP_RequestIntersectionIdsBVH(bvh, false);

    bvh.release();
}

void CollisionDetectionBroadPhase::CDBP_RequestIntersectionNumberBVH(LinearBVH bvh, bool selfCollision)
{
    size_t n = aabb_src.size();
    for(size_t i = 0; i < n; i++)
    {
        if(selfCollision)
        {
            mCounter[i] = bvh.requestIntersectionNumber(aabb_src[i], i);
        }
        else
        {
            mCounter[i] = bvh.requestIntersectionNumber(aabb_src[i]);
        }
    }
}

void CollisionDetectionBroadPhase::CDBP_RequestIntersectionIdsBVH(LinearBVH bvh, bool selfCollision) {
    size_t n = aabb_src.size();
    for(size_t i = 0; i < n; i++)
    {
        if(selfCollision)
        {
            bvh.requestIntersectionIds(contactPairs[i], aabb_src[i], i);
        }
        else
        {
            bvh.requestIntersectionIds(contactPairs[i], aabb_src[i]);
        }
    }

}


