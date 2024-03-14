//
// Created by LiYifan on 2024/3/11.
//

#include <numeric>
#include "CollisionDetectionBroadPhase.h"
#include "LinearBVH.h"

void CollisionDetectionBroadPhase::doCollisionWithLinearBVH()
{
    /*std::vector<AlignedBox> AABBs {
            AlignedBox(UT_Vector3(0, 0, 0), UT_Vector3(1, 1, 1)),
            AlignedBox(UT_Vector3(4, 4, 4), UT_Vector3(5, 5, 5)),
            AlignedBox(UT_Vector3(2, 2, 2), UT_Vector3(3, 3, 3)),
    };*/

    LinearBVH bvh;
    bvh.construct(aabb_tar);

    /*mCounter.resize(aabb_src.size());
    CDBP_RequestIntersectionNumberBVH(bvh, false);

    size_t total = std::accumulate(mCounter.begin(), mCounter.end(), 0);
    contactPairs.resize(total);

    CDBP_RequestIntersectionIdsBVH(bvh, false);*/

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

void CollisionDetectionBroadPhase::updateAABB(std::vector<AlignedBox> &aabb_src, std::vector<AlignedBox> &aabb_tar) {
    this->aabb_src = aabb_src;
    this->aabb_tar = aabb_tar;
    mCounter.resize(aabb_src.size());
    contactPairs.resize(aabb_src.size() * aabb_tar.size());
}

void CollisionDetectionBroadPhase::printAABB() {
    std::cout << "Source AABB: " << std::endl;
    std::cout << "Size: " << aabb_src.size() << std::endl;
    for(int i = 0; i < 10; i++)
    {
        std::cout << "(" << aabb_src[i].v0 << "," << aabb_src[i].v1 << ")" << std::endl;
    }
    /*for(auto &aabb : aabb_src)
    {
        std::cout << "(" << aabb.v0 << "," << aabb.v1 << ")" << std::endl;
    }*/
    /*std::cout << "Target AABB: " << std::endl;
    for(auto &aabb : aabb_tar)
    {
        std::cout << "(" << aabb.v0 << "," << aabb.v1 << ")" << std::endl;
    }*/
    /*std::cout << "Contact Pairs: " << std::endl;
    for(auto &pairs : contactPairs)
    {
        for(auto &pair : pairs)
        {
            std::cout << pair << " ";
        }
        std::cout << std::endl;
    }*/
}


