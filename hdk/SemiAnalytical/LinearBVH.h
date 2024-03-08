//
// Created by LiYifan on 2024/3/8.
//

#ifndef HINAPE_HOUDINI_LINEARBVH_H
#define HINAPE_HOUDINI_LINEARBVH_H

#include <iostream>
#include "AlignedBox.h"

class BVHNode
{
public:
    BVHNode(){
        parent = -1;
        left = -1;
        right = -1;
    }

    bool isLeaf(){
        return left == -1 && right == -1;
    }

    size_t parent;
    size_t left;
    size_t right;
};

class LinearBVH
{
public:
    LinearBVH() = default;
    ~LinearBVH() = default;

    void construct(std::vector<AlignedBox> &boxes);
    void clear();

public:
    static void LBVH_CalculateCenter(std::vector<UT_Vector3> &center, std::vector<AlignedBox> &aabb);
    static void LBVH_CalculateMortonCodes(std::vector<size_t> &morton, std::vector<size_t> &objectId, std::vector<UT_Vector3> &center, UT_Vector3 origin, fpreal L);
    static void LBVH_InitialAllNodes(std::vector<BVHNode> &bvhNodes);
    static void LBVH_ConstructBinaryRadixTree(std::vector<BVHNode> &bvhNodes, std::vector<AlignedBox> &sortedAABBs, std::vector<AlignedBox> &aabbs, std::vector<size_t> &mortonCodes, std::vector<size_t> &sortedObjectIds);
    void LBVH_CalculateBoundingBox(std::vector<AlignedBox> &sortedAABBs, std::vector<BVHNode> &bvhNodes, std::vector<size_t> &flags);
    static size_t morton3D(fpreal x, fpreal y, fpreal z);
    static size_t expandBits(size_t v);
public:
    std::vector<BVHNode> mAllNodes;
    std::vector<UT_Vector3> mCenters;
    std::vector<AlignedBox> mSortedAABBs;
    std::vector<size_t> mSortedObjectIds;
    std::vector<size_t> mFlags;
    std::vector<size_t> mMortonCodes;
};



#endif //HINAPE_HOUDINI_LINEARBVH_H
