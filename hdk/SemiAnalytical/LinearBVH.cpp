//
// Created by LiYifan on 2024/3/8.
//

#include "LinearBVH.h"
#include <thrust/extrema.h>
#include <thrust/sort.h>
#include <bitset>
#include <stack>

void LinearBVH::construct(std::vector<AlignedBox> &aabb) {
    size_t num = aabb.size();

    if (mCenters.size() != num){
        mCenters.resize(num);
        mMortonCodes.resize(num);
        mSortedObjectIds.resize(num);
        mFlags.resize(num);

        mSortedAABBs.resize(2 * num - 1);
        mAllNodes.resize(2 * num - 1);
    }

    LBVH_CalculateCenter(mCenters, aabb);

    auto minmax = thrust::minmax_element(mCenters.begin(), mCenters.end());
    UT_Vector3 v_min = *minmax.first;
    UT_Vector3 v_max = *minmax.second;

    fpreal L = std::max(v_max[0] - v_min[0], std::max(v_max[1] - v_min[1], v_max[2] - v_min[2]));
    L = L < (std::numeric_limits<fpreal>::epsilon)() ? fpreal(1) : L;

    UT_Vector3 origin = fpreal(0.5) * (v_min + v_max) - fpreal(0.5) * L;

    LBVH_CalculateMortonCodes(mMortonCodes, mSortedObjectIds, mCenters, origin, L);
    thrust::sort_by_key(
            mMortonCodes.begin(),
            mMortonCodes.end(),
            mSortedObjectIds.begin());

    LBVH_InitialAllNodes(mAllNodes);
    LBVH_ConstructBinaryRadixTree(mAllNodes, mSortedAABBs, aabb, mMortonCodes, mSortedObjectIds);

    std::fill(mFlags.begin(), mFlags.end(), 0);
    LBVH_CalculateBoundingBox(mSortedAABBs, mAllNodes, mFlags);
}

void LinearBVH::release() {
    mAllNodes.clear();
    mCenters.clear();
    mSortedAABBs.clear();
    mSortedObjectIds.clear();
    mFlags.clear();
    mMortonCodes.clear();
}

void LinearBVH::LBVH_CalculateCenter(std::vector<UT_Vector3> &center, std::vector<AlignedBox> &aabb) {
    size_t num = aabb.size();
    for (size_t i = 0; i < num; i++){
        center[i] = (aabb[i].v0 + aabb[i].v1) * 0.5;
    }
}

void LinearBVH::LBVH_CalculateMortonCodes(std::vector<size_t> &morton, std::vector<size_t> &objectId,
                                          std::vector<UT_Vector3> &center, UT_Vector3 origin, fpreal L) {
    size_t num = center.size();
    for(size_t i = 0; i < num; i++){
        UT_Vector3 scaled = (center[i] - origin) / L;

        size_t m64 = morton3D(scaled.x(), scaled.y(), scaled.z());
        m64 <<= 32;
        m64 |= i;

        morton[i] = m64;
        objectId[i] = i;
    }
}

size_t LinearBVH::morton3D(fpreal x, fpreal y, fpreal z) {
    x = std::min(std::max(x * fpreal(1024), fpreal(0)), fpreal(1023));
    y = std::min(std::max(y * fpreal(1024), fpreal(0)), fpreal(1023));
    z = std::min(std::max(z * fpreal(1024), fpreal(0)), fpreal(1023));
    uint xx = expandBits((uint)x);
    uint yy = expandBits((uint)y);
    uint zz = expandBits((uint)z);
    return xx * 4 + yy * 2 + zz;
}

size_t LinearBVH::expandBits(size_t v) {
    v = (v * 0x00010001u) & 0xFF0000FFu;
    v = (v * 0x00000101u) & 0x0F00F00Fu;
    v = (v * 0x00000011u) & 0xC30C30C3u;
    v = (v * 0x00000005u) & 0x49249249u;
    return v;
}

void LinearBVH::LBVH_InitialAllNodes(std::vector<BVHNode> &bvhNodes) {
    for(auto & bvhNode : bvhNodes){
        bvhNode = BVHNode();
    }
}

void LinearBVH::LBVH_ConstructBinaryRadixTree(std::vector<BVHNode> &bvhNodes, std::vector<AlignedBox> &sortedAABBs,
                                              std::vector<AlignedBox> &aabbs, std::vector<size_t> &mortonCodes,
                                              std::vector<size_t> &sortedObjectIds) {
    size_t N = sortedObjectIds.size();

    for (int i = 0; i < N; ++i)
    {
        sortedAABBs[i + N - 1] = aabbs[sortedObjectIds[i]];

        if (i >= N - 1)
            continue;

        auto delta = [&](int _i, int _j) -> int {
            if (_j < 0 || _j >= N)
                return -1;
            uint64_t value = mortonCodes[_i] ^ mortonCodes[_j];
            std::bitset<64> bs(value);
            return bs.size() - bs.to_ullong();
        };

        int d = delta(i, i + 1) - delta(i, i - 1) > 0 ? 1 : -1;

        int delta_min = delta(i, i - d);

        int len_max = 2;
        while (delta(i, i + len_max * d) > delta_min) {
            len_max *= 2;
        }

        int len = 0;
        for (int t = len_max / 2; t > 0; t = t / 2) {
            if (delta(i, i + (len + t) * d) > delta_min) {
                len = len + t;
            }
        }

        int j = i + len * d;

        int delta_node = delta(i, j);
        int s = 0;

        for (int t = (len + 1) / 2; t > 0; t = t == 1 ? 0 : (t + 1) / 2)
        {
            if (delta(i, i + (s + t) * d) > delta_node) {
                s = s + t;
            }
        }
        size_t gamma = i + s * d + std::min(d, 0);

        size_t left_idx = std::min(i, j) == gamma ? gamma + N - 1 : gamma;
        size_t right_idx = std::max(i, j) == gamma + 1 ? gamma + N : gamma + 1;

        bvhNodes[i].left = left_idx;
        bvhNodes[i].right = right_idx;

        bvhNodes[left_idx].parent = i;
        bvhNodes[right_idx].parent = i;
    }

}

void LinearBVH::LBVH_CalculateBoundingBox(std::vector<AlignedBox> &sortedAABBs, std::vector<BVHNode> &bvhNodes,
                                          std::vector<size_t> &flags) {
    size_t N = flags.size();
    for(size_t i = 0; i < N; i++){
        size_t idx = bvhNodes[i + N - 1].parent;
        while (idx != -1)
        {
            auto* flags_ptr = reinterpret_cast<std::atomic<size_t>*>(flags.data()); // 获取指向flags数据的原子指针
            std::atomic<size_t>& flag = flags_ptr[idx]; // 获取位置idx的原子引用
            size_t expected = 0;
            size_t desired = 1;
            bool success = flag.compare_exchange_strong(expected, desired); // 尝试设置新值
            int old = success ? 0 : flag.load(); // 获取旧值
            if (old == 0)
            {
                return;
            }

            const size_t l_idx = bvhNodes[idx].left;
            const size_t r_idx = bvhNodes[idx].right;
            const AlignedBox l_aabb = sortedAABBs[l_idx];
            const AlignedBox r_aabb = sortedAABBs[r_idx];
            sortedAABBs[idx] = l_aabb.merge(r_aabb);

            idx = bvhNodes[idx].parent;
        }
    }
}

size_t LinearBVH::requestIntersectionNumber(const AlignedBox &queryBox, const int queryId)
{
    std::array<int, 64> buffer;

    std::stack<int> stack;

    uint N = mSortedObjectIds.size();

    // Traverse nodes starting from the root.
    uint ret = 0;
    int idx = 0;
    do
    {
        // Check each child node for overlap.
        int idxL = mAllNodes[idx].left;
        int idxR = mAllNodes[idx].right;
        bool overlapL = queryBox.checkOverlap(mSortedAABBs[idxL]);
        bool overlapR = queryBox.checkOverlap(mSortedAABBs[idxR]);

        // Query overlaps a leaf node => report collision.
        if (overlapL && mAllNodes[idxL].isLeaf()) {
            int objId = mSortedObjectIds[idxL - N + 1];
            if(objId > queryId) ret++;
        }
        if (overlapR && mAllNodes[idxR].isLeaf()) {
            int objId = mSortedObjectIds[idxR - N + 1];
            if (objId > queryId) ret++;
        }

        // Query overlaps an internal node => traverse.
        bool traverseL = overlapL && !mAllNodes[idxL].isLeaf();
        bool traverseR = overlapR && !mAllNodes[idxR].isLeaf();

        if (!traverseL && !traverseR) {
            idx = stack.size() != 0 ? stack.top() : -1; // pop
            if (stack.size() != 0) stack.pop();
        }
        else
        {
            idx = traverseL ? idxL : idxR;
            if (traverseL && traverseR)
                stack.push(idxR); // push
        }
    } while (idx != -1);

    return ret;
}

void LinearBVH::requestIntersectionIds(std::vector<size_t> &ids, const AlignedBox &queryBox, const int queryId)
{
    std::deque<int> stack;

    size_t N = mSortedObjectIds.size();

    int idx = 0;
    const int EMPTY = -1;

    do
    {
        int idxL = mAllNodes[idx].left;
        int idxR = mAllNodes[idx].right;
        bool overlapL = queryBox.checkOverlap(mSortedAABBs[idxL]);
        bool overlapR = queryBox.checkOverlap(mSortedAABBs[idxR]);

        if (overlapL && mAllNodes[idxL].isLeaf()) {
            int objId = mSortedObjectIds[idxL - N + 1];
            if (objId > queryId)
                ids.push_back(objId);
        }

        if (overlapR && mAllNodes[idxR].isLeaf()) {
            int objId = mSortedObjectIds[idxR - N + 1];
            if (objId > queryId)
                ids.push_back(objId);
        }

        bool traverseL = (overlapL && !mAllNodes[idxL].isLeaf());
        bool traverseR = (overlapR && !mAllNodes[idxR].isLeaf());

        if (!traverseL && !traverseR) {
            idx = !stack.empty() ? stack.back() : EMPTY;
            if (!stack.empty())
                stack.pop_back();
        }
        else
        {
            idx = (traverseL) ? idxL : idxR;
            if (traverseL && traverseR)
                stack.push_back(idxR);
        }
    } while (idx != EMPTY);
}





