//
// Created by LiYifan on 2024/3/8.
//

#include "LinearBVH.h"
#include <thrust/extrema.h>
#include <thrust/sort.h>
#include <bitset>
#include <stack>

std::string toBinary(uint64_t value) {
    return std::bitset<64>(value).to_string();
}


void LinearBVH::construct(std::vector<AlignedBox> &aabb) {
    size_t num = aabb.size();

    if (mCenters.size() != num){
        mCenters.resize(num);
        mMortonCodes.resize(num);
        mSortedObjectIds.resize(num);
        mFlags.resize(num);

        mSortedAABBs.resize(2 * num - 1);
        mAllNodes.resize(2 * num - 1);
        /// 这个'2 * num - 1'大小的设置是因为你在构造一个二叉树结构。
        /// 对于一个完全二叉树来说，如果你有n个叶子节点（这里是num个对象）那么总的节点数（包括叶子节点和内部节点）就会是'2 * num - 1'。
    }

    /*/// Test
    std::cout << "AABB size: " << aabb.size() << std::endl;
    std::cout << "Center size: " << mCenters.size() << std::endl;
    std::cout << "Morton size: " << mMortonCodes.size() << std::endl;
    std::cout << "SortedObjectIds size: " << mSortedObjectIds.size() << std::endl;
    std::cout << "Flags size: " << mFlags.size() << std::endl;
    std::cout << "SortedAABBs size: " << mSortedAABBs.size() << std::endl;
    std::cout << "AllNodes size: " << mAllNodes.size() << std::endl;*/

    LBVH_CalculateCenter(mCenters, aabb);

    // 所有中心点的最小坐标v_min和最大坐标v_max
    auto minmax = thrust::minmax_element(mCenters.begin(), mCenters.end());
    UT_Vector3 v_min = *minmax.first;
    UT_Vector3 v_max = *minmax.second;
    // 所有物体中心点在空间上的最大跨度L,检查L是否接近于零，即坐标点是否过于接近，如果太接近则将L设为1以避免除以接近零的数（这通常用于避免精度问题）。
    fpreal L = std::max(v_max[0] - v_min[0], std::max(v_max[1] - v_min[1], v_max[2] - v_min[2]));
    L = L < (std::numeric_limits<fpreal>::epsilon)() ? fpreal(1) : L;
    // 如果所有的物体中心点都非常接近，那么L的值可能会非常接近零，如果直接用这样的L作为除数，可能会导致结果出现异常（比如无限大或非常大的值）
    // 所有中心点的范围内选择一个参考点
    UT_Vector3 origin = fpreal(0.5) * (v_min + v_max) - fpreal(0.5) * L;

    /*/// Test
    std::cout << "v_min: " << v_min << std::endl;
    std::cout << "v_max: " << v_max << std::endl;
    std::cout << "L: " << L << std::endl;
    std::cout << "origin: " << origin << std::endl;*/

    LBVH_CalculateMortonCodes(mMortonCodes, mSortedObjectIds, mCenters, origin, L);
    thrust::sort_by_key(
            mMortonCodes.begin(),
            mMortonCodes.end(),
            mSortedObjectIds.begin());

    /*/// Test
    for(size_t i = 0; i < mMortonCodes.size(); i++){
        std::cout << "Morton: " << toBinary(mMortonCodes[i]) << std::endl;
        std::cout << "ObjectId: " << mSortedObjectIds[i] << std::endl;
    }*/

    LBVH_InitialAllNodes(mAllNodes);
    LBVH_ConstructBinaryRadixTree(mAllNodes, mSortedAABBs, aabb, mMortonCodes, mSortedObjectIds);

    /*/// Test
    std::cout << "AllNodes size: " << mAllNodes.size() << std::endl;
    for(auto & bvhNode : mAllNodes){
        std::cout << "Parent: " << bvhNode.parent << std::endl;
        std::cout << "Left: " << bvhNode.left << std::endl;
        std::cout << "Right: " << bvhNode.right << std::endl;
    }*/

    /// Test
    /*for(auto & mSortedAABB : mSortedAABBs){
        std::cout << "AABB1: " << mSortedAABB.v0 << "," << mSortedAABB.v1 << std::endl;
    }*/

    std::fill(mFlags.begin(), mFlags.end(), 0);
    LBVH_CalculateBoundingBox(mSortedAABBs, mAllNodes, mFlags);

    /*for(auto & mSortedAABB : mSortedAABBs){
        std::cout << "AABB2: " << mSortedAABB.v0 << "," << mSortedAABB.v1 << std::endl;
    }*/

    /*std::vector<AlignedBox> TestAABBs;
    AlignedBox A1(UT_Vector3(-1,-1,-1), UT_Vector3(-0.5,-0.5,-0.5));
    AlignedBox A2(UT_Vector3(-1,-1,0), UT_Vector3(-0.5,-0.5,0.5));
    AlignedBox A3(UT_Vector3(0,0,0), UT_Vector3(0.5,0.5,0.5));
    AlignedBox A4(UT_Vector3(-1,0,-1), UT_Vector3(-0.5,0.5,-0.5));
    TestAABBs.push_back(A1);
    TestAABBs.push_back(A2);
    TestAABBs.push_back(A3);
    TestAABBs.push_back(A4);

    for(auto & mSortedAABB : TestAABBs){
        std::cout << "AABB1: " << mSortedAABB.v0 << "," << mSortedAABB.v1 << std::endl;
    }

    std::fill(mFlags.begin(), mFlags.end(), 0);
    LBVH_CalculateBoundingBox(TestAABBs, mAllNodes, mFlags);

    for(auto & mSortedAABB : TestAABBs){
        std::cout << "AABB2: " << mSortedAABB.v0 << "," << mSortedAABB.v1 << std::endl;
    }*/
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
        /*/// Test
        if(i < 10)
        {
            std::cout << "AABB: " << aabb[i].v0 << "," << aabb[i].v1 << std::endl;
            std::cout << "Center: " << center[i] << std::endl;
        }*/
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
        /// 保存原始顺序；然后在计算Morton码后，会同时得到一个按Morton码排序的objectId列表，这样就可以知道每个对象在按照Morton码排序后的新位置

        /*/// Test
        if(i < 10)
        {
            std::cout << "Center: " << center[i] << std::endl;
            std::cout << "Scaled: " << scaled << std::endl;
            std::cout << "Morton: " << m64 << std::endl;
            std::cout << "ObjectId: " << i << std::endl;
        }*/
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

    /*/// Test
    std::cout << "Node size: " << bvhNodes.size() << std::endl;
    for(auto & bvhNode : bvhNodes){
        std::cout << "Parent: " << bvhNode.parent << std::endl;
        std::cout << "Left: " << bvhNode.left << std::endl;
        std::cout << "Right: " << bvhNode.right << std::endl;
    }*/
}

void LinearBVH::LBVH_ConstructBinaryRadixTree(std::vector<BVHNode> &bvhNodes, std::vector<AlignedBox> &sortedAABBs,
                                              std::vector<AlignedBox> &aabbs, std::vector<size_t> &mortonCodes,
                                              std::vector<size_t> &sortedObjectIds) {
    size_t N = sortedObjectIds.size(); // 叶子节点（或者物体）的个数

    for (int i = 0; i < N; ++i)
    {
        sortedAABBs[i + N - 1] = aabbs[sortedObjectIds[i]];
        // 对于每个叶子节点，将其AABB存到二叉树的对应位置。因为在二叉树的数组表示中，叶子节点是排在后半部分的。

        if (i >= N - 1) // 如果这已经是最后一个叶子节点，则不再进行下面的处理
            continue;

        auto delta = [&](int _i, int _j) -> int { // 返回的是两个物体的Morton编码的最长公共前缀的长度
            if (_j < 0 || _j >= N)
                return -1;
            uint64_t value = mortonCodes[_i] ^ mortonCodes[_j];
            std::bitset<64> bs(value);
            return bs.size() - bs.to_ullong();
        };

        int d = delta(i, i + 1) - delta(i, i - 1) > 0 ? 1 : -1;
        // 确定物体i是更靠近自己前一个物体还是后一个物体
        // 大于0说明物体i与它后一个物体的Morton编码最长公共前缀长度比物体i与它前一个物体的Morton编码最长公共前缀长度大，物体i与其后一个物体i+1在Morton编码上的相似性更高，也就表示他们在三维空间中的接近程度更高。 物体i应离物体i+1更近。


        int delta_min = delta(i, i - d);
        // 是为了计算出物体i和它更远的那个物体（如果d为1，那么更远的那个物体就是i-1；如果d为-1，更远的那个物体就是i+1）的Morton编码的最长公共前缀的长度，也就是delta_min。

        // 进行二分查找，找出最大的长度L（满足delta(i, i + L * d) > delta_min），并在L的范围内进行二分查找，找出最小的长度j（满足delta(i, i + j * d) == delta_node）
        int len_max = 2;
        while (delta(i, i + len_max * d) > delta_min) {
            // 当delta(i, i + len_max * d) > delta_min成立的时候，说明节点i与节点i+len_max * d的最长公共前缀超过了节点i与它最远的邻居的当前最长公共前缀。
            // 换句话说，i和i+len_max * d更相似，也就是更临近。在这种情况下，将len_max翻倍，扩大搜索范围
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
        int gamma = i + s * d + std::min(d, (int)0);

        int left_idx = std::min(i, j) == gamma ? gamma + N - 1 : gamma;
        int right_idx = std::max(i, j) == gamma + 1 ? gamma + N : gamma + 1;

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
            /*auto* flags_ptr = reinterpret_cast<std::atomic<size_t>*>(flags.data()); // 获取指向flags数据的原子指针
            std::atomic<size_t>& flag = flags_ptr[idx]; // 获取位置idx的原子引用
            size_t expected = 0;
            size_t desired = 1;
            bool success = flag.compare_exchange_strong(expected, desired); // 尝试设置新值
            int old = success ? 0 : flag.load(); // 获取旧值
            if (old == 0)
            {
                return;
            }*/
            unsigned old = flags[idx];
            if (old == 0) {
                flags[idx] = 1;
                return;
            }
            assert(old == 1);
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
    std::stack<int> myStack;

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
            idx = !myStack.empty() ? myStack.top() : -1; // pop
            myStack.pop();
        }
        else
        {
            idx = traverseL ? idxL : idxR;
            if (traverseL && traverseR)
                myStack.push(idxR); // push
        }
    } while (idx != -1);

    return ret;
}

void LinearBVH::requestIntersectionIds(std::vector<size_t> &ids, const AlignedBox &queryBox, const int queryId)
{
    std::deque<int> myStack;

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
            idx = !myStack.empty() ? myStack.back() : EMPTY;
            if (!myStack.empty())
                myStack.pop_back();
        }
        else
        {
            idx = (traverseL) ? idxL : idxR;
            if (traverseL && traverseR)
                myStack.push_back(idxR);
        }
    } while (idx != EMPTY);
}





