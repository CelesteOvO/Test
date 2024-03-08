//
// Created by LiYifan on 2024/3/7.
//

#include "SIM_Hina_SemiAnalyticalBoundary.h"

SIM_HINA_GEOMETRY_IMPLEMENT(
        SemiAnalyticalBoundary,
        true,
        TARGET_PARTICLE_GEOMETRY(SIM_Hina_SemiAnalyticalBoundary)
)

void SIM_Hina_SemiAnalyticalBoundary::_init_SemiAnalyticalBoundary()
{
    this->offset2index.clear();
    this->index2offset.clear();
    this->vertices = nullptr;
    this->faces = nullptr;
    this->triangleAABBs = nullptr;
}

void SIM_Hina_SemiAnalyticalBoundary::_makeEqual_SemiAnalyticalBoundary(const SIM_Hina_SemiAnalyticalBoundary *src)
{
    this->offset2index = src->offset2index;
    this->index2offset = src->index2offset;
    this->vertices = src->vertices;
    this->faces = src->faces;
    this->triangleAABBs = src->triangleAABBs;
}

void SIM_Hina_SemiAnalyticalBoundary::_setup_gdp(GU_Detail *gdp) const
{
    // 创建三个新的primitive属性来储存每个三角形primitive的顶点索引
    GA_RWAttributeRef v1Attr = gdp->addIntTuple(GA_ATTRIB_PRIMITIVE, "v1", 1);
    GA_RWAttributeRef v2Attr = gdp->addIntTuple(GA_ATTRIB_PRIMITIVE, "v2", 1);
    GA_RWAttributeRef v3Attr = gdp->addIntTuple(GA_ATTRIB_PRIMITIVE, "v3", 1);

    GA_RWAttributeRef minAttr = gdp->addFloatTuple(GA_ATTRIB_PRIMITIVE, "bboxMin", 3);
    GA_RWAttributeRef maxAttr = gdp->addFloatTuple(GA_ATTRIB_PRIMITIVE, "bboxMax", 3);
}

void SIM_Hina_SemiAnalyticalBoundary::load_gdp(const GU_Detail *gdp_sop)
{
    if (vertices == nullptr || faces == nullptr || triangleAABBs == nullptr)
    {
        std::cout << "SIM_Hina_SemiAnalyticalBoundary::load() called with nullptr" << std::endl;
        return;
    }

    if(gdp_sop->getNumPoints() > 0) // Add
    {
        size_t pointIndex = 0;
        GA_Offset pt_off;
        GA_FOR_ALL_PTOFF(gdp_sop, pt_off)
        {
            // 获取该点的偏移并存入 map 中
            offset2index[pt_off] = pointIndex;
            index2offset[pointIndex] = pt_off;
            // 获取该点的位置并存入 vec 中
            UT_Vector3 pos = gdp_sop->getPos3(pt_off);
            vertices->push_back(pos);
            // 更新索引
            ++pointIndex;
        }

        size_t triangleIndex = 0;
        const GEO_Primitive *prim;
        GA_FOR_ALL_PRIMITIVES(gdp_sop, prim)
        {
            const auto *poly = dynamic_cast<const GEO_PrimPoly *>(prim);
            if (!poly)
            {
                std::cout << "ERROR ON CONVERT PRIM TO POLY" << std::endl;
                return;
            }

            // Triangulate Polygon
            std::vector<size_t> polyIndices;
            for (int vi = 0; vi < poly->getVertexCount(); ++vi)
                polyIndices.push_back(poly->getPointIndex(vi));
            for (size_t i = 1; i < polyIndices.size() - 1; ++i)
            {
                std::vector<size_t> triangulationIndices = {polyIndices[0], polyIndices[i + 1], polyIndices[i]}; // notice the normal
                faces->push_back(triangulationIndices);

                // 更新三角形的AABB
                UT_Vector3 min = gdp_sop->getPos3(polyIndices[0]);
                UT_Vector3 max = gdp_sop->getPos3(polyIndices[0]);
                for (size_t j = 1; j < 3; ++j)
                {
                    UT_Vector3 pos = gdp_sop->getPos3(polyIndices[j]);
                    for (size_t k = 0; k < 3; ++k)
                    {
                        min[k] = std::min(min[k], pos[k]);
                        max[k] = std::max(max[k], pos[k]);
                    }
                }
                triangleAABBs->push_back(AlignedBox(min, max));

                // 更新索引
                ++triangleIndex;
            }
        }
    }
}

void SIM_Hina_SemiAnalyticalBoundary::commit()
{
    if (vertices == nullptr || faces == nullptr || triangleAABBs == nullptr)
    {
        std::cout << "SIM_Hina_SemiAnalyticalBoundary::load() called with nullptr" << std::endl;
        return;
    }

    size_t size = vertices->size();
    if (size == 0)
        return;

    SIM_GeometryAutoWriteLock lock(this);
    GU_Detail &gdp = lock.getGdp();
    GA_RWHandleI v1_handle(gdp.findPrimitiveAttribute("v1"));
    GA_RWHandleI v2_handle(gdp.findPrimitiveAttribute("v2"));
    GA_RWHandleI v3_handle(gdp.findPrimitiveAttribute("v3"));
    GA_RWHandleV3 min_handle(gdp.findPrimitiveAttribute("bboxMin"));
    GA_RWHandleV3 max_handle(gdp.findPrimitiveAttribute("bboxMax"));

    for(auto pos : *vertices)
    {
        GA_Offset pt_off = gdp.appendPoint();
        gdp.setPos3(pt_off,pos);
    }

    for(size_t i = 0; i < faces->size(); ++i)
    {
        GA_Primitive *prim = gdp.appendPrimitive(GEO_PRIMPOLY);

        // 获取新面的偏移量
        GA_Offset prim_off = prim->getMapOffset();

        // 设置v1, v2, v3, bboxMin, bboxMax的属性
        v1_handle.set(prim_off, (*faces)[i][0]);
        v2_handle.set(prim_off, (*faces)[i][1]);
        v3_handle.set(prim_off, (*faces)[i][2]);

        auto aabb = (*triangleAABBs)[i];
        min_handle.set(prim_off, aabb.v0);
        max_handle.set(prim_off, aabb.v1);
    }
}

auto FetchAllSemiAnalyticalBoundaries(SIM_Object *fluid_obj) -> std::vector<SIM_Hina_SemiAnalyticalBoundary *>
{
    std::vector<SIM_Hina_SemiAnalyticalBoundary *> res;
    SIM_ObjectArray affectors;
    fluid_obj->getAffectors(affectors, "SIM_RelationshipCollide");
    exint num_affectors = affectors.entries();
    for (int i = 0; i < num_affectors; ++i)
    {
        SIM_Object *obj_collider = affectors(i);
//        if (obj_collider->getName().equal(fluid_obj->getName()))
//            continue;
        SIM_Hina_SemiAnalyticalBoundary *boundary_semi_analytical = SIM_DATA_GET(*obj_collider, SIM_Hina_SemiAnalyticalBoundary::DATANAME, SIM_Hina_SemiAnalyticalBoundary);
        if (boundary_semi_analytical)
            res.emplace_back(boundary_semi_analytical);
    }
    return res;
}

void InitAllSemiAnalyticalBoundaries(SIM_Object *fluid_obj)
{
    SIM_ObjectArray affectors;
    fluid_obj->getAffectors(affectors, "SIM_RelationshipCollide");
    exint num_affectors = affectors.entries();
    for (int i = 0; i < num_affectors; ++i)
    {
        SIM_Object *obj_collider = affectors(i);
        /*if (obj_collider->getName().equal(fluid_obj->getName()))
            continue;*/
        SIM_Hina_SemiAnalyticalBoundary *boundary_semi_analytical = SIM_DATA_GET(*obj_collider, SIM_Hina_SemiAnalyticalBoundary::DATANAME, SIM_Hina_SemiAnalyticalBoundary);
        if (boundary_semi_analytical)
        {
            SIM_Geometry *boundary_sop = SIM_DATA_GET(*obj_collider, SIM_GEOMETRY_DATANAME, SIM_Geometry);
            if (!boundary_sop)
                return;
            SIM_GeometryAutoReadLock lock(boundary_sop);
            const GU_Detail *gdp = lock.getGdp();
            boundary_semi_analytical->load_gdp(gdp);
        }
    }
}

