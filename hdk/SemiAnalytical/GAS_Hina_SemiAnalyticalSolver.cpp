//
// Created by LiYifan on 2024/3/7.
//

#include "GAS_Hina_SemiAnalyticalSolver.h"

GAS_HINA_SUBSOLVER_IMPLEMENT(
        SemiAnalyticalSolver,
        true,
        false,
        TARGET_PARTICLE_GEOMETRY(SIM_Hina_SemiAnalyticalBoundary)
)

void GAS_Hina_SemiAnalyticalSolver::_init()
{
    this->inited = false;
    this->SemiAnalyticalSolverPtr = nullptr;
}

void GAS_Hina_SemiAnalyticalSolver::_makeEqual(const GAS_Hina_SemiAnalyticalSolver *src)
{
    this->inited = src->inited;
    this->SemiAnalyticalSolverPtr = src->SemiAnalyticalSolverPtr;
}

bool GAS_Hina_SemiAnalyticalSolver::_solve(SIM_Engine &engine, SIM_Object *obj, SIM_Time time, SIM_Time timestep)
{
    SIM_Hina_SemiAnalyticalBoundary *SA_boundary = SIM_DATA_CAST(getGeometryCopy(obj, GAS_NAME_GEOMETRY), SIM_Hina_SemiAnalyticalBoundary);
    if (SA_boundary == nullptr)
        return true;

    if (!inited)
        init_data(SA_boundary, obj);

    if(SemiAnalyticalSolverPtr)
    {
        SemiAnalyticalSolverPtr->Solve(timestep);
    }

    return true;
}

void GAS_Hina_SemiAnalyticalSolver::init_data(SIM_Hina_SemiAnalyticalBoundary *boundary, SIM_Object *obj)
{
    SemiAnalyticalSolverPtr = std::make_shared<SemiAnalyticalSolver>();

    boundary->vertices = &SemiAnalyticalSolverPtr->vertices;
    boundary->faces = &SemiAnalyticalSolverPtr->faces;
    boundary->triangleAABBs = &SemiAnalyticalSolverPtr->triangleAABBs;

    SIM_Geometry *boundary_sop = SIM_DATA_GET(*obj, SIM_GEOMETRY_DATANAME, SIM_Geometry);
    if (!boundary_sop)
        return;
    SIM_GeometryAutoReadLock lock(boundary_sop);
    const GU_Detail *gdp = lock.getGdp();
    boundary->load_gdp(gdp);

    inited = true;
}

