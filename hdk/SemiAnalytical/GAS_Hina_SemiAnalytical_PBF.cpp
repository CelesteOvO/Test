//
// Created by LiYifan on 2024/3/12.
//

#include "GAS_Hina_SemiAnalytical_PBF.h"

GAS_HINA_SUBSOLVER_IMPLEMENT(
        SemiAnalytical_PBF,
        true,
        false,
        HINA_FLOAT_VECTOR_PARAMETER(FluidDomain, 3, 1., 1., 1.) \
        HINA_BOOL_PARAMETER(TopOpen, true) \
        HINA_FLOAT_PARAMETER(TargetSpacing, .02) \
        HINA_FLOAT_PARAMETER(KernelRadius, .04) \
        HINA_FLOAT_PARAMETER(TargetDensity, 1000.) \
        HINA_FLOAT_VECTOR_PARAMETER(Gravity, 3, 0, -9.8, 0) \
        HINA_INT_PARAMETER(MaxNumOfParticles, 100000) \
        HINA_BOOL_PARAMETER(IsOneShot, true) \
        HINA_FLOAT_VECTOR_PARAMETER(EmitStart, 3, -.3, -.3, -.3) \
        HINA_FLOAT_VECTOR_PARAMETER(EmitEnd, 3, .3, .3, .3) \
        static std::array<PRM_Name, 3> Kernels = {\
            PRM_Name("0", "Poly6"), \
            PRM_Name("1", "Spiky"), \
            PRM_Name(nullptr),}; \
        static PRM_Name KernelName("Kernel", "Kernel"); \
        static PRM_Default KernelNameDefault(1, "Spiky"); \
        static PRM_ChoiceList CLKernel(PRM_CHOICELIST_SINGLE, Kernels.data()); \
        PRMS.emplace_back(PRM_ORD, 1, &KernelName, &KernelNameDefault, &CLKernel); \
        static PRM_Name theFluidGeometryName("FluidGeometry", "FluidGeometry"); \
        static PRM_Default    theFluidGeometryNameDefault(0, SIM_Hina_Particles_PBF::DATANAME);\
        PRMS.emplace_back(PRM_STRING, 1, &theFluidGeometryName, &theFluidGeometryNameDefault);\
        static PRM_Name theBoundaryGeometryName("BoundaryGeometry", "BoundaryGeometry"); \
        static PRM_Default    theBoundaryGeometryNameDefault(0, SIM_Hina_SemiAnalyticalBoundary::DATANAME);\
        PRMS.emplace_back(PRM_STRING, 1, &theBoundaryGeometryName, &theBoundaryGeometryNameDefault);\
)

void GAS_Hina_SemiAnalytical_PBF::_init()
{
    this->PBFSolverPtr = nullptr;
    this->SemiAnalyticalSolverPtr = nullptr;
    this->fluid_inited = false;
    this->boundary_inited = false;
    this->fluid_emitted = false;
}

void GAS_Hina_SemiAnalytical_PBF::_makeEqual(const GAS_Hina_SemiAnalytical_PBF *src)
{
    this->PBFSolverPtr = src->PBFSolverPtr;
    this->SemiAnalyticalSolverPtr = src->SemiAnalyticalSolverPtr;
    this->fluid_inited = src->fluid_inited;
    this->boundary_inited = src->boundary_inited;
    this->fluid_emitted = src->fluid_emitted;
}

bool GAS_Hina_SemiAnalytical_PBF::_solve(SIM_Engine &engine, SIM_Object *obj, SIM_Time time, SIM_Time timestep)
{
    /// 同一个object，好取数，好理解，但是连线很抽象而且也不好debug（？
    // Fluid Step
    SIM_Hina_Particles_PBF *PBF_particles = SIM_DATA_CAST(getGeometryCopy(obj, "FluidGeometry"), SIM_Hina_Particles_PBF);

    if(!fluid_inited)
        init_fluid_data(PBF_particles, obj);

    if(!fluid_emitted)
        emit_fluid_data(PBF_particles);

    if(PBFSolverPtr)
        PBFSolverPtr->Solve(timestep);

    // Boundary Step
    SIM_Hina_SemiAnalyticalBoundary *boundary = SIM_DATA_CAST(getGeometryCopy(obj, "BoundaryGeometry"), SIM_Hina_SemiAnalyticalBoundary);

    if(!boundary_inited)
        init_boundary_data(boundary, obj);

    if(SemiAnalyticalSolverPtr)
    {
        /// PBFSolverPtr->Fluid->pointAABB 这个值的更新是在PBFSolverPtr->Solve(timestep)里面的
        /// SemiAnalyticalSolverPtr->triangleAABBs 这个值的初始化是在init_boundary_data函数中的load_gdp(gdp)里面的
        /// 由于先考虑这个不动的情况，所以先不考虑这个值的更新
        SemiAnalyticalSolverPtr->Solve(timestep,PBFSolverPtr->Fluid->pointAABB, SemiAnalyticalSolverPtr->triangleAABBs);
    }

    return true;

    /// 两个object，但是不好取数
    // SIM_Hina_Particles_PBF *PBF_particles = SIM_DATA_CAST(getGeometryCopy(obj, "FluidGeometry"), SIM_Hina_Particles_PBF);
    //
    //    if(PBF_particles != nullptr) // Fluid Step
    //    {
    //        if(!fluid_inited)
    //            init_fluid_data(PBF_particles, obj);
    //
    //        if(!fluid_emitted)
    //            emit_fluid_data(PBF_particles);
    //
    //        if(PBFSolverPtr)
    //            PBFSolverPtr->Solve(timestep);
    //    }else{
    //        // Boundary Step
    //        SIM_Hina_SemiAnalyticalBoundary *boundary = SIM_DATA_CAST(getGeometryCopy(obj, "BoundaryGeometry"), SIM_Hina_SemiAnalyticalBoundary);
    //        if(boundary == nullptr)
    //            return false; // 啥也不是
    //
    //        if(!boundary_inited)
    //            init_boundary_data(boundary, obj);
    //
    //        if(SemiAnalyticalSolverPtr)
    //        {
    //            SemiAnalyticalSolverPtr->Solve(timestep);
    //        }
    //    }
}

void GAS_Hina_SemiAnalytical_PBF::init_fluid_data(SIM_Hina_Particles_PBF *PBF_particles, SIM_Object *obj)
{
    PBFSolverPtr = std::make_shared<PbfSolver>(static_cast<real>(getKernelRadius()), getFluidDomainF());
    PBFSolverPtr->FLUID_REST_DENSITY = getTargetDensity();
    PBFSolverPtr->FLUID_PARTICLE_RADIUS = getTargetSpacing() / 2.;
    PBFSolverPtr->GRAVITY = getGravityF();
    PBFSolverPtr->TOP_OPEN = getTopOpen();

    PBF_particles->x = &PBFSolverPtr->Fluid->x;
    PBF_particles->v = &PBFSolverPtr->Fluid->v;
    PBF_particles->m = &PBFSolverPtr->Fluid->m;
    PBF_particles->a = &PBFSolverPtr->Fluid->a;
    PBF_particles->V = &PBFSolverPtr->Fluid->V;
    PBF_particles->rho = &PBFSolverPtr->Fluid->rho;
    PBF_particles->nt = &PBFSolverPtr->Fluid->neighbor_this;
    PBF_particles->no = &PBFSolverPtr->Fluid->neighbor_others;

    PBF_particles->pred_x = &PBFSolverPtr->Fluid->pred_x;
    PBF_particles->lambda = &PBFSolverPtr->Fluid->lambda;
    PBF_particles->delta_p = &PBFSolverPtr->Fluid->delta_p;
    PBF_particles->a_ext = &PBFSolverPtr->Fluid->a_ext;
    PBF_particles->pointAABB = &PBFSolverPtr->Fluid->pointAABB;

    fluid_inited = true;
}

void GAS_Hina_SemiAnalytical_PBF::emit_fluid_data(SIM_Hina_Particles_PBF *PBF_particles)
{
    size_t max_p = getMaxNumOfParticles();
    bool one_shot = getIsOneShot();
    real spacing = getTargetSpacing();
    Vector start = getEmitStart();
    Vector end = getEmitEnd();

    FluidEmitter::UseFluidBlock(PBF_particles->x, start, end, spacing);

    fluid_emitted = one_shot;
}

void GAS_Hina_SemiAnalytical_PBF::init_boundary_data(SIM_Hina_SemiAnalyticalBoundary *boundary, SIM_Object *obj)
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
    boundary->load_gdp(gdp); // 先单项考虑他静止，所以只在最初的时候load一次

    boundary_inited = true;
}