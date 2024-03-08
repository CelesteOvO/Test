//
// Created by LiYifan on 2024/3/5.
//

#include "GAS_Hina_Solver_PBF.h"

GAS_HINA_SUBSOLVER_IMPLEMENT(
        Solver_PBF,
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
        TARGET_PARTICLE_GEOMETRY(SIM_Hina_Particles_PBF)
)

void GAS_Hina_Solver_PBF::_init()
{
    this->PBFSolverPtr = nullptr;
    this->inited = false;
    this->emitted = false;
}

void GAS_Hina_Solver_PBF::_makeEqual(const GAS_Hina_Solver_PBF *src)
{
    this->PBFSolverPtr = src->PBFSolverPtr;
    this->inited = src->inited;
    this->emitted = src->emitted;
}

bool GAS_Hina_Solver_PBF::_solve(SIM_Engine &engine, SIM_Object *obj, SIM_Time time, SIM_Time timestep)
{
    SIM_Hina_Particles_PBF *PBF_particles = SIM_DATA_CAST(getGeometryCopy(obj, GAS_NAME_GEOMETRY), SIM_Hina_Particles_PBF);
    CHECK_NULL_RETURN_BOOL(PBF_particles)

    if (!inited)
        init_data(PBF_particles, obj);

    if (!emitted)
        emit_data(PBF_particles);

    //std::cout << "Particle count: " << PBF_particles->x->size() << std::endl;

    if (PBFSolverPtr)
    {
        //std::cout << PBFSolverPtr->Fluid->size << std::endl;
        PBFSolverPtr->Solve(timestep);
    }
    
    return true;
}

void GAS_Hina_Solver_PBF::init_data(SIM_Hina_Particles_PBF *PBF_particles, SIM_Object *obj)
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

    inited = true;
}

void GAS_Hina_Solver_PBF::emit_data(SIM_Hina_Particles_PBF *PBF_particles)
{
    size_t max_p = getMaxNumOfParticles();
    bool one_shot = getIsOneShot();
    real spacing = getTargetSpacing();
    Vector start = getEmitStart();
    Vector end = getEmitEnd();

    FluidEmitter::UseFluidBlock(PBF_particles->x, start, end, spacing);

    emitted = one_shot;
}
