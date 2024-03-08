#include <UT/UT_DSOVersion.h> // Very Important!!! Include this!!!

#include <GAS_Test_CFL_SubStep.h>
#include <Base/SIM_Hina_Particles.h>
#include <PBF/SIM_Hina_Particles_PBF.h>
#include <PBF/GAS_Hina_Solver_PBF.h>
#include <Rigid/SIM_Hina_RigidBody.h>
#include <Rigid/GAS_Hina_Solver_Rigid.h>
#include <SemiAnalytical/SIM_Hina_SemiAnalyticalBoundary.h>
#include <SemiAnalytical/GAS_Hina_SemiAnalyticalSolver.h>


void initializeSIM(void *)
{
	// Completed Classes
    IMPLEMENT_DATAFACTORY(GAS_Test_CFL_SubStep)
    IMPLEMENT_DATAFACTORY(SIM_Hina_Particles)
    IMPLEMENT_DATAFACTORY(SIM_Hina_Particles_PBF)
    IMPLEMENT_DATAFACTORY(GAS_Hina_Solver_PBF)
    IMPLEMENT_DATAFACTORY(SIM_Hina_RigidBody)
    IMPLEMENT_DATAFACTORY(GAS_Hina_Solver_Rigid)
    IMPLEMENT_DATAFACTORY(SIM_Hina_SemiAnalyticalBoundary)
    IMPLEMENT_DATAFACTORY(GAS_Hina_SemiAnalyticalSolver)
}
