#include <UT/UT_DSOVersion.h> // Very Important!!! Include this!!!

#include <GAS_Test_CFL_SubStep.h>
#include <Base/SIM_Hina_Particles.h>
#include <PBF/SIM_Hina_Particles_PBF.h>
#include <PBF/GAS_Hina_Solver_PBF.h>

void initializeSIM(void *)
{
	// Completed Classes
    IMPLEMENT_DATAFACTORY(GAS_Test_CFL_SubStep)
    IMPLEMENT_DATAFACTORY(SIM_Hina_Particles)
    IMPLEMENT_DATAFACTORY(SIM_Hina_Particles_PBF)
    IMPLEMENT_DATAFACTORY(GAS_Hina_Solver_PBF)
}
