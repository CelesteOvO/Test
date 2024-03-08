#include "GAS_Test_CFL_SubStep.h"
#include "SemiAnalytical/SIM_Hina_SemiAnalyticalBoundary.h"

#include <SIM/SIM_ObjectArray.h>
#include <SIM/SIM_DopDescription.h>
#include <PRM/PRM_Template.h>
#include <PRM/PRM_Default.h>
#include <Base/SIM_Hina_Particles.h>

void GAS_Test_CFL_SubStep::initializeSubclass() {
    SIM_Data::initializeSubclass();
    this->boundary_inited = false;
}

void GAS_Test_CFL_SubStep::makeEqualSubclass(const SIM_Data *source) {
    SIM_Data::makeEqualSubclass(source);
    const GAS_Test_CFL_SubStep *src = SIM_DATA_CASTCONST(source, GAS_Test_CFL_SubStep);

    this->boundary_inited = src->boundary_inited;
}


const SIM_DopDescription *GAS_Test_CFL_SubStep::getDopDescription()
{
	static PRM_Name MAX_SUBSTEP("MAX_SUBSTEP", "MAX_SUBSTEP");
	static PRM_Default DefaultMAX_SUBSTEP{2};

	static std::array<PRM_Template, 2> PRMS{
			PRM_Template(PRM_INT, 1, &MAX_SUBSTEP, &DefaultMAX_SUBSTEP),
			PRM_Template()
	};

	static SIM_DopDescription DESC(true,
								   "Test_CFL_SubStep",
								   "Test_CFL_SubStep",
								   "Test_CFL_SubStep",
								   classname(),
								   PRMS.data());
	DESC.setDefaultUniqueDataName(true);
	return &DESC;
}

SIM_Solver::SIM_Result GAS_Test_CFL_SubStep::solveObjectsSubclass(SIM_Engine &engine, SIM_ObjectArray &objects, SIM_ObjectArray &newobjects, SIM_ObjectArray &feedbacktoobjects, const SIM_Time &timestep)
{
	// TODO: Implement CFL substep
	float mas_step = getMAX_SUBSTEP();
	fpreal sub_time = timestep / mas_step;
	for (int i = 0; i < mas_step; ++i)
	{
		SIM_Solver::SIM_Result res = GAS_SubStep::solveObjectsSubclass(engine, objects, newobjects, feedbacktoobjects, sub_time);
		if (res != SIM_Solver::SIM_SOLVER_SUCCESS)
			return res;
	}


	for (int j = 0; j < objects.size(); ++j)
	{
		SIM_Object *obj = objects(j);
		SIM_Hina_Particles *particles = SIM_DATA_GET(*obj, SIM_Hina_Particles::DATANAME, SIM_Hina_Particles);
		if (particles && particles->x != nullptr)
			particles->commit();

        if(!boundary_inited)
        {
            std::vector<SIM_Hina_SemiAnalyticalBoundary *> semi_analytical_boundaries = FetchAllSemiAnalyticalBoundaries(obj);
            for (auto &semi_analytical_boundary: semi_analytical_boundaries)
                if (semi_analytical_boundary && semi_analytical_boundary->vertices != nullptr)
                    semi_analytical_boundary->commit();
            boundary_inited = true;
        }
	}
	return SIM_Solver::SIM_SOLVER_SUCCESS;
}



