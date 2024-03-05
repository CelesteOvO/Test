//
// Created by LiYifan on 2024/3/4.
//

#include "PBF.h"

#include <numeric>
#include <utility>
#include <execution>

PbfSolver::PbfSolver(real _r, Vector _b) : NeighborBuilder(_r),MaxBound(_b / 2.)
{
    PolyKernel::set_radius(_r);
    constexpr size_t n = 50000;
    Fluid = std::make_shared<PbfFluidCPU>();

    Fluid->x.reserve(n);
    Fluid->v.reserve(n);
    Fluid->a.reserve(n);
    Fluid->m.reserve(n);
    Fluid->V.reserve(n);
    Fluid->rho.reserve(n);
    Fluid->neighbor_this.reserve(n);
    Fluid->a.reserve(n);

    Fluid->pred_x.reserve(n);
    Fluid->lambda.reserve(n);
    Fluid->delta_p.reserve(n);
    Fluid->a_ext.reserve(n);
}

void PbfSolver::Solve(real dt) {
    _resize();
    predict_position(dt);
    solve_density_constraints();
    update_position_and_velocity(dt);
    enforce_boundary();
}

void PbfSolver::build_neighbors()
{
    NeighborBuilder.update_set(0);
    NeighborBuilder.build();

    _for_each_fluid_particle_pred([&](size_t i, Vector x_i)
     {
         Fluid->neighbor_this[i] = NeighborBuilder.n_neighbors(0, 0, i);
     });
}

void PbfSolver::compute_density() {
    _for_each_fluid_particle_pred([&](size_t i, Vector pred_x_i)
    {
        real rho_i = Fluid->V[i] * PolyKernel::W_zero();
        _for_each_neighbor_fluid(i, [&](size_t j, Vector x_j)
        {
            rho_i += Fluid->V[j] * PolyKernel::W(pred_x_i - x_j);
        });
        Fluid->rho[i] = rho_i * FLUID_REST_DENSITY;
    });
}

void PbfSolver::predict_position(real dt) {
    compute_external_a();
    _for_each_fluid_particle([&](size_t i, Vector x_i)
    {
        Fluid->v[i] += dt * Fluid->a_ext[i];
        Fluid->pred_x[i] = Fluid->x[i] + dt * Fluid->v[i];
    });
}

void PbfSolver::solve_density_constraints() {
    int iter = 0;
    while(iter < MAX_ITERATIONS)
    {
     build_neighbors();
/*      compute_density();
        compute_lambda();
        compute_delta_p();
        update_predict_position();*/
        iter++;
    }
}

void PbfSolver::compute_lambda() {
    _for_each_fluid_particle_pred([&](size_t i, Vector pred_x_i)
    {
        real C_i = Fluid->rho[i] / FLUID_REST_DENSITY - 1;
        if(C_i > 0)
        {
            real sum_grad_C_i_squared = 0;
            Vector grad_C_i = Vector(0, 0, 0);
            _for_each_neighbor_fluid(i, [&](size_t j, Vector x_j)
            {
                Vector grad_C_ij = - Fluid->V[i]  * PolyKernel::gradW(pred_x_i - x_j);
                sum_grad_C_i_squared += grad_C_ij.length2();
                grad_C_i -= grad_C_ij;
            });
            sum_grad_C_i_squared += grad_C_i.length2();
            Fluid->lambda[i] = - C_i / (sum_grad_C_i_squared + EPSILON);
        }else{
            Fluid->lambda[i] = 0;
        }
    });
}

void PbfSolver::compute_delta_p() {
    _for_each_fluid_particle_pred([&](size_t i, Vector pred_x_i)
    {
        auto k_corr = Fluid->m[i] * 1.0e-04;
        auto n_corr = 4.0;
        auto q_corr = 0.1;

        Vector delta_p_i = Vector(0, 0, 0);
        _for_each_neighbor_fluid(i, [&](size_t j, Vector x_j)
        {
            const auto w_corr = PolyKernel::W(q_corr * FLUID_PARTICLE_RADIUS);
            const auto ratio = PolyKernel::W((pred_x_i - x_j).length()) / w_corr;
            const auto s_corr = -k_corr * pow(ratio, n_corr);

            Vector grad_C_ij = - Fluid->V[i]  * PolyKernel::gradW(pred_x_i - x_j);
            delta_p_i += (Fluid->lambda[i] + Fluid->lambda[j] + s_corr) * grad_C_ij;
        });
        Fluid->delta_p[i] = delta_p_i;
    });
}

void PbfSolver::update_predict_position() {
    _for_each_fluid_particle_pred([&](size_t i, Vector pred_x_i)
    {
        Fluid->pred_x[i] += Fluid->delta_p[i];
    });
}

void PbfSolver::update_position_and_velocity(real dt) {
    _for_each_fluid_particle([&](size_t i, Vector x_i)
    {
        Fluid->v[i] = (Fluid->pred_x[i] - Fluid->x[i]) / dt;
        Fluid->x[i] = Fluid->pred_x[i];
    });
}

void PbfSolver::enforce_boundary() {
    _for_each_fluid_particle(
            [&](size_t i, Vector x_i)
            {
                Vector collision_normal{0, 0, 0};
                if (x_i.x() > MaxBound.x())
                {
                    Fluid->x[i].x() = MaxBound.x();
                    collision_normal.x() += 1;
                }
                if (x_i.x() < -MaxBound.x())
                {
                    Fluid->x[i].x() = -MaxBound.x();
                    collision_normal.x() -= 1;
                }
                if (!TOP_OPEN)
                {
                    if (x_i.y() > MaxBound.y())
                    {
                        Fluid->x[i].y() = MaxBound.y();
                        collision_normal.y() += 1;
                    }
                }
                if (x_i.y() < -MaxBound.y())
                {
                    Fluid->x[i].y() = -MaxBound.y();
                    collision_normal.y() -= 1;
                }
                if (x_i.z() > MaxBound.z())
                {
                    Fluid->x[i].z() = MaxBound.z();
                    collision_normal.z() += 1;
                }
                if (x_i.z() < -MaxBound.z())
                {
                    Fluid->x[i].z() = -MaxBound.z();
                    collision_normal.z() -= 1;
                }
                collision_normal.normalize();
                constexpr real c_f = static_cast<real>(0.5f);
                Fluid->v[i] -= (1. + c_f) * Fluid->v[i].dot(collision_normal) * collision_normal;
            });
}

void PbfSolver::compute_external_a() {

    _for_each_fluid_particle([&](size_t i, Vector x_i)
    {
        Fluid->a_ext[i] = GRAVITY;
    });
}

void PbfSolver::_resize()
{
    size_t pre_size = Fluid->size;
    if (Fluid->size != Fluid->x.size())
    {
        Fluid->size = Fluid->x.size();
        size_t n = Fluid->size;
        Fluid->x.resize(n);
        Fluid->v.resize(n);
        Fluid->a.resize(n);
        Fluid->m.resize(n);
        Fluid->V.resize(n);
        Fluid->rho.resize(n);
        Fluid->neighbor_this.resize(n);
        Fluid->a.resize(n);

        Fluid->pred_x.resize(n);
        Fluid->lambda.resize(n);
        Fluid->delta_p.resize(n);
        Fluid->a_ext.resize(n);
    }

    if (pre_size == 0)
    {
        real d = static_cast<real>(2.f) * FLUID_PARTICLE_RADIUS;
        std::fill(Fluid->V.begin(), Fluid->V.end(), static_cast<real>(.8f) * d * d * d);
        std::transform(Fluid->V.begin(), Fluid->V.end(), Fluid->m.begin(), [&](real V) { return V * FLUID_REST_DENSITY; });
        std::transform(Fluid->x.begin(), Fluid->x.end(), Fluid->pred_x.begin(), [](Vector x) { return x; });
        std::vector<VectorArrayCPU *> x_sets;
        x_sets.emplace_back(&Fluid->pred_x);
        NeighborBuilder.init(x_sets);
    }else{
//        NeighborBuilder.resize_set(0, &Fluid->pred_x); // if new fluid particles is generated, update fluid set
    }

}

void PbfSolver::_for_each_fluid_particle(const std::function<void(size_t, Vector)> &f) {
    parallel_for(Fluid->size, [&](size_t i) { f(i, Fluid->x[i]); });
}

void PbfSolver::_for_each_neighbor_fluid(size_t i, const std::function<void(size_t, Vector)> &f) {
    NeighborBuilder.for_each_neighbor(0, 0, i, f);
}

void PbfSolver::_for_each_fluid_particle_pred(const std::function<void(size_t, Vector)> &f) {
    parallel_for(Fluid->size, [&](size_t i) { f(i, Fluid->pred_x[i]); });
}













