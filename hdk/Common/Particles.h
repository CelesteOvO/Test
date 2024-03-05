//
// Created by LiYifan on 2024/2/26.
//

#ifndef HINAPE_HOUDINI_PARTICLES_H
#define HINAPE_HOUDINI_PARTICLES_H

#include <functional>
#include "tbb/tbb.h"

inline void serial_for(size_t n, const std::function<void(size_t)> &f) { for (size_t i = 0; i < n; ++i) { f(i); }}
inline void parallel_for(size_t n, const std::function<void(size_t)> &f) { tbb::parallel_for(size_t(0), n, [&](size_t i) { f(i); }); }
//inline void parallel_for(size_t n, const std::function<void(size_t)> &f) { serial_for(n, f); }

template<typename real, typename Vector3, typename ScalarArray, typename Vector3Array>
struct IFluid
{
    size_t size; // number of particles
    Vector3Array x; // position
    Vector3Array v; // velocity
    Vector3Array a; // acceleration
    ScalarArray m; // mass
    ScalarArray V; // volume
    ScalarArray rho; // density
    ScalarArray neighbor_this;
    ScalarArray neighbor_others;
};


#endif //HINAPE_HOUDINI_PARTICLES_H
