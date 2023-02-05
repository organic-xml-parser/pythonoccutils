#ifndef CONSTRAINT_WRAPPERS_H
#define CONSTRAINT_WRAPPERS_H

#include "slvs_includes.h"
#include "util.h"
#include "param_wrappers.h"
#include "entity_wrappers.h"
#include "Allocator.h"
#include <vector>
#include <memory>

namespace slvswrap {

    class ConstraintWrapper : public Allocatable<Slvs_Constraint> {
    protected:
        std::vector<std::weak_ptr<EntityWrapper>> entities;
    };


    class PtPtDistanceWrapper : public ConstraintWrapper {

        Slvs_hGroup g;
        tReal distance;

    public:
        PtPtDistanceWrapper(Allocator &alloc,
                            const Slvs_hGroup &g,
                            const std::weak_ptr<PointWrapper> &p0,
                            const std::weak_ptr<PointWrapper> &p1,
                            const tReal &distance) : g(g), distance(distance) {

            this->entities.push_back(p0);
            this->entities.push_back(p1);
        }

        Slvs_Constraint applyImpl() override {
            auto e_id0 = this->entities[0].lock()->getId();
            auto e_id1 = this->entities[1].lock()->getId();

            return Slvs_MakeConstraint(
                    getId(),
                    g,
                    SLVS_C_PT_PT_DISTANCE,
                    SLVS_FREE_IN_3D,
                    distance,
                    e_id0,
                    e_id1,
                    0,
                    0);
        }
    };
}

#endif