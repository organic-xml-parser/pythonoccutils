#include "Allocator.h"
#include "constraint_wrappers.h"
#include <iostream>

namespace slvswrap {

    void Allocator::assertHasGroup(const Slvs_hGroup &group) {
        if (group <= 0 || group > this->groupCount) {
            throw std::runtime_error("Group is not known.");
        }
    }

    Slvs_hGroup Allocator::addGroup() {
        groupCount++;
        return groupCount;
    }

    void Allocator::allocate() {
        sys.param = new Slvs_Param[this->paramWrappers.size()];
        sys.params = this->paramWrappers.size();

        sys.entity = new Slvs_Entity[this->entityWrappers.size()];
        sys.entities = this->entityWrappers.size();

        sys.constraint = new Slvs_Constraint[this->constraintWrappers.size()];
        sys.constraints = this->constraintWrappers.size();

        // clear the dragged parameters
        sys.dragged[0] = 0;
        sys.dragged[1] = 0;
        sys.dragged[2] = 0;
        sys.dragged[3] = 0;

        int handle = 1;
        int currentDraggedAddress = 0;
        for (auto &paramWrapper: this->paramWrappers) {
            paramWrapper->allocate(handle);

            if (paramWrapper->isPrioritized()) {
                if (currentDraggedAddress >= 4) {
                    throw std::runtime_error("There are more than 4 prioritized (dragged) parameters.");
                }

                sys.dragged[currentDraggedAddress] = handle;
                currentDraggedAddress++;
            }

            handle++;
        }

        for (auto &entityWrapper: this->entityWrappers) {
            entityWrapper->allocate(handle);

            handle++;
        }

        for (auto &constraintWrapper: this->constraintWrappers) {
            constraintWrapper->allocate(handle);

            handle++;
        }
    }

    void Allocator::apply() {
        for (int i = 0; i < paramWrappers.size(); i++) {
            sys.param[i] = paramWrappers[i]->apply();
        }

        for (int i = 0; i < entityWrappers.size(); i++) {
            sys.entity[i] = entityWrappers[i]->apply();
        }

        for (int i = 0; i < constraintWrappers.size(); i++) {
            sys.constraint[i] = constraintWrappers[i]->apply();
        }
    }

    void Allocator::solve(const Slvs_hGroup & group) {
        assertHasGroup(group);

        Slvs_Solve(&sys, group);
        if (sys.result == SLVS_RESULT_OKAY) {
            std::cout << "Solve success! (" << sys.dof << " dof)" << std::endl;
            std::cout << "Populating computed values..." << std::endl;
            for (int i = 0; i < this->paramWrappers.size(); i++) {
                this->paramWrappers[i]->setComputedValue(sys.param[i].val);
            }
        } else {
            std::cout << "Solve failed!" << std::endl;
            //printf("solve failed");
        }
    }

    void Allocator::dispose() {
        delete[] sys.entity;
        delete[] sys.param;
        delete[] sys.constraint;
    }

}