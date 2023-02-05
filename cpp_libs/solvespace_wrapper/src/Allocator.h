#ifndef ALLOCATOR_H
#define ALLOCATOR_H

#include "slvs_includes.h"
#include <memory>
#include <vector>
#include <set>

namespace slvswrap {

    class ParamWrapper;
    class EntityWrapper;
    class ConstraintWrapper;


    class Allocator {
    private:
        Slvs_System sys;

        Slvs_hGroup groupCount = 1;

        std::vector<std::shared_ptr<ParamWrapper>> paramWrappers;
        std::vector<std::shared_ptr<EntityWrapper>> entityWrappers;
        std::vector<std::shared_ptr<ConstraintWrapper>> constraintWrappers;

        void assertHasGroup(const Slvs_hGroup & group);

    public:
        inline Slvs_hGroup defaultGroup() {
            return 1;
        }

        Slvs_hGroup addGroup();

        template<typename ...T>
        std::weak_ptr<ParamWrapper> requestParamWrapper(const Slvs_hGroup& g, T... values) {
            auto result = std::make_shared<ParamWrapper>(g, values...);

            paramWrappers.push_back(result);

            return result;
        }

        template<typename TEntity, typename ...T>
        std::weak_ptr<TEntity> requestEntityWrapper(const Slvs_hGroup& g, T &&... values) {
            auto result = std::make_shared<TEntity>(*this, g, values...);

            entityWrappers.push_back(result);

            return result;
        }

        template<typename TConstraint, typename ...T>
        std::weak_ptr<TConstraint> requestConstraintWrapper(const Slvs_hGroup& g, T &&... values) {
            auto result = std::make_shared<TConstraint>(*this, g, values...);

            constraintWrappers.push_back(result);

            return result;
        }

        void allocate();

        void apply();

        /**
         * @param group The group to solve
         */
        void solve(const Slvs_hGroup & group);

        void dispose();

    };


}

#endif