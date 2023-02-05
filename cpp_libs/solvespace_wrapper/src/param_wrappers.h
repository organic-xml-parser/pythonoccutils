#ifndef PARAM_WRAPPERS_H
#define PARAM_WRAPPERS_H

#include "slvs_includes.h"
#include "allocatable.h"
#include <optional>

namespace slvswrap {

    class ParamWrapper : public Allocatable<Slvs_Param> {

        Slvs_hGroup g;

        tReal initialValue;

        std::optional<int> id;

        std::optional<tReal> _computedValue;

        bool _isPrioritized = false;

    public:
        explicit ParamWrapper(const Slvs_hGroup &g, const tReal &initialValue) :
                g(g),
                initialValue(initialValue) {

        }

        void setPrioritized(bool isPrioritized) {
            this->_isPrioritized = isPrioritized;
        }

        bool isPrioritized() {
            return this->_isPrioritized;
        }

        Slvs_Param applyImpl() override {
            return Slvs_MakeParam(this->getId(), g, initialValue);
        }

        tReal getInitialValue() {
            return initialValue;
        }

        bool hasComputedValue() {
            return this->_computedValue.has_value();
        }

        tReal getComputedValue() {
            if (!this->_computedValue.has_value()) {
                throw std::runtime_error("Value has not been computed yet.");
            }

            return this->_computedValue.value();
        }

        void setComputedValue(const tReal &computedValue) {
            this->_computedValue = computedValue;
        }

        void clearComputedValue() {
            if (!this->_computedValue.has_value()) {
                throw std::runtime_error("Computed value not set. Cannot be cleared.");
            }
        }
    };
}

#endif