#ifndef ALLOCATABLE_H
#define ALLOCATABLE_H

#include <optional>
#include <exception>
#include <iostream>

namespace slvswrap {

    template<typename TAppliedType>
    class Allocatable {
    private:
        std::optional<int> id = std::nullopt;

    public:
        void serializeId(std::ostream& os) const {
            if (!id.has_value()) {
                os << "N/A";
            } else {
                os << id.value();
            }
        }

        void allocate(const int &allocatedId) {
            if (this->id.has_value()) {
                throw std::runtime_error("Already allocated");
            }

            this->id = allocatedId;
        }

        int getId() {
            if (!this->id.has_value()) {
                throw std::runtime_error("Not allocated");
            }

            return this->id.value();
        }

        TAppliedType apply() {
            if (!this->id.has_value()) {
                throw std::runtime_error("Not yet allocated");
            }

            return applyImpl();
        }

    protected:
        virtual TAppliedType applyImpl() = 0;
    };

}

#endif