#ifndef UTIL_H
#define UTIL_H

#include <iostream>
#include <optional>

namespace slvswrap {


    template<typename U, typename V>
    class Alternate {

        std::optional<U> first;
        std::optional<V> second;

        Alternate(std::optional<U> first, std::optional<V> second) :
                first(std::move(first)),
                second(std::move(second)) {
        }

    public:

        U getFirst() const {
            if (!first.has_value()) {
                throw std::runtime_error("First value not allocated.");
            }

            return first.value();
        }

        V getSecond() const {
            if (first.has_value()) {
                throw std::runtime_error("Second value not allocated.");
            }

            return second.value();
        }

        bool hasFirst() {
            return first.has_value();
        }

        static Alternate<U, V> with_first(U first) {
            return Alternate(first, std::nullopt);
        }

        static Alternate<U, V> with_second(V second) {
            return Alternate(std::nullopt, second);
        }
    };

}

#endif