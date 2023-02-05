#include "solvespace_wrapper.h"
#include "param_wrappers.h"

int main() {
    using namespace slvswrap;

    Allocator allocator;

    auto wp_group = allocator.defaultGroup();
    auto wp = allocator.requestEntityWrapper<WorkPlaneWrapper>(wp_group,
                                                               0, 0, 0,
                                                               1, 0, 0,
                                                               0, 1, 0);


    auto group = allocator.addGroup();

    auto p0 = allocator.requestEntityWrapper<Point2DWrapper>(group, wp, 10, 10);
    auto p1 = allocator.requestEntityWrapper<Point2DWrapper>(group, wp, 20, 20);

    std::cout << *wp.lock() << std::endl;

    std::cout << "Allocating..." << std::endl;
    allocator.allocate();

    std::cout << *wp.lock() << std::endl;

    std::cout << "Applying..." << std::endl;
    allocator.apply();

    std::cout << *wp.lock() << std::endl;

    std::cout << "Solving..." << std::endl;
    allocator.solve(group);

    std::cout << "WORKPLANE" << std::endl;
    std::cout << *wp.lock() << std::endl;

    std::cout << "Points" << std::endl;
    std::cout << *p0.lock() << std::endl;
    std::cout << *p1.lock() << std::endl;


    allocator.dispose();

    return 0;
}
