#include "lsc/proc/filter.h"

#include <chrono>
#include <cmath>
#include <cstdlib>
#include <iostream>
#include <random>

namespace {

lsc::PointCloud generateCloud(size_t count) {
    std::mt19937 rng(42);
    std::normal_distribution<double> noise(0.0, 0.2);

    lsc::PointCloud cloud;
    cloud.reserve(count + 16);
    for (size_t i = 0; i < count; ++i) {
        const double t = static_cast<double>(i);
        cloud.push_back({
            std::fmod(t, 100.0) + noise(rng),
            std::fmod(t / 100.0, 100.0) + noise(rng),
            std::sin(t * 0.01) + noise(rng),
        });
    }

    for (int i = 0; i < 16; ++i) {
        cloud.push_back({1000.0 + i, -1000.0, 500.0});
    }
    return cloud;
}

} // namespace

int main() {
    constexpr size_t kPointCount = 10000;
    constexpr long long kMaxMillis = 5000;

    const lsc::PointCloud cloud = generateCloud(kPointCount);
    const auto start = std::chrono::steady_clock::now();
    const lsc::PointCloud filtered = lsc::PointCloudFilter::statisticalFilter(cloud, 50, 1.0);
    const auto end = std::chrono::steady_clock::now();

    const auto ms = std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count();
    std::cout << "statisticalFilter(" << cloud.size() << " pts): " << ms
              << " ms, output=" << filtered.size() << "\n";

    return ms > kMaxMillis ? EXIT_FAILURE : EXIT_SUCCESS;
}
