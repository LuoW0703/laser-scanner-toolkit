#include "lsc/proc/filter.h"
#include "lsc/core/log.h"
#include <algorithm>
#include <cmath>
#include <limits>
#include <map>
#include <memory>
#include <queue>
#include <tuple>
#include <vector>

namespace lsc {

namespace {

class KDTree {
public:
    explicit KDTree(const PointCloud& cloud) : cloud_(cloud) {
        // 通过 nth_element 在当前切分轴上选中位点，递归构造近似平衡树。
        // 构建平均复杂度约 O(N log N)，后续每个 KNN 查询通常远快于
        // 扫描全部 N 个点。
        std::vector<size_t> indices(cloud.size());
        for (size_t i = 0; i < indices.size(); ++i) {
            indices[i] = i;
        }
        root_ = build(indices, 0, indices.size(), 0);
    }

    std::vector<double> nearestDistances(size_t queryIndex, int k) const {
        std::priority_queue<std::pair<double, size_t>> best;
        search(root_.get(), cloud_[queryIndex], queryIndex, k, best);

        std::vector<double> distances;
        distances.reserve(best.size());
        while (!best.empty()) {
            distances.push_back(std::sqrt(best.top().first));
            best.pop();
        }
        return distances;
    }

private:
    struct Node {
        size_t pointIndex = 0;
        int axis = 0;
        std::unique_ptr<Node> left;
        std::unique_ptr<Node> right;
    };

    static double coordinate(const Point3d& point, int axis) {
        return axis == 0 ? point.x() : (axis == 1 ? point.y() : point.z());
    }

    double distanceSquared(const Point3d& a, const Point3d& b) const {
        const double dx = a.x() - b.x();
        const double dy = a.y() - b.y();
        const double dz = a.z() - b.z();
        return dx * dx + dy * dy + dz * dz;
    }

    std::unique_ptr<Node> build(std::vector<size_t>& indices, size_t begin, size_t end, int depth) {
        if (begin >= end) return nullptr;

        const int axis = depth % 3;
        const size_t mid = begin + (end - begin) / 2;
        std::nth_element(indices.begin() + static_cast<std::ptrdiff_t>(begin),
                         indices.begin() + static_cast<std::ptrdiff_t>(mid),
                         indices.begin() + static_cast<std::ptrdiff_t>(end),
                         [&](size_t lhs, size_t rhs) {
                             return coordinate(cloud_[lhs], axis) < coordinate(cloud_[rhs], axis);
                         });

        auto node = std::make_unique<Node>();
        node->pointIndex = indices[mid];
        node->axis = axis;
        node->left = build(indices, begin, mid, depth + 1);
        node->right = build(indices, mid + 1, end, depth + 1);
        return node;
    }

    void search(const Node* node,
                const Point3d& query,
                size_t queryIndex,
                int k,
                std::priority_queue<std::pair<double, size_t>>& best) const {
        if (node == nullptr) return;

        const double diff = coordinate(query, node->axis) - coordinate(cloud_[node->pointIndex], node->axis);
        const Node* first = diff < 0.0 ? node->left.get() : node->right.get();
        const Node* second = diff < 0.0 ? node->right.get() : node->left.get();

        // 优先访问查询点所在半空间。best 是最大堆，堆顶始终是当前
        // K 个候选中最远的点，便于常数时间判断新点能否替换它。
        search(first, query, queryIndex, k, best);

        if (node->pointIndex != queryIndex) {
            const double d2 = distanceSquared(query, cloud_[node->pointIndex]);
            if (static_cast<int>(best.size()) < k) {
                best.emplace(d2, node->pointIndex);
            } else if (d2 < best.top().first) {
                best.pop();
                best.emplace(d2, node->pointIndex);
            }
        }

        // 若查询点到切分平面的距离已经大于当前第 K 近距离，另一侧
        // 不可能出现更近候选，可以安全剪枝；否则必须继续搜索。
        const double axisDist2 = diff * diff;
        if (static_cast<int>(best.size()) < k || axisDist2 < best.top().first) {
            search(second, query, queryIndex, k, best);
        }
    }

    const PointCloud& cloud_;
    std::unique_ptr<Node> root_;
};

} // namespace

// ============================================================================
// 统计离群点滤波
//
// 对每个点计算其到 K 个最近邻的平均距离。
// 假设距离分布接近高斯分布，剔除平均距离超过 mu + stdThresh * sigma 的点。
// 使用 KD-tree KNN，避免大点云下的 O(N^2) 暴力搜索。
// ============================================================================

PointCloud PointCloudFilter::statisticalFilter(const PointCloud& cloud,
                                                int kNeighbors,
                                                double stdThresh) {
    if (cloud.empty()) {
        return cloud;
    }

    const size_t N = cloud.size();

    // 实际 K 值不能超过总点数 - 1
    int K = std::min(kNeighbors, static_cast<int>(N) - 1);
    if (K < 1) {
        return cloud; // 点太少，直接返回
    }

    const KDTree tree(cloud);

    // 对每个点计算到 K 个最近邻的平均距离
    std::vector<double> meanDistances;
    meanDistances.reserve(N);

    for (size_t i = 0; i < N; ++i) {
        const std::vector<double> dists = tree.nearestDistances(i, K);
        if (static_cast<int>(dists.size()) < K) {
            meanDistances.push_back(0.0);
            continue;
        }

        double sum = 0.0;
        for (double dist : dists) {
            sum += dist;
        }
        meanDistances.push_back(sum / static_cast<double>(K));
    }

    // 计算全局均值 μ 和总体标准差 σ。孤立噪点的局部平均距离通常
    // 显著偏大，因此使用 μ + stdThresh*σ 作为可解释的剔除阈值。
    double mu = 0.0;
    for (double d : meanDistances) {
        mu += d;
    }
    mu /= N;

    double sigma = 0.0;
    for (double d : meanDistances) {
        double diff = d - mu;
        sigma += diff * diff;
    }
    sigma = std::sqrt(sigma / N);

    // 阈值
    double threshold = mu + stdThresh * sigma;

    // 过滤
    PointCloud result;
    result.reserve(N);
    for (size_t i = 0; i < N; ++i) {
        if (meanDistances[i] <= threshold) {
            result.push_back(cloud[i]);
        }
    }

    LSC_INFO("Filter") << "statisticalFilter: input=" << N << ", output="
                       << result.size() << ", removed=" << (N - result.size());

    return result;
}

// ============================================================================
// 体素下采样
//
// 将空间按 voxelSize 划分为立方体格网，使用
// std::map<std::tuple<int,int,int>, std::vector<size_t>> 做空间哈希。
// 每个体素内的点用其重心替代。
// ============================================================================

PointCloud PointCloudFilter::voxelDownsample(const PointCloud& cloud,
                                              double voxelSize) {
    if (cloud.empty() || voxelSize <= 0.0) {
        return cloud;
    }

    // 空间哈希：体素索引 floor(x/s), floor(y/s), floor(z/s)
    // 映射到该格内点集合。floor 对负坐标同样保持连续体素划分，
    // 不能用直接截断替代。
    std::map<std::tuple<int, int, int>, std::vector<size_t>> voxelMap;

    for (size_t i = 0; i < cloud.size(); ++i) {
        int ix = static_cast<int>(std::floor(cloud[i].x() / voxelSize));
        int iy = static_cast<int>(std::floor(cloud[i].y() / voxelSize));
        int iz = static_cast<int>(std::floor(cloud[i].z() / voxelSize));
        voxelMap[{ix, iy, iz}].push_back(i);
    }

    // 每个体素取重心
    PointCloud result;
    result.reserve(voxelMap.size());

    for (const auto& kv : voxelMap) {
        const auto& indices = kv.second;
        if (indices.empty()) continue;

        double cx = 0.0, cy = 0.0, cz = 0.0;
        for (size_t idx : indices) {
            cx += cloud[idx].x();
            cy += cloud[idx].y();
            cz += cloud[idx].z();
        }
        double n = static_cast<double>(indices.size());
        result.push_back(Point3d(cx / n, cy / n, cz / n));
    }

    LSC_INFO("Filter") << "voxelDownsample: input=" << cloud.size()
                       << ", output=" << result.size() << ", voxelSize="
                       << voxelSize << " mm";

    return result;
}

} // namespace lsc
