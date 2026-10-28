#pragma once

#include <algorithm>
#include <vector>
#include "common/std_types.h"
#include "core/graph/optimizer.h"
#include "core/opti_algo/algo_select.h"
#include "core/types/edge_se3.h"

namespace lightning {

// A historical target must not contain the source frame or its recent local
// neighborhood. Otherwise NDT can match the source against its own trajectory.
inline bool IsHistoricalLoopFrame(int frame, int current, int minimum_gap) {
    return frame >= 0 && frame <= current - std::max(1, minimum_gap);
}

inline std::shared_ptr<miao::Optimizer> MakeLoopOptimizer() {
    miao::OptimizerConfig config(miao::AlgorithmType::LEVENBERG_MARQUARDT,
                                miao::LinearSolverType::LINEAR_SOLVER_SPARSE_EIGEN, false);
    // Incremental InitFromLast only appends new edges; it does not remove old
    // active edges whose Level was changed by outlier rejection.
    config.incremental_mode_ = false;
    return miao::SetupOptimizer<6, 3>(config);
}

inline std::shared_ptr<miao::VertexSE3> MakeLoopVertex(int id, const SE3& pose) {
    auto vertex = std::make_shared<miao::VertexSE3>();
    vertex->SetId(id);
    vertex->SetEstimate(pose);
    vertex->SetFixed(id == 0);
    return vertex;
}

struct LoopRefinement {
    std::size_t rejected = 0;
    std::size_t newly_rejected = 0;
};

inline LoopRefinement RefineLoopGraph(miao::Optimizer& optimizer,
    const std::vector<std::shared_ptr<miao::EdgeSE3>>& loops) {
    optimizer.InitializeOptimization();
    optimizer.Optimize(20);
    LoopRefinement result;
    for (const auto& edge : loops) {
        if (edge->Level() != 0) {
            ++result.rejected;
            continue;
        }
        const auto kernel = edge->GetRobustKernel();
        if (kernel && edge->Chi2() > kernel->Delta()) {
            edge->SetLevel(1);
            ++result.rejected;
            ++result.newly_rejected;
        }
        // Keep inlier robust kernels: later loop closures may expose an earlier
        // false match. Removing its kernel would make it permanently trusted.
    }
    if (result.newly_rejected) {
        // Rebuild the level-0 set and undo the pull of the rejected constraints
        // before updating keyframes/redrawing/saving the map.
        optimizer.InitializeOptimization();
        optimizer.Optimize(20);
    }
    return result;
}

}  // namespace lightning
