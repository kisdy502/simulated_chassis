#include <iostream>
#include <stdexcept>
#include "core/loop_closing/loop_validation.h"
#include "core/robust_kernel/cauchy.h"

using namespace lightning;

static void require(bool value, const char* message) {
    if (!value) throw std::runtime_error(message);
}

int main() {
    require(IsHistoricalLoopFrame(80, 100, 20), "Boundary history lost");
    require(!IsHistoricalLoopFrame(81, 100, 20), "Recent neighborhood leaked into target");
    require(!IsHistoricalLoopFrame(100, 100, 0), "Source frame leaked into target");
    require(!IsHistoricalLoopFrame(101, 100, 20), "Future frame leaked into target");
    require(!IsHistoricalLoopFrame(-1, 100, 20), "Negative frame accepted");

    auto optimizer = MakeLoopOptimizer();
    require(!optimizer->GetConfig().incremental_mode_, "Outlier levels must rebuild the active graph");
    optimizer->SetVerbose(false);
    // Pure translation: forward, sideways, backward, sideways. No yaw change.
    const std::vector<Vec3d> positions{{0, 0, 0}, {1, 0, 0}, {1, 1, 0}, {0, 1, 0}, {0, 0, 0}, {0.5, -0.5, 0}};
    std::vector<std::shared_ptr<miao::VertexSE3>> vertices;
    std::vector<std::shared_ptr<miao::EdgeSE3>> loops;
    auto add_vertex = [&](int id) {
        auto vertex = MakeLoopVertex(id, SE3(SO3(), positions[id]));
        vertices.push_back(vertex);
        optimizer->AddVertex(vertex);
    };
    auto edge = [&](int from, int to, const Vec3d& translation, bool loop) {
        auto constraint = std::make_shared<miao::EdgeSE3>();
        constraint->SetVertex(0, vertices[from]);
        constraint->SetVertex(1, vertices[to]);
        constraint->SetMeasurement(SE3(SO3(), translation));
        constraint->SetInformation(Mat6d::Identity() * (loop ? 25.0 : 100.0));
        if (loop) {
            auto robust = std::make_shared<miao::RobustKernelCauchy>();
            robust->SetDelta(1.04);
            constraint->SetRobustKernel(robust);
            loops.push_back(constraint);
        }
        optimizer->AddEdge(constraint);
        return constraint;
    };
    for (int id = 0; id < 5; ++id) {
        add_vertex(id);
        if (id) edge(id-1, id, positions[id]-positions[id-1], false);
    }
    auto good = edge(0, 4, Vec3d::Zero(), true);
    auto bad = edge(0, 2, Vec3d(6, 5, 0), true);
    auto first = RefineLoopGraph(*optimizer, loops);
    require(first.newly_rejected == 1 && bad->Level() == 1, "False translation loop not rejected");
    require(good->Level() == 0 && good->GetRobustKernel(), "Good loop lost future validation");
    require(std::find(optimizer->ActiveEdges().begin(), optimizer->ActiveEdges().end(), bad.get()) == optimizer->ActiveEdges().end(),
            "Rejected loop still acts on optimizer");
    require(vertices[0]->Fixed() && vertices[0]->Estimate().translation().norm() < 1e-12, "Map anchor moved");
    for (int id = 1; id < 5; ++id) {
        require((vertices[id]->Estimate().translation()-positions[id]).norm() < 1e-5,
                "Rejected loop left deformation in translation-only trajectory");
    }
    add_vertex(5);
    edge(4, 5, positions[5]-positions[4], false);
    edge(2, 5, positions[5]-positions[2], true);
    auto later_bad = edge(1, 5, Vec3d(5, -3, 0), true);
    auto second = RefineLoopGraph(*optimizer, loops);
    require(second.rejected == 2 && second.newly_rejected == 1, "Outlier filtering broken after graph growth");
    require(std::find(optimizer->ActiveEdges().begin(), optimizer->ActiveEdges().end(), bad.get()) == optimizer->ActiveEdges().end(),
            "Old rejected edge reactivated");
    require(later_bad->Level() == 1, "New outlier not excluded");
    require((vertices[5]->Estimate().translation()-positions[5]).norm() < 1e-5, "Second pass did not remove outlier pull");
    std::cout << "PASS: historical target isolation, translation-only loop closure, outlier removal/reoptimization, persistent rejection, map anchor\n";
}
