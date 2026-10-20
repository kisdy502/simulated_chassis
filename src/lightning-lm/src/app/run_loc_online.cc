//
// Created by xiang on 25-3-18.
//

#include <gflags/gflags.h>
#include <glog/logging.h>
#include <yaml-cpp/yaml.h>

#include "core/system/loc_system.h"
#include "ui/pangolin_window.h"
#include "wrapper/ros_utils.h"

DEFINE_string(config, "./config/default.yaml", "配置文件");

/// 运行定位的测试
int main(int argc, char** argv) {
    google::InitGoogleLogging(argv[0]);
    FLAGS_colorlogtostderr = true;
    FLAGS_stderrthreshold = google::INFO;

    google::ParseCommandLineFlags(&argc, &argv, true);
    using namespace lightning;

    rclcpp::init(argc, argv);

    LocSystem::Options opt;
    LocSystem loc(opt);

    if (!loc.Init(FLAGS_config)) {
        LOG(ERROR) << "failed to init loc";
    }

    /// sim-chassis 补丁：支持 yaml loc.init_pose: [x, y, z, qx, qy, qz, qw]（缺省单位位姿，即建图起点）
    SE3 init_pose = SE3();
    try {
        auto yaml = YAML::LoadFile(FLAGS_config);
        if (yaml["loc"] && yaml["loc"]["init_pose"]) {
            auto v = yaml["loc"]["init_pose"];
            if (v.size() == 7) {
                // YAML/ROS: x,y,z,w; Eigen constructor: w,x,y,z.
                init_pose = SE3(Quatd(v[6].as<double>(), v[3].as<double>(), v[4].as<double>(), v[5].as<double>()).normalized(),
                                Vec3d(v[0].as<double>(), v[1].as<double>(), v[2].as<double>()));
                LOG(INFO) << "loaded init_pose from yaml";
            } else {
                LOG(WARNING) << "loc.init_pose should have 7 elements [x,y,z,qx,qy,qz,qw], using identity";
            }
        }
    } catch (const std::exception& e) {
        LOG(WARNING) << "parse init_pose failed: " << e.what();
    }

    loc.SetInitPose(init_pose);
    loc.Spin();

    rclcpp::shutdown();

    return 0;
}