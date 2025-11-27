#ifndef my_pinocchio_h
#define my_pinocchio_h

#include <iostream>
#include <pinocchio/parsers/urdf.hpp>
#include <pinocchio/algorithm/kinematics.hpp>
#include <pinocchio/multibody/model.hpp>
#include <pinocchio/multibody/data.hpp>
#include <Eigen/Dense>

void pinocchio_test()
{
    std::string urdf_path =
        "/opt/openrobots/share/example-robot-data/robots/ur_description/urdf/ur5_robot.urdf";

    // 构建模型
    pinocchio::Model model;
    pinocchio::urdf::buildModel(urdf_path, model);
    pinocchio::Data data(model);

    std::cout << "Robot: " << model.name << std::endl;
    std::cout << "joints: " << model.njoints << std::endl;

    // 初始配置 q = 0
    Eigen::VectorXd q = Eigen::VectorXd::Zero(model.nq);

    // 前向运动学
    pinocchio::forwardKinematics(model, data, q);

    // 末端（最后一个 joint 对应的 frame）
    const auto &oMi = data.oMi.back();

    std::cout << "EE position = " << oMi.translation().transpose() << std::endl;
}

#endif