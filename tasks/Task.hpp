#pragma once

#include "pancam_transformer/TaskBase.hpp"

namespace pancam_transformer {

class Task : public TaskBase {
  public:
    explicit Task(std::string const& name = "pancam_transformer::Task")
            : TaskBase(name) {}

  protected:
    bool configureHook(void);

    void updateHook(void);

  protected:
    Eigen::Vector3d motorToBodyRotation_;

    Eigen::Affine3d sensorToMotorTF_;
    Eigen::Affine3d motorToBodyTF_;
};

}  // namespace pancam_transformer

