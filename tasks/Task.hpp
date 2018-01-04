#pragma once

#include "pancam_transformer/TaskBase.hpp"

namespace pancam_transformer {

class Task : public TaskBase {
  public:
    explicit Task(std::string const& name = "pancam_transformer::Task")
            : TaskBase(name) {}

  protected:
    void updateHook(void);
};

}  // namespace pancam_transformer

