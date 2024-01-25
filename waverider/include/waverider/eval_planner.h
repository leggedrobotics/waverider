#ifndef WAVERIDER_EVAL_PLANNER_H_
#define WAVERIDER_EVAL_PLANNER_H_

#include <chrono>
#include <iostream>
#include <string>
#include <vector>

#include <Eigen/Dense>

namespace waverider {

class EvalPlanner {
 public:
  typedef struct {
    bool success;
    std::chrono::steady_clock::duration duration;
    std::vector<Eigen::Vector3d> states_out;
  } Result;

  virtual std::string getName() = 0;
  virtual Result plan(Eigen::Vector3d start, Eigen::Vector3d end) = 0;

  Eigen::Vector3d color;
};

}  // namespace waverider

#endif  // WAVERIDER_EVAL_PLANNER_H_
