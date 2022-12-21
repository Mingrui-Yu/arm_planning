#ifndef ARM_PLANNING_PLANNING_RESPONSE_H
#define ARM_PLANNING_PLANNING_RESPONSE_H

#include <arm_planning/common_include.h>

namespace dual_arm_planning{

class PlanningResponse
{
public:
    PlanningResponse(){}

public:
    std::vector<std::vector<double> > arm_0_path_;
    std::vector<std::vector<double> > arm_1_path_;

    bool success_;
    double total_time_cost_;
    int total_iter_;



}; // end class

} // end namespace

#endif