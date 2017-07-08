#ifndef OPENVALVE_H
#define OPENVALVE_H

#include <functional>
#include <cstdint>
#include <map>

#include <aris.h>
#include <Robot_Base.h>

struct   OpenValve final:public aris::server::GaitParamBase
{
    std::int32_t totalCount{ 500 };  //INT
    std::int32_t totalCount1{ 1000 };
    double d1{ 0.05 };
    double d2{ 0.075 };
    double h{ 0 };
    std::int32_t n{ 1 };
    
};
auto parseMoveWithOpenValve(const std::string &cmd, const std::map<std::string, std::string> &params, aris::core::Msg &msg)->void;
auto openvalve(aris::dynamic::Model &model, const aris::dynamic::PlanParamBase &param)->int;

#endif
