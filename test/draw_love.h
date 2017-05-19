/* 步态功能：移动身体
 * by liujimu, 2016-12-12
 */

#ifndef DRAW_LOVE_H
#define DRAW_LOVE_H

#include <iostream>
#include <cstring>
#include <iomanip>
#include <bitset>
#include <cstring>
#include <map>
#include <string>
#include <stdlib.h>

#include <aris.h>
#include <Robot_Gait.h>
#include <Robot_Base.h>

#ifndef PI
#define PI 3.141592653589793
#endif

struct dlParam final:public aris::server::GaitParamBase
{
    std::int32_t totalCount{ 2000 };
    double r{ 0.15 };
    double alpha{ PI / 180 * 45 };
    double beta{ PI / 180 * 30 };
    double bodyPitch{ PI / 180 * 20 };
    double bodyUp{ 0.15 };
    double bodyBack{ 0.15 };
};

auto drawLoveParse(const std::string &cmd, const std::map<std::string, std::string> &params, aris::core::Msg &msg)->void;
auto drawLoveGait(aris::dynamic::Model &model, const aris::dynamic::PlanParamBase &param_in)->int;

#endif // DRAW_LOVE_H
