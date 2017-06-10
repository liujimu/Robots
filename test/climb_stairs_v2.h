/* 步态功能：跨越障碍
 * by liujimu, 2016-12-12
 */

/*将以下注释代码添加到xml文件*/
/*
*/

#ifndef CLIMB_STAIRS_V2_H
#define CLIMB_STAIRS_V2_H

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

struct cs2Param final:public aris::server::GaitParamBase
{
    std::int32_t totalCount{ 1000 };
    std::int32_t n{ 1 };
    double ds = 0.2;//台阶宽度
    double hs = 0.2;//台阶高度
    double h = 0.02;//安全抬腿高度
};

auto climbStairsV2Parse(const std::string &cmd, const std::map<std::string, std::string> &params, aris::core::Msg &msg)->void;
auto climbStairsV2Gait(aris::dynamic::Model &model, const aris::dynamic::PlanParamBase &param_in)->int;

#endif // CLIMB_STAIRS_V2_H
