/* 步态功能：跨越障碍
 * by liujimu, 2016-12-12
 */

/*将以下注释代码添加到xml文件*/
/*
            <mb default="mb_param">
                <mb_param type="group">
                    <totalCount abbreviation="t" type="int" default="3000"/>
					<n abbreviation="n" type="int" default="4"/>
					<d abbreviation="d" type="double" default="0.7"/>
                    <h abbreviation="h" type="double" default="0.25"/>
                    <y abbreviation="y" type="double" default="0.12"/>
                </mb_param>
            </mb>
*/

#ifndef ADJUST_PEE_H
#define ADJUST_PEE_H

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

struct apParam final:public aris::server::GaitParamBase
{
    std::int32_t totalCount{ 2000 };
	double x{ 0.1 }; 
	double z{ 0 };
	double h{ 0.05 };
};

auto adjustPeeParse(const std::string &cmd, const std::map<std::string, std::string> &params, aris::core::Msg &msg)->void;
auto adjustPeeGait(aris::dynamic::Model &model, const aris::dynamic::PlanParamBase &param_in)->int;

#endif // ADJUST_PEE_H
