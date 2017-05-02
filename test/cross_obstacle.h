/* 步态功能：跨越障碍
 * by liujimu, 2016-12-12
 */

/*将以下注释代码添加到xml文件*/
/*
            <mb default="mb_param">
                <mb_param type="group">
                    <totalCount abbreviation="t" type="int" default="2000"/>
                    <d abbreviation="d" type="double" default="0"/>
                    <h abbreviation="h" type="double" default="0"/>
                    <z abbreviation="z" type="double" default="0"/>
                    <pitch abbreviation="u" type="double" default="0"/>
                    <yaw abbreviation="v" type="double" default="0"/>
                    <roll abbreviation="w" type="double" default="0"/>
                </mb_param>
            </mb>
*/

#ifndef CROSS_OBSTACLE_H
#define CROSS_OBSTACLE_H

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

struct coParam final:public aris::server::GaitParamBase
{
    std::int32_t totalCount{ 2000 };
	std::int32_t n{ 5 };
	double d{ 0.5 };
	double h{ 0.28 };
	double y{ 0.15 };
};

auto crossObstacleParse(const std::string &cmd, const std::map<std::string, std::string> &params, aris::core::Msg &msg)->void;
auto crossObstacleGait(aris::dynamic::Model &model, const aris::dynamic::PlanParamBase &param_in)->int;

#endif // CROSS_OBSTACLE_H
