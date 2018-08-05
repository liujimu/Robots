/* 步态功能：抬起两只前腿，用后四条腿走路
* by liujimu, 2018-8-4
*/

/*将以下注释代码添加到xml文件*/
/*
            <sh default="sh_param">
                <sh_param type="group">
                    <totalCount abbreviation="t" type="int" default="2000"/>
                    <stepLength abbreviation="d" type="double" default="0.2"/>
                    <stepHeight abbreviation="h" type="double" default="0.05"/>
                    <bodyUp abbreviation="u" type="double" default="0.04"/>
                    <bodyPitch abbreviation="p" type="double" default="20"/>
                    <helloAmplitude abbreviation="a" type="double" default="0.1"/>
                    <helloTimes abbreviation="n" type="int" default="1"/>
					<direction type="unique" default="backward">
                        <forward abbreviation="f"/>
                        <backward abbreviation="b"/>
					</direction>
                </sh_param>
            </sh>
*/

#ifndef QUADRUPED_GAIT_H
#define QUADRUPED_GAIT_H

#include <iostream>
#include <cstring>
#include <iomanip>
#include <bitset>
#include <cstring>
#include <map>
#include <string>
#include <stdlib.h>
#include <atomic>

#include <aris.h>
#include <Robot_Gait.h>
#include <Robot_Base.h>

#ifndef PI
#define PI 3.141592653589793
#endif

struct qgParam final :public aris::server::GaitParamBase
{
    std::int32_t totalCount{ 1000 };
    double bodyUp{ 0.0 };
    double bodyBack{ 0.1 };
    double bodyPitch{ PI / 18 };
    double footOffset{ 0.2 };
    double stepLength{ 0.1 };
    double stepHeight{ 0.04 };
    std::int32_t n{ 1 };
};

auto quadrupedGaitParse(const std::string &cmd, const std::map<std::string, std::string> &params, aris::core::Msg &msg)->void;
auto quadrupedGait(aris::dynamic::Model &model, const aris::dynamic::PlanParamBase &param_in)->int;

#endif // QUADRUPED_GAIT_H
