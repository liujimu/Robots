/* 步态功能：验证腿部机构运动精度
* by liujimu, 2019-9-18
*/

/*将以下注释代码添加到xml文件*/
/*
            <vl default="vl_param">
                <vl_param type="group">
                    <totalCount abbreviation="t" type="int" default="5000"/>
					<waitingCount abbreviation="w" type="int" default="2000"/>
					<leg abbreviation="l" type="int" default="0"/>
                </vl_param>
            </vl>
*/

#ifndef VALIDATION_H
#define VALIDATION_H

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

/*gait parameters*/
struct valiParam final :public aris::server::GaitParamBase
{
    std::int32_t totalCount{ 3000 };
	std::int32_t waitingCount{ 3000 };
    int leg_id{ 0 };
};

auto validationParse(const std::string &cmd, const std::map<std::string, std::string> &params, aris::core::Msg &msg)->void;
auto validationGait(aris::dynamic::Model &model, const aris::dynamic::PlanParamBase &param_in)->int;

#endif // VALIDATION_H
