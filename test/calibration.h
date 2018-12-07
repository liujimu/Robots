/* 步态功能：腿部机构标定
* by zhaoyue, 2018-12-7
*/

/*将以下注释代码添加到xml文件*/
/*
            <sw default="sw_param">
                <sw_param type="group">
                    <totalCount abbreviation="t" type="int" default="8000"/>
                    <xAngle abbreviation="x" type="double" default="0"/>
                    <zAngle abbreviation="z" type="double" default="0"/>
                    <yAngle abbreviation="y" type="double" default="0"/>
                    <rDistance abbreviation="r" type="double" default="0.5"/>
                    <yDistance abbreviation="d" type="double" default="-0.05"/>
                </sw_param>
            </sw>
*/

#pragma once
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

class CaliState
{
public:
    static CaliState& getState()
    {
        static CaliState s;
        return s;
    }
    bool& isStopping() { return isStopping_; }
private:
    bool isStopping_{ true };
    CaliState() = default;
};

/*gait parameters*/
struct caliParam final :public aris::server::GaitParamBase
{
    std::int32_t totalCount{ 8000 };
    int leg_id{ 0 };
};

auto calibrationParse(const std::string &cmd, const std::map<std::string, std::string> &params, aris::core::Msg &msg)->void;
auto calibrationGait(aris::dynamic::Model &model, const aris::dynamic::PlanParamBase &param_in)->int;
