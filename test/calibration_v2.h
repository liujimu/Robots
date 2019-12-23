/* 步态功能：腿部机构标定
* by liujimu, 2019-9-14
*/

/*将以下注释代码添加到xml文件*/
/*
            <cl2 default="cl2_param">
                <cl2_param type="group">
                    <totalCount abbreviation="t" type="int" default="2000"/>
					<waitingCount abbreviation="w" type="int" default="2000"/>
					<leg abbreviation="l" type="int" default="0"/>
                </cl2_param>
            </cl2>
*/

#ifndef CALIBRATION_H
#define CALIBRATION_H

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

//enum class Command
//{
//    NONE = 0,
//    NEXT = 1,
//    PREVIOUS = 2,
//    QUIT = 99
//};

//class CaliCmd
//{
//public:
//    static CaliCmd& getCmd()
//    {
//        static CaliCmd c;
//        return c;
//    }
//    Command& cmd() { return cmd_; }
//private:
//    Command cmd_{ Command::NEXT };
//    CaliCmd() = default;
//};

/*gait parameters*/
struct caliParam2 final :public aris::server::GaitParamBase
{
    std::int32_t totalCount{ 3000 };
	std::int32_t waitingCount{ 3000 };
    int leg_id{ 0 };
};

auto calibrationParse(const std::string &cmd, const std::map<std::string, std::string> &params, aris::core::Msg &msg)->void;
auto calibrationGait(aris::dynamic::Model &model, const aris::dynamic::PlanParamBase &param_in)->int;

#endif // CALIBRATION_H
