#ifndef ROBOT_GAIT_H
#define ROBOT_GAIT_H

#include <functional>
#include <cstdint>
#include <map>

#include <aris.h>
#include <Robot_Base.h>

namespace Robots
{
	auto basicParse(const std::string &cmd, const std::map<std::string, std::string> &params, aris::core::Msg &msg_out)->void;

	struct RecoverParam final :public aris::server::GaitParamBase
	{
		std::int32_t recover_count{ 3000 };
		std::int32_t align_count{ 3000 };
		bool active_leg[6]{ true,true,true,true,true,true };
		double margin_offset{ 0.01 };//meter
		double alignPee[18]
        {  -0.30,   -0.40,   -0.52,
           -0.60,   -0.40,    0,
           -0.30,   -0.40,    0.52,
            0.30,   -0.40,   -0.52,
            0.60,   -0.40,    0,
            0.30,   -0.40,    0.52 };
        double recoverPee[18]
        {  -0.30,   -0.50,   -0.52,
           -0.60,   -0.50,    0,
           -0.30,   -0.50,    0.52,
            0.30,   -0.50,   -0.52,
            0.60,   -0.50,    0,
            0.30,   -0.50,    0.52 };
	};

	auto recoverParse(const std::string &cmd, const std::map<std::string, std::string> &params, aris::core::Msg &msg_out)->void;
	auto recoverGait(aris::dynamic::Model &model, const aris::dynamic::PlanParamBase & plan_param)->int;

	struct WalkParam final :public aris::server::GaitParamBase
	{
		std::int32_t totalCount{ 3000 };
		std::int32_t n{ 2 };
		double d{ 0.5 };
		double h{ 0.05 };
		double alpha{ 0.3 };
		double beta{ 0.3 };
	};
	auto walkParse(const std::string &cmd, const std::map<std::string, std::string> &params, aris::core::Msg &msg)->void;
	auto walkGait(aris::dynamic::Model &model, const aris::dynamic::PlanParamBase &param)->int;

	struct ResetOriginParam final :public aris::server::GaitParamBase {};
	auto resetOriginParse(const std::string &cmd, const std::map<std::string, std::string> &params, aris::core::Msg &msg)->void;
	auto resetOriginGait(aris::dynamic::Model &model, const aris::dynamic::PlanParamBase &param)->int;
}

#endif
