#include "climb_stairs_v2.h"

#ifdef WIN32
#define rt_printf printf
#include <windows.h>
#undef CM_NONE
#endif
#ifdef UNIX
#include "rtdk.h"
#include "unistd.h"
#endif

auto climbStairsV2Parse(const std::string &cmd, const std::map<std::string, std::string> &params, aris::core::Msg &msg)->void
{
    cs2Param param;

    for (auto &i : params)
    {
        if (i.first == "totalCount")
        {
            param.totalCount = std::stoi(i.second);
        }
    }

    msg.copyStruct(param);
}

auto climbStairsV2Gait(aris::dynamic::Model &model, const aris::dynamic::PlanParamBase &param_in)->int
{
    auto &robot = static_cast<Robots::RobotBase &>(model);
    auto &param = static_cast<const cs2Param &>(param_in);

    //初始化
    static aris::dynamic::FloatMarker beginMak{ robot.ground() };
    static double beginPee[18];

	if (param.count%param.totalCount == 0)
	{
		beginMak.setPrtPm(*robot.body().pm());
		beginMak.update();
		robot.GetPee(beginPee, beginMak);
    }

    const int leg_begin_id = (param.count / param.totalCount) % 2 == 1 ? 0 : 3;
	const int period_count = param.count%param.totalCount;
	const double s = -0.5 * cos(PI * (period_count + 1) / param.totalCount) + 0.5; //s从0到1. 

    const double hs = param.hs;
    const double ds = param.ds;
    const double l = std::sqrt(hs*hs + ds*ds);
    const double h = param.h;
    const double angle = -1 * std::atan2(hs, ds);
    const double sa = std::sin(angle);
    const double ca = std::cos(angle);
    double x = ds / 2 * (1 - std::cos(PI*s));
    double y = s < 0.5 ? (hs + h)*std::sin(PI*s) : (hs + h*std::sin(PI*s));
    double x2b = ca*x - sa*y;
    double y2b = sa*x + ca*y;

	double Peb[6], Pee[18];
	std::fill(Peb, Peb + 6, 0);
	std::copy(beginPee, beginPee + 18, Pee);
	for (int i = leg_begin_id; i < 18; i += 6)
    {
        Pee[i] += x2b;
        Pee[i + 1] += y2b;
    }
    
    const double s1 = -0.5 * cos(PI * (param.count%(2*param.totalCount) + 1) / (2*param.totalCount)) + 0.5; //s从0到1. 
    Peb[0] = l/2 * s;

    robot.SetPeb(Peb, beginMak);
    robot.SetPee(Pee, beginMak);

    return 2 * param.totalCount * param.n - param.count - 1;
}
