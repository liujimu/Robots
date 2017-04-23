#include "cross_obstacle.h"

#ifdef WIN32
#define rt_printf printf
#include <windows.h>
#undef CM_NONE
#endif
#ifdef UNIX
#include "rtdk.h"
#include "unistd.h"
#endif

auto crossObstacleParse(const std::string &cmd, const std::map<std::string, std::string> &params, aris::core::Msg &msg)->void
{
    coParam param;

    for (auto &i : params)
    {
        if (i.first == "totalCount")
        {
            param.totalCount = std::stoi(i.second);
        }
    }

    msg.copyStruct(param);
}

auto crossObstacleGait(aris::dynamic::Model &model, const aris::dynamic::PlanParamBase &param_in)->int
{
    auto &robot = static_cast<Robots::RobotBase &>(model);
    auto &param = static_cast<const coParam &>(param_in);

    //初始化
    static aris::dynamic::FloatMarker beginMak{ robot.ground() };
    static double beginPee[18];
    static double targetPeb[6];

    if (param.count%param.totalCount == 0)
    {
        beginMak.setPrtPm(*robot.body().pm());
        beginMak.update();
        robot.GetPee(beginPee, beginMak);
        std::fill_n(targetPeb, 6, 0);
    }

    double Peb[6], Pee[18];
    std::fill(Peb, Peb + 6, 0);
	std::copy(beginPee, beginPee + 18, Pee);

	const double s = -0.5 * cos(PI * (param.count + 1) / param.totalCount) + 0.5; //s从0到1. 

    double d{ 0 };
    double h{ 0.25 };

    const int leg_begin_id = (param.count / param.totalCount) % 2 == 1 ? 3 : 0;
    if ((param.count / param.totalCount) == 0)//第一步
    {
        d = 0.2;
        targetPeb[0] = d / 2;
        targetPeb[1] = 0.15;
    }
    else if ((param.count / param.totalCount) == 7)//最后一步
    {
        d = 0.2;
        targetPeb[0] = d / 2;
        targetPeb[1] = -0.15;
    }
    else
    {
        d = 0.7;
        targetPeb[0] = d / 2;
    }

    for (int i = leg_begin_id; i < 18; i += 6)
    {
        Pee[i * 3] += -d*(1 - std::cos(PI*s)) / 2;
        Pee[i * 3 + 1] += h*std::sin(PI*s);
    }

    robot.SetPeb(Peb, beginMak);
    robot.SetPee(Pee, beginMak);

    return param.totalCount * 8 - param.count - 1;
}
