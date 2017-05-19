#include "draw_love.h"

#ifdef WIN32
#define rt_printf printf
#include <windows.h>
#undef CM_NONE
#endif
#ifdef UNIX
#include "rtdk.h"
#include "unistd.h"
#endif

auto drawLoveParse(const std::string &cmd, const std::map<std::string, std::string> &params, aris::core::Msg &msg)->void
{
    dlParam param;

    for (auto &i : params)
    {
        if (i.first == "totalCount")
        {
            param.totalCount = std::stoi(i.second);
        }
        else if (i.first == "r")
        {
            param.r = std::stod(i.second);
        }
        else if (i.first == "alpha")
        {
            param.alpha = std::stod(i.second);
        }
        else if (i.first == "beta")
        {
            param.beta = std::stod(i.second);
        }
        else if (i.first == "bodyPitch")
        {
            param.bodyPitch = std::stod(i.second);
        }
    }

    msg.copyStruct(param);
}

auto drawLoveGait(aris::dynamic::Model &model, const aris::dynamic::PlanParamBase &param_in)->int
{
    auto &robot = static_cast<Robots::RobotBase &>(model);
    auto &param = static_cast<const dlParam &>(param_in);

    //初始化
    static aris::dynamic::FloatMarker beginMak{ robot.ground() };
    static double beginPee[18];
    if (param.count == 0)
    {
        beginMak.setPrtPm(*robot.body().pm());
        beginMak.update();
        robot.GetPee(beginPee, beginMak);
    }

    double Peb[6], Pee[18];
    std::fill(Peb, Peb + 6, 0);
	std::copy(beginPee, beginPee + 18, Pee);

	const double s = -0.5 * cos(PI * (param.count%param.totalCount + 1) / param.totalCount) + 0.5; //s从0到1. 

    const double r = param.r;
    const double a = param.alpha;
    const double b = param.beta;
    const double l1 = r*(std::cos(a) + std::cos(b));
    const double l = l1 / std::sin(a);
    const double yc = l*std::cos(a) + r*std::sin(a);
    const double xc = r*std::cos(b);

    Peb[1] = param.bodyUp*s;
    Peb[2] = param.bodyBack*s;
    Peb[4] = param.bodyPitch*s;

    if (param.count < param.totalCount)
    {
        Pee[0] -= l*std::sin(a)*s;
        Pee[1] += l*std::cos(a)*s;
        Pee[9] += l*std::sin(a)*s;
        Pee[10] += l*std::cos(a)*s;
        Peb[1] = param.bodyUp*s;
        Peb[2] = param.bodyBack*s;
        Peb[4] = param.bodyPitch*s;
    }
    else
    {
        double theta = -a*(1 - s) + (PI - b)*s;
        Pee[0] -= l*std::sin(a) + r*(std::cos(theta) - std::cos(a));
        Pee[1] += l*std::cos(a) + r*(std::sin(theta) + std::sin(a));
        Pee[9] += l*std::sin(a) + r*(std::cos(theta) - std::cos(a));
        Pee[10] += l*std::cos(a) + r*(std::sin(theta) + std::sin(a));
        Peb[1] = param.bodyUp;
        Peb[2] = param.bodyBack;
        Peb[4] = param.bodyPitch;
    }

    robot.SetPeb(Peb, beginMak);
    robot.SetPee(Pee, beginMak);

    return 2 * param.totalCount - param.count - 1;
}
