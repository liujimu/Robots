#include "pitch.h"
#ifdef WIN32
#define rt_printf printf
#include <windows.h>
#undef CM_NONE
#endif
#ifdef UNIX
#include "rtdk.h"
#include "unistd.h"
#endif

auto pitchParse(const std::string &cmd, const std::map<std::string, std::string> &params, aris::core::Msg &msg)->void
{
    pitchParam param;

    for (auto &i : params)
    {
        if (i.first == "totalCount")
        {
            param.totalCount = std::stoi(i.second);
        }
        else if (i.first == "y")
        {
            param.y = stod(i.second);
        }
        else if (i.first == "z")
        {
            param.z = stod(i.second);
        }
        else if (i.first == "a")
        {
            param.a = stod(i.second) / 180 * PI;
        }
    }

    msg.copyStruct(param);
}

auto pitchGait(aris::dynamic::Model &model, const aris::dynamic::PlanParamBase &param_in)->int
{
    auto &robot = static_cast<Robots::RobotBase &>(model);
    auto &param = static_cast<const pitchParam &>(param_in);

    //初始化
    static aris::dynamic::FloatMarker beginMak{ robot.ground() };
    static double beginPee[18];
    static double lastPeb[6];

    if (param.count == 0)
    {
        beginMak.setPrtPm(*robot.body().pm());
        beginMak.update();
        robot.GetPee(beginPee, beginMak);
    }

    static bool isWalking{ false };
    static int beginCount{ 0 };

    double Peb[6], Pee[18];
    std::fill(Peb, Peb + 6, 0);
    std::copy(beginPee, beginPee + 18, Pee);

    const double s = -0.5 * cos(PI * (param.count + 1) / param.totalCount) + 0.5;//s 从0到1.

	Peb[0] = 0;
	Peb[1] = param.y * s;
	Peb[2] = param.z * s;
	Peb[3] = 0;
	Peb[4] = param.a * s;
	Peb[5] = 0;

	robot.SetPeb(Peb, beginMak);
	robot.SetPee(Pee, beginMak);

    if (PitchState::getState().isStopping() && (!isWalking))
    {
        return 0;
    }
    else
    {
        return 1;
    }
}
