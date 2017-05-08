#include "climb_stairs.h"

#ifdef WIN32
#define rt_printf printf
#include <windows.h>
#undef CM_NONE
#endif
#ifdef UNIX
#include "rtdk.h"
#include "unistd.h"
#endif

auto climbStairsParse(const std::string &cmd, const std::map<std::string, std::string> &params, aris::core::Msg &msg)->void
{
	csParam param;

	for (auto &i : params)
	{
		if (i.first == "totalCount")
		{
			param.totalCount = std::stoi(i.second);
		}
		else if (i.first == "stair_length")
		{
			param.stair_length = std::stod(i.second);
		}
		else if (i.first == "stair_height")
		{
			param.stair_height = std::stod(i.second);
		}
	}

	msg.copyStruct(param);
}

auto climbStairsGait(aris::dynamic::Model &model, const aris::dynamic::PlanParamBase &param_in)->int
{
	auto &robot = static_cast<Robots::RobotTypeIII &>(model);
	auto &param = static_cast<const csParam &>(param_in);

    //初始化
    static aris::dynamic::FloatMarker beginMak{ robot.ground() };
    static double beginPee[18];
    static double beginPeb[6];
    static double beginWa;
    static double foot_distance;
    if (param.count == 0)
    {
        beginMak.setPrtPm(*robot.body().pm());
        beginMak.update();
        robot.GetPee(beginPee, beginMak);
        robot.GetWa(beginWa);
    }

    const int period_count = param.count%param.totalCount;
    const int period_n = param.count / param.totalCount;
    const int leg_begin_id = period_n % 2;
    const double s = -PI / 2 * cos(PI * (period_count + 1) / param.totalCount) + PI / 2;//s 从0到PI. 

    double h[6];
    double d[6];
    double hb;
    double db;
    for (int i = 0; i < 3; ++i)
    {
        d[i]=i<=period_n/4?param.stair_length:
    }
    if (period_n < 2)
    {
        d[0] = param.stair_length;
        d[1] = 0.5;
        d[2] = 0.5;
        std::copy(d, d + 3, d + 3);
        h[0] = param.stair_height;
        h[1] = 0;
        h[2] = 0;
        std::copy(h, h + 3, h + 3);
    }
    else if (period_n < 4)
    {
        d[0] = param.stair_length;
        d[1] = param.stair_length;
        d[2] = 0.5;
        std::copy(d, d + 3, d + 3);
        h[0] = param.stair_height;
        h[1] = 0;
        h[2] = 0;
        std::copy(h, h + 3, h + 3);
    }

    double Peb[6], Pee[18];
    std::fill(Peb, Peb + 6, 0);
    std::copy(beginPee, beginPee + 18, Pee);
    double Wa{ 0 };
    //规划腿
    for (int i = leg_begin_id; i < 6; i += 2)
    {
        Pee[3 * i + 1] = beginPee[3 * i + 1] + h[i] * (1 - std::cos(s)) / 2 + param.stair_height / 2 * std::sin(s);
        Pee[3 * i + 2] = beginPee[3 * i + 2] - d[i] * (1 - std::cos(s)) / 2;
    }
    //规划身体位姿
    Peb[1] = beginPeb[1] + hb * (1 - std::cos(s)) / 4;
    Peb[2] = beginPeb[2] + db * (1 - std::cos(s)) / 4;


    robot.SetPeb(Peb, beginMak);
    robot.SetWa(Wa);
    robot.SetPee(Pee, beginMak);

    return 2 * param.n * param.totalCount - param.count - 1;
}