#include "calibration.h"
#ifdef WIN32
#define rt_printf printf
#include <windows.h>
#undef CM_NONE
#endif
#ifdef UNIX
#include "rtdk.h"
#include "unistd.h"
#endif

auto calibrationParse(const std::string &cmd, const std::map<std::string, std::string> &params, aris::core::Msg &msg)->void
{
    caliParam param;

    for (auto &i : params)
    {
        if (i.first == "totalCount")
        {
            param.totalCount = std::stoi(i.second);
        }
        else if (i.first == "leg_id")
        {
            param.leg_id = stoi(i.second);
        }
        else if (i.first == "next")
        {
        }
        else if (i.first == "previous")
        {
        }
        else if (i.first == "quit")
        {
        }
    }
    CaliState::getState().isStopping() = false;
    msg.copyStruct(param);
}

auto calibrationGait(aris::dynamic::Model &model, const aris::dynamic::PlanParamBase &param_in)->int
{
    auto &robot = static_cast<Robots::RobotBase &>(model);
    auto &param = static_cast<const caliParam &>(param_in);

    const double targetPee[][3]={ 
        { 0.580, -0.100, -0.100 },
        { 0.580,  0.000, -0.100 },
        { 0.580,  0.100, -0.100 },
        { 0.630, -0.100, -0.100 },
        { 0.630,  0.000, -0.100 },
        { 0.630,  0.100, -0.100 },
        { 0.680, -0.100, -0.100 },
        { 0.680,  0.000, -0.100 },
        { 0.680,  0.100, -0.100 },
        { 0.580, -0.100,  0.000 },
        { 0.580,  0.000,  0.000 },
        { 0.580,  0.100,  0.000 },
        { 0.630, -0.100,  0.000 },
        { 0.630,  0.000,  0.000 },
        { 0.630,  0.100,  0.000 },
        { 0.680, -0.100,  0.000 },
        { 0.680,  0.000,  0.000 },
        { 0.680,  0.100,  0.000 },
        { 0.580, -0.100,  0.100 },
        { 0.580,  0.000,  0.100 },
        { 0.580,  0.100,  0.100 },
        { 0.630, -0.100,  0.100 },
        { 0.630,  0.000,  0.100 },
        { 0.630,  0.100,  0.100 },
        { 0.680, -0.100,  0.100 },
        { 0.680,  0.000,  0.100 },
        { 0.680,  0.100,  0.100 } };


    //初始化
    static aris::dynamic::FloatMarker beginMak{ robot.ground() };
    static double beginPee[18];
    static int status = 0;

    if (param.count == 0)
    {
        beginMak.setPrtPm(*robot.body().pm());
        beginMak.update();
        robot.GetPee(beginPee, beginMak);
    }

    double Peb[6], Pee[18];
    std::fill(Peb, Peb + 6, 0);
    std::copy(beginPee, beginPee + 18, Pee);

    const double s = -PI * cos(PI * (param.count + 1) / param.totalCount) + PI;//s 从0到2*PI.

    return param.totalCount - param.count - 1;
}
