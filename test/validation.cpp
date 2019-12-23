#include "validation.h"
#ifdef WIN32
#define rt_printf printf
#include <windows.h>
#undef CM_NONE
#endif
#ifdef UNIX
#include "rtdk.h"
#include "unistd.h"
#endif

auto validationParse(const std::string &cmd, const std::map<std::string, std::string> &params, aris::core::Msg &msg)->void
{
    valiParam param;

    for (auto &i : params)
    {
        if (i.first == "totalCount")
        {
            param.totalCount = std::stoi(i.second);
        }
        else if (i.first == "waitingCount")
        {
            param.waitingCount = std::stoi(i.second);
        }
        else if (i.first == "leg")
        {
            param.leg_id = stoi(i.second);
            if (param.leg_id < 0 || param.leg_id > 5)
                throw std::runtime_error("invalid leg_id");
            std::fill_n(param.active_motor, 18, false);
            std::fill_n(param.active_motor + param.leg_id * 3, 3, true);
        }
        //else if (i.first == "next")
        //{
        //    CaliCmd::getCmd().cmd() = Command::NEXT;
        //}
        //else if (i.first == "previous")
        //{
        //    CaliCmd::getCmd().cmd() = Command::PREVIOUS;
        //}
        //else if (i.first == "quit")
        //{
        //    CaliCmd::getCmd().cmd() = Command::QUIT;
        //}
    }
    msg.copyStruct(param);
}

auto validationGait(aris::dynamic::Model &model, const aris::dynamic::PlanParamBase &param_in)->int
{
    auto &robot = static_cast<Robots::RobotBase &>(model);
    auto &param = static_cast<const valiParam &>(param_in);

    const double deltaPee[][3] = {
        { 0,    0,     0.1 },
        { 0,    0,     0.2 },
        { 0,    0,     0.3 },
        { 0,    0.05,  0.3 },
        { 0,    0.05,  0.2 },
        { 0,    0.05,  0.1 },
        { 0,    0.05,  0   },
        { 0,    0.05, -0.1 },
        { 0,    0.05, -0.2 },
        { 0,    0.05, -0.3 },
        { 0,    0,    -0.3 },
        { 0,    0,    -0.2 },
        { 0,    0,    -0.1 },
        { 0,    0,     0   },
        { 0.1,  0,     0   },
        { 0.2,  0,     0   },
        { 0.3,  0,     0   },
        { 0.3,  0.05,  0   },
        { 0.2,  0.05,  0   },
        { 0.1,  0.05,  0   },
        { 0,    0.05,  0   },
        {-0.1,  0.05,  0   },
        {-0.2,  0.05,  0   },
        {-0.3,  0.05,  0   },
        {-0.3,  0,     0   },
        {-0.2,  0,     0   },
        {-0.1,  0,     0   },
        { 0,    0,     0   } };
    int n_target = sizeof(deltaPee) / sizeof(deltaPee[0]);

    //初始化
    static aris::dynamic::FloatMarker beginMak{ robot.ground() };
    static double beginPee[18];
    static double lastPee[3];
    static bool isRunning;
    static int target_id;
    static std::int32_t begin_count;

    if (param.count == 0)
    {
        beginMak.setPrtPm(*robot.body().pm());
        beginMak.update();
        robot.GetPee(beginPee, beginMak);
        robot.pLegs[param.leg_id]->GetPee(lastPee, beginMak);
        isRunning = true;
        target_id = 0;
        begin_count = 0;

        rt_printf("Initial pee of leg %d is: %f %f %f\n", param.leg_id,
            lastPee[0], lastPee[1], lastPee[2]);
    }

    double Peb[6], Pee[18];
    std::fill(Peb, Peb + 6, 0);
    std::copy(beginPee, beginPee + 18, Pee);
    robot.SetPeb(Peb);

    if (isRunning)
    {
        std::int32_t period_count = param.count - begin_count;
        const double s = -PI / 2 * cos(PI * (period_count + 1) / param.totalCount) + PI / 2;//s 从0到PI.
        double legPee[3];
        double targetPee[3];

        for (int i = 0; i < 3; ++i)
        {
            targetPee[i] = deltaPee[target_id][i] + beginPee[param.leg_id * 3 + i];
            legPee[i] = lastPee[i] * (1 + cos(s)) / 2 + targetPee[i] * (1 - cos(s)) / 2;
        }

        robot.pLegs[param.leg_id]->SetPee(legPee, beginMak);

        if (period_count + 1 == param.totalCount)
        {
            isRunning = false;
            //CaliCmd::getCmd().cmd() = Command::NONE;
            std::copy_n(targetPee, 3, lastPee);
            rt_printf("Validation Point No. %d\n", target_id);
            double testPee[3];
            robot.pLegs[param.leg_id]->GetPee(testPee, beginMak);
            rt_printf("testPee: %f %f %f\n", testPee[0], testPee[1], testPee[2]);
        }
    }
    else
    {
        std::int32_t period_count = param.count - begin_count;
        if (period_count + 1 == param.totalCount + param.waitingCount)
        {
            isRunning = true;
            target_id++;
            begin_count = param.count + 1;
        }
    }

    return n_target - target_id;
}
