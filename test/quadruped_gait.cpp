#include "quadruped_gait.h"

#ifdef WIN32
#define rt_printf printf
#include <windows.h>
#undef CM_NONE
#endif
#ifdef UNIX
#include "rtdk.h"
#include "unistd.h"
#endif

auto quadrupedGaitParse(const std::string &cmd, const std::map<std::string, std::string> &params, aris::core::Msg &msg)->void
{
    qgParam param;

    for (auto &i : params)
    {
        if (i.first == "totalCount")
        {
            param.totalCount = std::stoi(i.second);
        }
        else if (i.first == "bodyUp")
        {
            param.bodyUp = std::stod(i.second);
        }
        else if (i.first == "bodyPitch")
        {
            param.bodyPitch = std::stod(i.second) / 180 * PI;
        }
        else if (i.first == "stepLength")
        {
            param.stepLength = std::stod(i.second);
        }
        else if (i.first == "stepHeight")
        {
            param.stepHeight = std::stod(i.second);
        }
        else if (i.first == "n")
        {
            param.n = std::stoi(i.second);
        }
    }

    msg.copyStruct(param);
}

auto quadrupedGait(aris::dynamic::Model &model, const aris::dynamic::PlanParamBase &param_in)->int
{
    auto &robot = static_cast<Robots::RobotBase &>(model);
    auto &param = static_cast<const qgParam &>(param_in);

    //初始化
    static aris::dynamic::FloatMarker beginMak{ robot.ground() };
    static double beginPee[18];
    if (param.count == 0)
    {
        beginMak.setPrtPm(*robot.body().pm());
        beginMak.update();
        robot.GetPee(beginPee, beginMak);
    }

    int totalCount = param.totalCount;
    int n = param.n;
    double footDist = std::sqrt(std::pow(beginPee[2], 2) + std::pow(beginPee[1] - param.bodyUp, 2));
    double theta = std::atan2(std::fabs(beginPee[2]), std::fabs(beginPee[1]) + param.bodyUp);

    double Peb[6], Pee[18];
    std::fill(Peb, Peb + 6, 0);

    double targetPeb[6];
    std::fill(targetPeb, targetPeb + 6, 0);
    targetPeb[1] = param.bodyUp;
    targetPeb[4] = param.bodyPitch;
    targetPeb[2] = param.bodyBack;

    double state1Pee[18]; //调整中间腿
    std::copy(beginPee, beginPee + 18, state1Pee);
    state1Pee[3] += param.footInnerOffset;
    state1Pee[5] -= param.footForwardOffset;
    state1Pee[12] -= param.footInnerOffset;
    state1Pee[14] -= param.footForwardOffset;

    double state2Pee[18]; //举起前腿
    std::copy(state1Pee, state1Pee + 18, state2Pee);
    for (int i = 0; i < 6; i += 3)
    {
        state2Pee[3 * i] = -0.1 * pow(-1, i);
        state2Pee[3 * i + 1] = param.bodyUp - footDist * std::cos(2 * param.bodyPitch + theta);
        state2Pee[3 * i + 2] = -footDist * std::sin(2 * param.bodyPitch + theta) + targetPeb[2] + 0.1;
    }
    static double lastStepPee[18]; //上一步结束时足尖位置
    static double lastStepPeb[6];
    if (param.count == 0)
    {
        std::copy(state2Pee, state2Pee + 18, lastStepPee);
        std::copy(targetPeb, targetPeb + 6, lastStepPeb);
    }

    //第一步：迈中间腿
    if (param.count < param.totalCount)
    {
        const double s = -PI / 2 * std::cos(PI * (param.count + 1) / param.totalCount) + PI / 2; //s从0到PI.
        std::copy(beginPee, beginPee + 18, Pee);
        for (int i = 1; i < 6; i += 3)
        {
            Pee[3 * i] -= std::pow(-1, i) * param.footInnerOffset * (1 - std::cos(s)) / 2;
            Pee[3 * i + 1] += param.stepHeight * std::sin(s);
            Pee[3 * i + 2] -= param.footForwardOffset * (1 - std::cos(s)) / 2;
        }
    }

    //第二步：抬头，抬前腿
    else if (param.count < 2 * param.totalCount)
    {
        const double s = -0.5 * std::cos(PI * (param.count + 1 - param.totalCount) / param.totalCount) + 0.5; //s从0到1.
        for (int i = 0; i < 6; ++i)
        {
            Peb[i] = targetPeb[i] * s;
        }
        std::copy(state1Pee, state1Pee + 18, Pee);
        for (int i = 0; i < 18; ++i)
        {
            Pee[i] = state1Pee[i] * (1 - s) + state2Pee[i] * s;
        }
    }

    //第三步：行走
    else if (param.count < (2 + 8 * n) * param.totalCount)
    {
        const int stepOrder[4]{ 2,1,5,4 };
        const int step_id = (param.count / param.totalCount - 2) % 8; //step_id从0到7
        int leg_id = stepOrder[step_id / 2]; //确定当前迈腿序号
        int period_count = param.count % param.totalCount;
        const double s = -(PI / 2) * cos(PI * (period_count + 1) / param.totalCount) + PI / 2; //s从0到PI. 

        std::copy(lastStepPeb, lastStepPeb + 6, Peb);
        std::copy(lastStepPee, lastStepPee + 18, Pee);
        
        //规划身体位置
        if (step_id % 2 == 1)
        {
            double dx, dy, dz;
            switch (step_id)
            {
            case 1:
                dx = 0.1;
                dy = -0.02;
                dz = 0.0;
                break;
            case 3:
                dx = -0.1;
                dy = 0;
                dz = -0.03;
                break;
            case 5:
                dx = -0.1;
                dy = 0;
                dz = 0;
                break;
            case 7:
                dx = 0.1;
                dy = 0.02;
                dz = -0.03;
                break;
            default:
                dx = 0;
                dy = 0;
                dz = 0;
                break;
            }
            Peb[0] += dx * (1 - std::cos(s)) / 2;
            Peb[1] += dy * (1 - std::cos(s)) / 2;
            Peb[2] += dz * (1 - std::cos(s)) / 2;
            //规划两条前腿
            for (int i = 0; i < 6; i += 3)
            {
                Pee[3 * i] += dx * (1 - std::cos(s)) / 2;
                Pee[3 * i + 1] += dy * (1 - std::cos(s)) / 2;
                Pee[3 * i + 2] += dz * (1 - std::cos(s)) / 2;
            }
            if (period_count == param.totalCount - 1)
            {
                lastStepPeb[0] += dx;
                lastStepPeb[1] += dy;
                lastStepPeb[2] += dz;
                for (int i = 0; i < 6; i += 3)
                {
                    lastStepPee[3 * i] += dx;
                    lastStepPee[3 * i + 1] += dy;
                    lastStepPee[3 * i + 2] += dz;
                }
            }
        }

        //规划足尖位置
        if (step_id % 2 == 0)
        {
            Pee[3 * leg_id + 1] += param.stepHeight * std::sin(s);
            Pee[3 * leg_id + 2] -= param.stepLength * (1 - std::cos(s)) / 2;
            if (period_count == param.totalCount - 1)
            {
                lastStepPee[3 * leg_id + 2] -= param.stepLength;
            }
        }
    }
/*
    //第四步：收前腿，恢复身体姿态
    else if (param.count < (3 + 4 * n) * param.totalCount)
    {
        const double s = -0.5 * std::cos(PI * (param.count + 1 - (2 + n) * param.totalCount) / param.totalCount) + 0.5; //s从0到1.
        for (int i = 0; i < 6; ++i)
        {
            Peb[i] = targetPeb[i] * (1 - s);
        }
        std::copy(state2Pee, state2Pee + 18, Pee);
        for (int i = 0; i < 6; i += 3)
        {
            Pee[3 * i + 1] = state2Pee[3 * i + 1] * (1 - s) + state1Pee[3 * i + 1] * s;
            Pee[3 * i + 2] = state2Pee[3 * i + 2] * (1 - s) + state1Pee[3 * i + 2] * s;
        }
    }

    //第五步：收中间腿
    else
    {
        const double s = -PI / 2 * std::cos(PI * (param.count + 1 - (3 + n) * param.totalCount) / param.totalCount) + PI / 2; //s从0到PI.
        std::copy(state1Pee, state1Pee + 18, Pee);
        for (int i = 1; i < 6; i += 3)
        {
            Pee[3 * i + 1] += param.stepHeight * std::sin(s);
            Pee[3 * i + 2] += param.footOffset * (1 - std::cos(s)) / 2;
        }
    }
*/

    robot.SetPeb(Peb, beginMak);
    robot.SetPee(Pee, beginMak);

    return (2 + 8 * n) * param.totalCount - param.count - 1;
    //return 2 * param.totalCount - param.count - 1;
}