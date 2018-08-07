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
            param.total_count = std::stoi(i.second);
        }
        else if (i.first == "bodyUp")
        {
            param.body_up = std::stod(i.second);
        }
        else if (i.first == "bodyPitch")
        {
            param.body_pitch = std::stod(i.second) / 180 * PI;
        }
        else if (i.first == "stepLength")
        {
            param.walk_step_length = std::stod(i.second);
        }
        else if (i.first == "stepHeight")
        {
            param.walk_step_height = std::stod(i.second);
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

    //初始化静态变量
    static aris::dynamic::FloatMarker beginMak{ robot.ground() };
    static double beginPee[18];
    static double lastStepPeb[6]; //上一步结束时身体位姿
    static double lastStepPee[18]; //上一步结束时足尖位置
    if (param.count == 0)
    {
        beginMak.setPrtPm(*robot.body().pm());
        beginMak.update();
        robot.GetPee(beginPee, beginMak);
        std::fill(lastStepPeb, lastStepPeb + 6, 0);
        std::copy(beginPee, beginPee + 18, lastStepPee);
    }

    int total_count = param.total_count;
    int n = param.n;
    int step_id = param.count / total_count; //step_id从0到7
    int period_count = param.count % total_count;
    double s = -(PI / 2) * cos(PI * (period_count + 1) / total_count) + PI / 2; //s从0到PI. 

    double Peb[6], Pee[18];
    std::copy(lastStepPeb, lastStepPeb + 6, Peb);
    std::copy(lastStepPee, lastStepPee + 18, Pee);

    //double footDist = std::sqrt(std::pow(beginPee[2], 2) + std::pow(beginPee[1] - param.body_up, 2));
    //double theta = std::atan2(std::fabs(beginPee[2]), std::fabs(beginPee[1]) + param.body_up);

    //double targetPeb[6];
    //std::fill(targetPeb, targetPeb + 6, 0);
    //targetPeb[1] = param.body_up;
    //targetPeb[4] = param.body_pitch;
    //targetPeb[2] = param.body_back;

    //double state1Pee[18]; //调整中间腿
    //std::copy(beginPee, beginPee + 18, state1Pee);
    //state1Pee[3] += param.middle_foot_inner_offset;
    //state1Pee[5] -= param.middle_foot_forward_offset;
    //state1Pee[12] -= param.middle_foot_inner_offset;
    //state1Pee[14] -= param.middle_foot_forward_offset;

    //double state2Pee[18]; //举起前腿
    //std::copy(state1Pee, state1Pee + 18, state2Pee);
    //for (int i = 0; i < 6; i += 3)
    //{
    //    state2Pee[3 * i] = -0.1 * pow(-1, i);
    //    state2Pee[3 * i + 1] = param.body_up - footDist * std::cos(2 * param.body_pitch + theta);
    //    state2Pee[3 * i + 2] = -footDist * std::sin(2 * param.body_pitch + theta) + targetPeb[2] + 0.1;
    //}

    //前两步：调整后腿
    if (step_id < 2)
    {
        double dx = -param.rear_foot_outer_offset * pow(-1, step_id); //step_id==0,dx<0; step_id==1,dx>0
        int leg_id = 3 * step_id + 2;
        Pee[3 * leg_id] += dx * (1 - std::cos(s)) / 2;
        Pee[3 * leg_id + 1] += param.walk_step_height * std::sin(s);
        if (period_count == total_count - 1)
        {
            lastStepPee[3 * leg_id] += dx;
        }
    }

    //第三步：迈中间腿
    else if (step_id == 2)
    {
        for (int i = 0; i < 18; i += 9)
        {
            double dx = param.middle_foot_inner_offset * pow(-1, i); //i==0,+; i==9,-
            Pee[i + 3] += dx  * (1 - std::cos(s)) / 2;
            Pee[i + 4] += param.walk_step_height * std::sin(s);
            Pee[i + 5] -= param.middle_foot_forward_offset * (1 - std::cos(s)) / 2;
        }
        if (period_count == total_count - 1)
        {
            lastStepPee[3] += param.middle_foot_inner_offset;
            lastStepPee[5] -= param.middle_foot_forward_offset;
            lastStepPee[12] -= param.middle_foot_inner_offset;
            lastStepPee[14] -= param.middle_foot_forward_offset;
        }
    }

    //第四步：前腿夹取对象
    else if (step_id == 3)
    {
        //规划前腿
        for (int i = 0; i < 18; i += 9)
        {
            double dx = param.front_foot_inner_offset * pow(-1, i); //i==0,+; i==9,-
            Pee[i] += dx *(1 - std::cos(s)) / 2;
            Pee[i + 1] += param.front_foot_upward_offset *(1 - std::cos(s)) / 2;
        }
        if (period_count == total_count - 1)
        {
            lastStepPee[0] += param.front_foot_inner_offset;
            lastStepPee[1] += param.front_foot_upward_offset;
            lastStepPee[9] -= param.front_foot_inner_offset;
            lastStepPee[10] += param.front_foot_upward_offset;
        }
    }

    //第五步：调整身体，抬前腿
    else if (step_id == 4)
    {
        //规划身体
        Peb[1] += param.body_up*(1 - std::cos(s)) / 2;
        Peb[2] += param.body_back*(1 - std::cos(s)) / 2;
        Peb[4] += param.body_pitch*(1 - std::cos(s)) / 2;
        //规划前腿
        for (int i = 0; i < 18; i += 9)
        {
            Pee[i + 1] += param.front_foot_upward_offset_2 *(1 - std::cos(s)) / 2;
            Pee[i + 2] += param.front_foot_backward_offset_2*(1 - std::cos(s)) / 2;
        }
        if (period_count == total_count - 1)
        {
            lastStepPeb[1] += param.body_up;
            lastStepPeb[2] += param.body_back;
            lastStepPeb[4] += param.body_pitch;
            lastStepPee[1] += param.front_foot_upward_offset_2;
            lastStepPee[2] += param.front_foot_backward_offset_2;
            lastStepPee[10] += param.front_foot_upward_offset_2;
            lastStepPee[11] += param.front_foot_backward_offset_2;
        }
    }

    //第六步：行走
    else if (step_id < 5 + 8 * n)
    {
        const int step_order[4]{ 2,1,5,4 };
        int walk_step_id = (step_id - 5) % 8; //walk_step_id从0到7
        int leg_id = step_order[walk_step_id / 2]; //确定当前迈腿序号        
        //规划身体位置
        if (walk_step_id % 2 == 1)
        {
            double dx, dy, dz;
            switch (walk_step_id)
            {
            case 1:
                dx = param.body_sidesway;
                dy = -param.body_down;
                dz = 0.0;
                break;
            case 3:
                dx = -param.body_sidesway;
                dy = 0;
                dz = -param.walk_step_length / 2;
                break;
            case 5:
                dx = -param.body_sidesway;
                dy = 0;
                dz = 0;
                break;
            case 6:
                dx = 0;
                dy = param.body_down;
                dz = 0;
                break;
            case 7:
                dx = param.body_sidesway;
                dy = 0;
                dz = -param.walk_step_length / 2;
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
            if (period_count == param.total_count - 1)
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
        if (walk_step_id % 2 == 0)
        {
            Pee[3 * leg_id + 1] += param.walk_step_height * std::sin(s);
            Pee[3 * leg_id + 2] -= param.walk_step_length * (1 - std::cos(s)) / 2;
            if (period_count == total_count - 1)
            {
                lastStepPee[3 * leg_id + 2] -= param.walk_step_length;
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

    return (5 + 8 * n) * param.total_count - param.count - 1;
    //return 2 * param.totalCount - param.count - 1;
}