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

    //��ʼ��
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

    double state1Pee[18]; //�����м���
    std::copy(beginPee, beginPee + 18, state1Pee);
    state1Pee[3] += param.footOffset;
    state1Pee[12] -= param.footOffset;
    state1Pee[5] -= param.footOffset;
    state1Pee[14] -= param.footOffset;

    double state2Pee[18]; //����ǰ��
    std::copy(state1Pee, state1Pee + 18, state2Pee);
    for (int i = 0; i < 6; i += 3)
    {
        state2Pee[3 * i + 1] = param.bodyUp - footDist * std::cos(2 * param.bodyPitch + theta);
        state2Pee[3 * i + 2] = -footDist * std::sin(2 * param.bodyPitch + theta) + targetPeb[2];
    }
    static double lastStepPee[18]; //��һ������ʱ���λ��
    static double lastStepPeb[6];
    if (param.count == 0)
    {
        std::copy(state2Pee, state2Pee + 18, lastStepPee);
        std::copy(targetPeb, targetPeb + 6, lastStepPeb);
    }

    //��һ�������м���
    if (param.count < param.totalCount)
    {
        const double s = -PI / 2 * std::cos(PI * (param.count + 1) / param.totalCount) + PI / 2; //s��0��PI.
        std::copy(beginPee, beginPee + 18, Pee);
        for (int i = 1; i < 6; i += 3)
        {
            Pee[3 * i] -= std::pow(-1, i) * param.footOffset * (1 - std::cos(s)) / 2;
            Pee[3 * i + 1] += param.stepHeight * std::sin(s);
            Pee[3 * i + 2] -= param.footOffset * (1 - std::cos(s)) / 2;
        }
    }

    //�ڶ�����̧ͷ��̧ǰ��
    else if (param.count < 2 * param.totalCount)
    {
        const double s = -0.5 * std::cos(PI * (param.count + 1 - param.totalCount) / param.totalCount) + 0.5; //s��0��1.
        for (int i = 0; i < 6; ++i)
        {
            Peb[i] = targetPeb[i] * s;
        }
        std::copy(state1Pee, state1Pee + 18, Pee);
        for (int i = 0; i < 6; i += 3)
        {
            Pee[3 * i + 1] = state1Pee[3 * i + 1] * (1 - s) + state2Pee[3 * i + 1] * s;
            Pee[3 * i + 2] = state1Pee[3 * i + 2] * (1 - s) + state2Pee[3 * i + 2] * s;
        }
    }

    //������������
    else if (param.count < (2 + 4 * n) * param.totalCount)
    {
        const int stepOrder[4]{ 2,1,5,4 };
        const int step_id = (param.count / param.totalCount - 2) % 4;
        int leg_id = stepOrder[step_id]; //ȷ����ǰ�������
        int period_count = param.count % param.totalCount;
        const double s = -(PI / 2) * cos(PI * (period_count + 1) / param.totalCount) + PI / 2;//s ��0��PI. 

        //�滮����λ��
        std::copy(lastStepPeb, lastStepPeb + 6, Peb);
        Peb[2] -= step_id % 2 * param.stepLength / 2 * (1 - std::cos(s)) / 2;


        //�滮���λ��
        std::copy(lastStepPee, lastStepPee + 18, Pee);
        for (int i = 0; i < 6; i += 3)
        {
            Pee[3 * i + 2] -= step_id % 2 * param.stepLength / 2 * (1 - std::cos(s)) / 2;
        }

        Pee[3 * leg_id + 1] += param.stepHeight * std::sin(s);
        Pee[3 * leg_id + 2] -= param.stepLength * (1 - std::cos(s)) / 2;

        if (period_count == param.totalCount - 1)
        {
            lastStepPee[3 * leg_id + 2] -= param.stepLength;
            lastStepPeb[2] -= param.stepLength / 4;
            rt_printf("count: %d\n", param.count);
            rt_printf("lastStepPee: %f %f %f %f %f %f\n",
                lastStepPee[2], lastStepPee[5], lastStepPee[8],
                lastStepPee[11], lastStepPee[14], lastStepPee[17]);

        }
    }
/*
    //���Ĳ�����ǰ�ȣ��ָ�������̬
    else if (param.count < (3 + 4 * n) * param.totalCount)
    {
        const double s = -0.5 * std::cos(PI * (param.count + 1 - (2 + n) * param.totalCount) / param.totalCount) + 0.5; //s��0��1.
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

    //���岽�����м���
    else
    {
        const double s = -PI / 2 * std::cos(PI * (param.count + 1 - (3 + n) * param.totalCount) / param.totalCount) + PI / 2; //s��0��PI.
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

    return (2 + 4 * n) * param.totalCount - param.count - 1;
    //return 2 * param.totalCount - param.count - 1;
}