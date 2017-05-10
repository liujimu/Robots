#include "creeping_gait.h"

#ifdef WIN32
#define rt_printf printf
#include <windows.h>
#undef CM_NONE
#endif
#ifdef UNIX
#include "rtdk.h"
#include "unistd.h"
#endif

auto creepingParse(const std::string &cmd, const std::map<std::string, std::string> &params, aris::core::Msg &msg)->void
{
	creepingParam param;

	for (auto &i : params)
	{
		if (i.first == "totalCount")
		{
			param.totalCount = std::stoi(i.second);
		}
		else if (i.first == "d")
		{
			param.d = std::stod(i.second);
		}
		else if (i.first == "h")
		{
			param.h = std::stod(i.second);
		}
	}

	CmdState::getState().isStoppingCmd() = false;

	msg.copyStruct(param);
}

auto creepingGait(aris::dynamic::Model &model, const aris::dynamic::PlanParamBase &param_in)->int
{
	auto &robot = static_cast<Robots::RobotTypeIII &>(model);
	auto &param = static_cast<const creepingParam &>(param_in);

	//模拟地图数据
	double gridMap[120][600]{ 0 };
	for (int i = 0; i < 120; ++i)
	{
		for (int j = 0; j < 150; ++j)
		{
			gridMap[i][j] = -0.85;
		}
		for (int j = 150; j < 450; ++j)
		{
			gridMap[i][j] = -0.85 + 0.01 * (j - 150) * std::tan(PI * 35 / 180);
		}
		for (int j = 450; j < 600; ++j)
		{
			gridMap[i][j] = -0.85 + 3 * std::tan(PI * 35 / 180);
		}
	}
	static double curretMap[120][300];
	static int mapOffset{ 0 };
	static double heightOffset{ 0 };

	//初始化
	static aris::dynamic::FloatMarker beginMak{ robot.ground() };
	static double beginPee[18]{ 0 };
	static double beginPeb[6]{ 0 };
	static double targetPee[18]{ 0 };
	static double targetPeb[6]{ 0 };
	static double targetWa{ 0 };
	static bool isLiftingLeg[6]{ false };

	if (param.count == 0)
	{
		std::fill_n(beginPee, 18, 0);
		std::fill_n(beginPeb, 6, 0);
		std::fill_n(targetPee, 18, 0);
		std::fill_n(targetPeb, 6, 0);
		targetWa = 0;
		mapOffset = 0;
		heightOffset = 0;
	}

	double step_d;
	if (param.count / param.totalCount == 0 || param.count / param.totalCount == param.n * 4)
	{
		step_d = param.d / 2;
	}
	else
	{
		step_d = param.d;
	}

	std::int32_t count = param.count % (4 * param.totalCount);
	if (count == 0)
	{
		beginMak.setPrtPm(*robot.body().pm());
		beginMak.update();

		//update map
		mapOffset += int(-targetPeb[2] / 0.01);
		heightOffset += targetPeb[1];

		if (param.count == 0)
		{
			mapOffset = 0;
			heightOffset = 0;
		}

		for (int i = 0; i < 120; ++i)
		{
			for (int j = 0; j < 300; ++j)
			{
				curretMap[i][j] = gridMap[i][j + mapOffset] - heightOffset;
			}
		}
		rt_printf("mapOffset: %d\n", mapOffset);
		rt_printf("heightOffset: %f\n", heightOffset);
	}
	if (count % param.totalCount == 0)
	{
		rt_printf("\ncount: %d\n\n", param.count);

		robot.GetPee(beginPee, beginMak);
		robot.GetPeb(beginPeb, beginMak);
		std::copy_n(beginPee, 18, targetPee);
		std::copy_n(beginPeb, 6, targetPeb);
		const int LegSeq[4]{ 1, 0, -1, 2 };
		int k = LegSeq[count / param.totalCount];
		std::fill_n(isLiftingLeg, 6, false);
		if (k >= 0)
		{
			for (int i = 0; i < 2; ++i)
			{
				isLiftingLeg[k + 3 * i] = true;
				double tmpPee[3];
				std::copy_n(beginPee + 3 * k + 9 * i, 3, tmpPee);
				double x0 = tmpPee[0];
				double z0 = tmpPee[2] - step_d;
				int m = int((0.6 + x0) / 0.01);
				int n = int((0.701 - z0) / 0.01);
				double y0 = curretMap[m][n];

				rt_printf("tmpPee[0]: %f\n", tmpPee[0]);
				rt_printf("tmpPee[2]: %f\n", tmpPee[2]);
				rt_printf("z0: %f\n", z0);
				rt_printf("m: %d\n", m);
				rt_printf("n: %d\n", n);
				rt_printf("y0: %f\n", y0);

				double dy0 = y0 - tmpPee[1];
				double alpha0 = std::atan(dy0 / step_d);
				double d = step_d * std::cos(alpha0);
				double z = tmpPee[2] - d;
				n = int((0.701 - z) / 0.01);
				double y = curretMap[m][n];
				targetPee[3 * k + 9 * i] = x0;
				targetPee[3 * k + 9 * i + 1] = y;
				targetPee[3 * k + 9 * i + 2] = z;
				//for test
				rt_printf("k: %d\n", k);
				rt_printf("step_d: %f\n", step_d);
				rt_printf("dy0: %f\n", dy0);
				rt_printf("alpha0: %f\n", alpha0);
				rt_printf("y: %f\n", y);
			}
		}
		double pee_upward{ 0 };
		double pee_forward{ 0 };
		for (int i = 0; i < 6; ++i)
		{
			pee_upward += (targetPee[3 * i + 1] - beginPee[3 * i + 1]);
			pee_forward += targetPee[3 * i + 2];
		}
		double dy = ((targetPee[1] - targetPee[7]) + (targetPee[10] - targetPee[16])) / 2;
		double dz = -((targetPee[2] - targetPee[8]) + (targetPee[11] - targetPee[17])) / 2;
		targetPeb[1] = beginPeb[1] + pee_upward / 6;
		if (k == 0)
		{
			targetPeb[2] = (targetPee[5] + targetPee[14]) / 2 + param.d / 10;
			targetWa = 1.5 * std::atan(dy / dz);
		}
		else if (k == -1)
		{
			targetPeb[2] = (targetPee[5] + targetPee[14]) / 2 - param.d / 10;
			targetWa = 1.5 * std::atan(dy / dz);
		}
		else if (k == 2)
		{
			targetPeb[2] = (targetPee[2] + targetPee[5] + targetPee[14] + targetPee[17]) / 4;
			targetWa = 1.2 * std::atan(dy / dz);
		}
		else
		{
			targetPeb[2] = pee_forward / 6;
			targetWa = 1.5 * std::atan(dy / dz);
		}
	}

	basicParam basic_param;
	basic_param.count = count % param.totalCount;
	basic_param.beginMak = &beginMak;
	std::copy_n(isLiftingLeg, 6, basic_param.lifting_leg);
	std::copy_n(targetPeb, 6, basic_param.targetPeb);
	std::copy_n(targetPee, 18, basic_param.targetPee);
	basic_param.targetWa = targetWa;

	int ret = basicGait(robot, basic_param);

	//行程保护
	double pin[18];
	robot.GetPin(pin);
	const double p1_max = 1.067;
	const double p1_min = 0.639;
	const double p23_max = 1.09;
	const double p23_min = 0.69;
	bool out_of_limit = false;
	for (int i = 0; i < 18; ++i)
	{
		double max, min;
		if (i % 3 == 0)
		{
			max = p1_max;
			min = p1_min;
		}
		else
		{
			max = p23_max;
			min = p23_min;
		}
		if (pin[i] > max)
		{
			rt_printf("pin[%d] = %f, longer than limit.\n", i, pin[i]);
			out_of_limit = true;
		}
		else if (pin[i] < min)
		{
			rt_printf("pin[%d] = %f, shorter than limit.\n", i, pin[i]);
			out_of_limit = true;
		}
	}
	if (out_of_limit)
	{
		rt_printf("Pin out of limit at count: %d\n", param.count);
		return 0;
	}

	return (4 * param.n + 1) * param.totalCount - param.count - 1;
}

auto forceCreepingGait(aris::dynamic::Model & model, const aris::dynamic::PlanParamBase & param_in) -> int
{
	auto &robot = static_cast<Robots::RobotTypeIII &>(model);
	auto &param = static_cast<const creepingParam &>(param_in);

	static aris::dynamic::FloatMarker beginMak{ robot.ground() };
	static double beginPee[18];
	static double beginWa;

	//力传感器
	static double forceOffsetSum[36]{ 0 };
	double forceOffsetAvg[36]{ 0 };
	double realForceData[36]{ 0 };

	double forceInFoot[6]{ 0 };
	const double ForceThreshold[6]{ -100, -100, -100, -100, -100, -100 }; //力信号的触发阈值,单位N或Nm
	const int Leg2Sensor[6]{ 0, 1, 2, 3, 4, 5 }; //力传感器与腿的顺序映射
	const double ForceJudgeCount{ 500 };
	for (int i = 0; i < 6; i++)
	{
		forceInFoot[i] = param.force_data->at(Leg2Sensor[i]).Fz;
	}

	//触地判断
	static bool isTouchingGround[6];

	//迈步规划
	const int LegSeq[2][4]{ { 1, 0, -1, 2 },{ 1, 2, -1, 0 } }; //LegSeq[0]为向前迈步的次序，LegSeq[1]为向后迈步的次序，-1表示六条腿都不迈
	static int step{ 0 }; //总步数
	static double slope[3]{ 0 };//预计坡度
	static double prePee[18];
	static double contactPee[18];
	static double targetPee[18];
	static double targetPeb[6];

	static std::int32_t beginCount{ 0 };
	static bool isStepFinished{ false };
	static bool isProcessFinished{ false };
	static bool isLastStep{ false };
	
	double Pee[18];
	double Peb[6];
	double Wa;
		//初始化
	if (param.count == 0)
	{
		isLastStep = false;
	}

	int swingLegID{ -1 };
	if (param.d > 0)
	{
		swingLegID = LegSeq[0][step];
	}
	else
	{
		swingLegID = LegSeq[1][step];
	}

	//单步规划
	int period_count = param.count - beginCount;
	if (period_count == 0)
	{
		beginMak.setPrtPm(*robot.body().pm());
		beginMak.update();
		robot.GetPee(beginPee, beginMak);
		robot.GetWa(beginWa);

	}

	//椭圆轨迹
	const double s = -0.5 * std::cos(PI * (period_count + 1) / param.totalCount) + 0.5; //s从0到1
	//足尖插值


	double h = param.h;
	std::copy(beginPee, beginPee + 18, Pee);
	for (int i = swingLegID; i < 6; i+=3)
	{
		if (isTouchingGround[i])
		{
			std::copy_n(contactPee + 3 * i, 3, Pee + 3 * i);
		}
		else
		{
			double dx = targetPee[3 * i] - beginPee[3 * i];
			double dy = targetPee[3 * i + 1] - beginPee[3 * i + 1];
			double dz = targetPee[3 * i + 2] - beginPee[3 * i + 2];
			double alpha = std::atan(-dy / dz);
			double d = dz / std::cos(alpha);
			double y0 = h * std::sin(PI * s);
			double z0 = d / 2 * (1 - std::cos(PI * s));
			Pee[3 * i] += dx * s;
			Pee[3 * i + 1] += std::cos(alpha) * y0 - std::sin(alpha) * z0;
			Pee[3 * i + 2] += std::sin(alpha) * y0 + std::cos(alpha) * z0;
		}
	}
	//身体插值
	for (int i = 0; i < 3; ++i)
	{
		Peb[i] = targetPeb[i] * s;
	}
	//腰关节插值
	double targetWa;
	Wa = beginWa * (1 - s) + targetWa * s;

	if (period_count == param.totalCount - 1)
	{
		isStepFinished = true;
	}

	//触地检测
	if (period_count > ForceJudgeCount - 1)
	{
		for (int i = swingLegID; i < 6; i += 3)
		{
			if (forceInFoot[i] < ForceThreshold[i])
			{
				isTouchingGround[i] = true;
				std::copy_n(Pee + 3 * i, 3, contactPee + 3 * i);
			}
		}
		if (isTouchingGround[swingLegID] && isTouchingGround[swingLegID + 3])
		{
			isStepFinished = true;
		}
	}

	if (isStepFinished)
	{
		beginCount += period_count + 1;
		++step;
		if (step == 4)
		{
			isProcessFinished = true;
			step -= 4;
		}
	}

	if (isProcessFinished && CmdState::getState().isStoppingCmd())
	{
		isLastStep = true;
	}

	if (isLastStep && period_count == param.totalCount - 1)
	{
		return 0;
	}
	else
	{
		return 1;
	}
}

auto basicGait(aris::dynamic::Model &model, const aris::dynamic::PlanParamBase &param_in)->int
{
	auto &robot = static_cast<Robots::RobotTypeIII &>(model);
	auto &param = static_cast<const basicParam &>(param_in);

	//初始化
	//static aris::dynamic::FloatMarker beginMak{ robot.ground() };
	static double beginPee[18]{ 0 };
	static double beginPeb[6]{ 0 };
	static double beginWa{ 0 };
	if (param.count == 0)
	{
		//beginMak.setPrtPm(*robot.body().pm());
		//beginMak.update();
		robot.GetPee(beginPee, *param.beginMak);
		robot.GetPeb(beginPeb, *param.beginMak);
		robot.GetWa(beginWa);

		//for test
		rt_printf("lifting_leg: %d %d %d %d %d %d\n", param.lifting_leg[0], param.lifting_leg[1],
			param.lifting_leg[2], param.lifting_leg[3], param.lifting_leg[4], param.lifting_leg[5]);
		rt_printf("Get beginWa: %f\n", beginWa);
		rt_printf("beginPeb: %f %f %f %f %f %f\n",
			beginPeb[0], beginPeb[1], beginPeb[2], beginPeb[3], beginPeb[4], beginPeb[5]);
		rt_printf("beginPee: \n%f %f %f %f %f %f %f %f %f \n%f %f %f %f %f %f %f %f %f\n\n",
			beginPee[0], beginPee[1], beginPee[2], beginPee[3], beginPee[4], beginPee[5],
			beginPee[6], beginPee[7], beginPee[8], beginPee[9], beginPee[10], beginPee[11],
			beginPee[12], beginPee[13], beginPee[14], beginPee[15], beginPee[16], beginPee[17]);

		rt_printf("targetWa: %f\n", param.targetWa);
		rt_printf("targetPeb: %f %f %f %f %f %f\n",
			param.targetPeb[0], param.targetPeb[1], param.targetPeb[2], param.targetPeb[3], param.targetPeb[4], param.targetPeb[5]);
		rt_printf("targetPee: \n%f %f %f %f %f %f %f %f %f \n%f %f %f %f %f %f %f %f %f\n\n",
			param.targetPee[0], param.targetPee[1], param.targetPee[2], param.targetPee[3], param.targetPee[4], param.targetPee[5],
			param.targetPee[6], param.targetPee[7], param.targetPee[8], param.targetPee[9], param.targetPee[10], param.targetPee[11],
			param.targetPee[12], param.targetPee[13], param.targetPee[14], param.targetPee[15], param.targetPee[16], param.targetPee[17]);
	}

	double Peb[6]{ 0 }, Pee[18]{ 0 };
	double Wa{ 0 };
	const double s = -0.5 * std::cos(PI * (param.count + 1) / param.totalCount) + 0.5; //s从0到1
	//身体插值
	for (int i = 0; i < 3; ++i)
	{
		Peb[i] = param.targetPeb[i] * s + beginPeb[i] * (1 - s);
	}
	//足尖插值
	double h = param.h;
	std::copy(beginPee, beginPee + 18, Pee);
	for (int i = 0; i < 6; ++i)
	{
		if (param.lifting_leg[i])
		{
			double dx = param.targetPee[3 * i] - beginPee[3 * i];
			double dy = param.targetPee[3 * i + 1] - beginPee[3 * i + 1];
			double dz = param.targetPee[3 * i + 2] - beginPee[3 * i + 2];
			double alpha = std::atan(-dy / dz);
			double d = dz / std::cos(alpha);
			double y0 = h * std::sin(PI * s);
			double z0 = d / 2 * (1 - std::cos(PI * s));
			Pee[3 * i] += dx * s;
			Pee[3 * i + 1] += std::cos(alpha) * y0 - std::sin(alpha) * z0;
			Pee[3 * i + 2] += std::sin(alpha) * y0 + std::cos(alpha) * z0;
		}
	}
	//腰关节插值
	Wa = beginWa * (1 - s) + param.targetWa * s;
	//test
	//if (param.count % 100 == 0)
	//{
	//	rt_printf("count: %d\n", param.count);
	//	rt_printf("SetWa: %f\n", Wa);
	//}

	robot.SetPeb(Peb, *param.beginMak);
	robot.SetWa(Wa);
	robot.SetPee(Pee, *param.beginMak);

	return param.totalCount - param.count - 1;
}