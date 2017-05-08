#include <Eigen/Eigen>

#include <aris.h>
#include <Robot_Type_III.h>
#include <Basic_Gait.h>
#include "creeping_gait.h"
#include "cross_chasm.h"
#include "move_single_leg.h"

Robots::RobotTypeIII rbt;

int main_test(int argc, char *argv[]);
int main(int argc, char *argv[])
{
	try
	{
		main_test(argc, argv);
	}
	catch (std::exception &e)
	{
		std::cout << e.what() << std::endl;
	}


	std::cout << "finished" << std::endl;

	char aaa;
	std::cin >> aaa;
	return 0;
}

int main_test(int argc, char *argv[])
{
	
#ifdef WIN32
	rbt.loadXml("G:\\Hexapod\\Robots_LJM\\src\\Robot_Type_III\\resource\\Robot_IX\\Robot_IX.xml");
	//rbt.loadXml("C:\\Robots\\resource\\Robot_Type_I\\Robot_VIII\\Robot_VIII.xml");
#endif
#ifdef UNIX
	rbt.loadXml("/usr/Robots/resource/Robot_Type_I/Robot_III/Robot_III.xml");
#endif
	
	double beginEE[]{
		-0.3, -0.95, -0.65,
		-0.45, -0.95, 0,
		-0.3, -0.95, 0.65,
		0.3, -0.95, -0.65,
		0.45, -0.95, 0,
		0.3, -0.95, 0.65 };

	double beginPE[6]{ 0 };
	double beginWa{ 0 };

	//ÅÀÆÂ²âÊÔ
	//double phi = PI / 9; //²àÆÂ½Ç¶È
	//for (int i = 0; i < 6; ++i)
	//{
	//	beginEE[3 * i + 1] += beginEE[3 * i] * std::tan(phi);
	//}
	//beginPE[3] = phi;
	
	Robots::Gait::WalkParam wk_param;
	wk_param.totalCount = 2000;
	wk_param.n = 1;
	wk_param.beta = 0;
	wk_param.alpha = 0;
	wk_param.h = 0.1;
	wk_param.d = 1.3;

	Robots::Gait::AdjustWaistParam aw_param;
	aw_param.angle = 0;

	creepingParam cp_param;
	cp_param.n = 3;
	cp_param.d = 0.4;

	ccParam cc_param;
	cc_param.chasmWidth = 0.8;
	cc_param.isBackward = true;
	cc_param.stepHeight = 0.1;
	cc_param.totalCount = 2000;

	MslParam msl_param;
	msl_param.legID = 0;
	msl_param.displacement[0] = 0.05;
	msl_param.displacement[1] = 0.1;
	msl_param.displacement[2] = -0.15;
	msl_param.isAbsolute = false;
	msl_param.totalCount = 2000;

	rbt.SetPeb(beginPE);
	rbt.SetWa(beginWa);
	rbt.SetPee(beginEE);
	
	auto result = rbt.simToAdams("G:\\Models\\Adams\\RobotIX", Robots::Gait::walkGait, wk_param, 50);

	result.saveToTxt("G:\\Models\\Adams\\RobotIX\\test");

	//rbt.saveXml("G:\\Hexapod\\Robots_LJM_build\\simAdams\\test.xml");
	
	return 0;
}