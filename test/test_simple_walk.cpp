#include <Eigen/Eigen>

#include <aris.h>
#include <Robot_Type_I.h>
#include "move_body.h"
#include "swing.h"
#include "twist_waist.h"
#include "cross_obstacle.h"
#include "say_hello.h"
#include "climb_stairs_v2.h"
#include "OpenValve.h"

Robots::RobotTypeI rbt;

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
    rbt.loadXml("C:\\Robots\\resource\\Robot_Type_I\\Robot_XV\\Robot_XV.xml");
    //rbt.loadXml("C:\\Robots\\resource\\Robot_Type_I\\Robot_VIII\\Robot_VIII.xml");
#endif
#ifdef UNIX
    rbt.loadXml("/usr/Robots/resource/Robot_Type_I/Robot_III/Robot_III.xml");
#endif

    const double beginEE[]{
		-0.30,   -0.58,   -0.52,
		-0.60,   -0.58,    0,
		-0.30,   -0.58,    0.52,
		 0.30,   -0.58,   -0.52,
		 0.60,   -0.58,    0,
		 0.30,   -0.58,    0.52 };
    //const double offset = 0.058;
    //const double beginEE[]{
    //    -0.2828 - offset,   -0.61,   -0.35,
    //    -0.5656 - offset,   -0.61,    0,
    //    -0.2828 - offset,   -0.61,    0.35,
    //    0.2828 - offset,   -0.61,   -0.35,
    //    0.5656 - offset,   -0.61,    0,
    //    0.2828 - offset,   -0.61,    0.35 };

    double beginPE[6]{ 0 };

    /*
    //调整身体姿态和足尖位置到楼梯上
    const double angle{ PI / 180 * 45 };
    const double sa = std::sin(angle);
    const double ca = std::cos(angle);
    beginPE[3] = angle;
    double beginPee[18]{ 0 };
    for (int i = 0; i < 18; i += 3)
    {
        beginPee[i] = ca*beginEE[i] - sa*beginEE[i + 1];
        beginPee[i+1] = sa*beginEE[i] + ca*beginEE[i + 1];
        beginPee[i + 2] = beginEE[i + 2];
    }
    printf("beginPee:\n%f\t%f\t%f\n%f\t%f\t%f\n", 
        beginPee[0], beginPee[1], beginPee[2], beginPee[3], beginPee[4], beginPee[5]);
    */

    Robots::WalkParam wk_param;
    wk_param.totalCount = 1000;
    wk_param.n = 2;
    wk_param.beta = 0;
    wk_param.alpha = 0;
    wk_param.d = 0.5;
	wk_param.h = 0.05;

    mbParam mb_param;
    mb_param.totalCount = 1000;
    mb_param.x = 0.1;

    swParam sw_param;
    sw_param.totalCount = 4000;
    sw_param.zAngle = PI * 15 / 180;
    sw_param.rDistance = 0.5;
    sw_param.yDistance = -0.05;

    twParam tw_param;
    tw_param.totalCount = 6000;
    tw_param.height = 0;
    tw_param.diameter = 0.05;
    tw_param.pitchMax = PI * 15 / 180;
    tw_param.rollMax = PI * 15 / 180;

    coParam co_param;

    shParam sh_param;
    sh_param.isForward = true;

    cs2Param cs2_param;

    OpenValve ov_param;

    rbt.SetPeb(beginPE);
    rbt.SetPee(beginEE);
    //rbt.SetPee(beginPee);

	//auto result = rbt.simToAdams("D:\\Lab\\Models\\Adams\\RobotXV\\test.cmd", Robots::walkGait, wk_param, 50);
    auto result = rbt.simToAdams("D:\\Lab\\Models\\Adams\\RobotXV\\test.cmd", moveBodyGait, mb_param, 50);

	result.saveToTxt("D:\\Lab\\Models\\Adams\\RobotXV\\test");

	rbt.saveXml("D:\\Lab\\Models\\Adams\\RobotXV\\test.xml");

    /*
    Robots::WalkParam wk_param;
    wk_param.totalCount = 2000;
    wk_param.n = 3;
    wk_param.beta = 0.3;
    wk_param.d = 0.5;
    aris::dynamic::SimResult result;
    rbt.SetPeb(beginPE);
    rbt.SetPee(beginEE);
    rbt.simKin(Robots::walk, wk_param, result, true);
    result.saveToTxt("C:\\Users\\yang\\Desktop\\test");


    auto d = 0.5;

    double s[1000];

    for (int i = 0; i < 1000;++i)
    s[i]=aris::dynamic::s_interp(1000, i + 1, 0, d/2, 0, d/1000);

    aris::dynamic::dlmwrite("C:\\Users\\yang\\Desktop\\s.txt",s, 1, 1000);

    /*
    SimpleWalkParam param;
    rbt.GetPee(param.beginPee);
    rbt.GetPeb(param.beginPeb);

    rbt.SimByMatlab("C:\\Users\\yang\\Desktop\\test\\", SimpleWalk, param);

    rbt.SimScriptClear();
    rbt.SimScriptSetTopologyA();
    rbt.SimScriptSimulate(500, 10);
    rbt.SimScriptSetTopologyB();
    rbt.SimScriptSimulate(500, 10);
    rbt.SimByAdams("C:\\Users\\yang\\Desktop\\test.cmd", SimpleWalk, param, 10, true);
    rbt.SimByAdamsResultAt(101);

    double fin[18];
    rbt.GetFinDyn(fin);
    aris::dynamic::dsp(fin, 18, 1);

    {
    rbt.ClbPre();
    rbt.ClbUpd();
    rbt.ClbSetInverseMethod([](int n, double *A)
    {
    Eigen::Map<Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> >Am(A, n, n);
    auto im = Am.inverse();
    Am = im;
    });
    rbt.ForEachMotion([](aris::dynamic::Motion *p)
    {
    p->SetMotFce(p->MotFce());
    });

    aris::dynamic::Matrix D(rbt.ClbDimM(), rbt.ClbDimN());
    aris::dynamic::Matrix b(rbt.ClbDimM(), 1);
    aris::dynamic::Matrix x(rbt.ClbDimN(), 1);

    rbt.ClbMtx(D.Data(), b.Data());
    rbt.ClbUkn(x.Data());

    auto result = D*x;
    result.dsp();
    b.dsp();
    }

    */
    return 0;
}