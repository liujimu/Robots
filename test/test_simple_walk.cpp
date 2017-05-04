#include <Eigen/Eigen>

#include <aris.h>
#include <Robot_Type_I.h>
#include "move_body.h"
#include "swing.h"
#include "twist_waist.h"
#include "cross_obstacle.h"
#include "pitch.h"
#include "say_hello.h"

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
    rbt.loadXml("C:\\Robots\\resource\\Robot_Type_I\\Robot_XIII\\Robot_XIII.xml");
    //rbt.loadXml("C:\\Robots\\resource\\Robot_Type_I\\Robot_VIII\\Robot_VIII.xml");
#endif
#ifdef UNIX
    rbt.loadXml("/usr/Robots/resource/Robot_Type_I/Robot_III/Robot_III.xml");
#endif

    //const double beginEE[]{
    //   -0.5,   -0.9,   -0.60,
    //   -0.8,   -0.9,    0,
    //   -0.9,   -0.9,    0.60,
    //    0.5,   -0.9,   -0.60,
    //    0.8,   -0.9,    0,
    //    0.9,   -0.9,    0.60 };

	const double beginEE[]{
		-0.60,   -0.9,   -0.60,
		-0.80,   -0.9,    0,
		-0.60,   -0.9,    0.60,
		0.60,   -0.9,   -0.60,
		0.80,   -0.9,    0,
		0.60,   -0.9,    0.60 };

    double beginPE[6]{ 0 };

    Robots::WalkParam wk_param;
    wk_param.totalCount = 2000;
    wk_param.n = 1;
    wk_param.beta = 0;
    wk_param.alpha = 0;
    wk_param.d = -0.3;

    mbParam mb_param;
    mb_param.totalCount = 1000;
    mb_param.z = -0.12;
    mb_param.y = 0.04;
    mb_param.pitch = PI * 25 / 180;

    swParam sw_param;
    sw_param.totalCount = 4000;
    sw_param.xAngle = PI * 20 / 180;
    sw_param.rDistance = 0.6;
    sw_param.yDistance = -0.05;

    twParam tw_param;

    coParam co_param;
	co_param.totalCount = 2000;
	co_param.n = 5;
	co_param.d = -0.5;
	co_param.h = 0.28;
	co_param.y = 0.15;

	pitchParam pitch_param;

    shParam sh_param;

    rbt.SetPeb(beginPE);
    rbt.SetPee(beginEE);

	auto result = rbt.simToAdams("D:\\Lab\\Models\\Adams\\RobotXIII\\test.cmd", sayHelloGait, sh_param, 50);

	result.saveToTxt("D:\\Lab\\Models\\Adams\\RobotXIII\\test");

	rbt.saveXml("D:\\Lab\\Models\\Adams\\RobotXIII\\test.xml");

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