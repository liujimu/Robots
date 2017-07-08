#include "OpenValve.h"

#ifdef UNIX
#include "rtdk.h"
#endif
#ifdef WIN32
#define rt_printf printf
#endif

#include <cstring>
#include <cmath>
#include <algorithm>
#include <memory>

void parseMoveWithOpenValve(const std::string &cmd, const std::map<std::string, std::string> &params, aris::core::Msg &msg)
{
     OpenValve param;// You set param in the shape of Moveupstairs
 
	for(auto &i:params)
    {
        if (i.first == "distance1")
        {
        param.d1 = stod(i.second);
        }
        else if (i.first == "distance2")
        {
        param.d2 = stod(i.second);
        }
        else if (i.first == "height")
        {
		param.h = stod(i.second);
        }

        else if(i.first=="t")
        {
            param.totalCount=stod(i.second);   
        }
		else if (i.first == "t1")             //open valve, time of a circle
		{
            param.totalCount1 = stod(i.second);
		}

		else if (i.first == "number")    //number of circle
		{
			param.n = stod(i.second);
		}
		
        else
        {
            std::cout<<"parse failed"<<std::endl;
        }
    }

    msg.copyStruct(param);

    std::cout<<"finished parse"<<std::endl;
}

int  openvalve(aris::dynamic::Model &model, const aris::dynamic::PlanParamBase &param_in)
{
	auto &robot = static_cast<Robots::RobotBase &>(model);
    auto &param = static_cast<const  OpenValve &>(param_in);// change PlanParamBase to MoveupstairsParam

	static double beginBodyPE213[6];
	static double pEE[18];

	if(param.count==0)
	{
        robot.GetPeb(beginBodyPE213);
		robot.GetPee(pEE);
	}
    double realBody[6];
    double realpEE[18];
    static double realBody1[6];

    std::copy(beginBodyPE213,beginBodyPE213 + 6, realBody);
    std::copy(pEE, pEE + 18, realpEE);

   if(param.count>0 && param.count<param.totalCount)         // motion on the plat of xy
    {
       double x = param.d2*(cos(PI*param.count / param.totalCount-PI)+1)/2;
       double y = param.h*(cos(PI*param.count / param.totalCount - PI) + 1)/2;

       realBody[0]=beginBodyPE213[0]+x;
	   realBody[1]=beginBodyPE213[1]+y;
	   
    }

   else if (param.count >= param.totalCount && param.count < 2 * param.totalCount)    //motion of z
   {
       double z = param.d1*(cos(PI*(param.count - param.totalCount + 1) / param.totalCount - PI) + 1)/2;


	   realBody[2] = beginBodyPE213[2] + z;

	   realBody[0] = beginBodyPE213[0] + param.d2;
	   realBody[1] = beginBodyPE213[1] + param.h;
   }
	
   else if (param.count >= 2 * param.totalCount && param.count < 2 * param.totalCount+ param.n*param.totalCount1)   //open valve
   {
	   realBody[2] = beginBodyPE213[2] + param.d1;

       double c = 10;
       double m = 0.3;

       double r = sqrt(param.d2*param.d2 + param.h*param.h);     //radius of circle
       double w = 2 * PI * 1000.0 / param.totalCount1;
	   double A = r*c*w;
	   double theta = atan2(param.h, param.d2);
	    
       static double vx;
       static double vy;
       static double x;
       static double y;

       if(param.count==2*param.totalCount)
       {
           vx=0;
           vy=0;
           x = r*cos(theta);
           y = r*sin(theta);
       }

	   realBody[2] = beginBodyPE213[2] + param.d1;

	   if (param.count < 2 * param.totalCount + 0.9*param.n*param.totalCount1)
       {
          // rt_printf("vx=%.4f,vy=%.4f,x=%.4f,y=%.4f\n",vx,vy,x,y);
           double Fx = -A*sin(w*(param.count - 2 * param.totalCount) / 1000.0 + theta);
           double Fy = A*cos(w*(param.count - 2 * param.totalCount) / 1000.0 + theta);

           double vx1 = vx + 1 / 1000.0 * Fx / m - 1 / 1000.0 * c* vx / m;
           double x1 = x + vx / 1000.0;
           vx=vx1;
           x=x1;

           double vy1 = vy + 1 / 1000.0 * Fy/ m - 1 / 1000.0 * c* vy / m;
           double y1 = y + vy / 1000.0;
           vy=vy1;
           y=y1;

           realBody[0] = beginBodyPE213[0]  + x;
           realBody[1] = beginBodyPE213[1]  + y;

	   }

	   else
       {  // rt_printf("2");
           double Fx = 0;
           double Fy = 0;

           double vx1 = vx + 1/ 1000.0 * Fx / m - 1 / 1000.0 * c* vx / m;
           double x1 = x + vx * 1 / 1000.0;
           x=x1;
           vx=vx1;

           double vy1 = vy + 1 / 1000.0 * Fy/ m - 1 / 1000.0 * c* vy / m;
           double y1 = y + vy/ 1000.0;
           vy=vy1;
           y=y1;

           realBody[0] = beginBodyPE213[0] + x;
           realBody[1] = beginBodyPE213[1] + y;

	   }
	}

   else if (param.count == 2 * param.totalCount + param.n*param.totalCount1)
   {
       robot.GetPeb(realBody1);
       realBody[0]=realBody1[0];
       realBody[1]=realBody1[1];
       realBody[2]=realBody1[2];
   
   }

   double delta_x = realBody1[0] - beginBodyPE213[0];
   double delta_y = realBody1[1] - beginBodyPE213[1];
   double delta_z = realBody1[2] - beginBodyPE213[2];

   if (param.count > 2 * param.totalCount + param.n*param.totalCount1 && param.count <= 3 * param.totalCount + param.n*param.totalCount1)
	{
        realBody[0] = realBody1[0];
        realBody[1] = realBody1[1];

        realBody[2] = realBody1[2] - delta_z*(0.5 - 0.5*cos(PI*(param.count - 2 * param.totalCount - param.n*param.totalCount1) / param.totalCount));

	}
	
	else if (param.count > 3 * param.totalCount + param.n*param.totalCount1 && param.count <= 4 * param.totalCount + param.n*param.totalCount1)
	{
		realBody[0] = realBody1[0] - delta_x*(0.5 - 0.5*cos(PI*(param.count - 3 * param.totalCount - param.n*param.totalCount1) / param.totalCount));
		realBody[1] = realBody1[1] - delta_y*(0.5 - 0.5*cos(PI*(param.count - 3 * param.totalCount - param.n*param.totalCount1) / param.totalCount));
	}
	
    //rt_printf("realBody:%.4f,%.4f,%.4f,%.4f,%.4f,%.4f\n",realBody[0],realBody[1],realBody[2],realBody[3],realBody[4],realBody[5]);
    robot.SetPeb(realBody);
    robot.SetPee(realpEE);

    return  4 * param.totalCount + param.n*param.totalCount1 - param.count ;
}

