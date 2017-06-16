#include <complex>
#include <cmath>
#include <ctime>
#include <iostream>
#include <fstream>

#include <aris.h>

#include "Robot_Gait.h"
#include "Robot_Type_IV.h"

//#define EIGEN_NO_MALLOC
#include "Eigen/Eigen"	

using namespace aris::dynamic;
using namespace std;

namespace Robots
{	
	LegIV::LegIV(const char *Name, RobotTypeIV* pRobot)
		: LegBase(static_cast<RobotBase *>(pRobot), Name)
		, pRobot(pRobot)
	{
	}

	void LegIV::calculate_from_pEE()
	{
		_CalCdByPos();
		_CalVarByCd();
		_CalPartByVar();
	}
	void LegIV::calculate_from_pIn()
	{
		_CalCdByPlen();
		_CalVarByCd();
		_CalPartByVar();
	}

	RobotTypeIV::RobotTypeIV(): pLF{ &LF_Leg }, pLM{ &LM_Leg }, pLR{ &LR_Leg }, pRF{ &RF_Leg }, pRM{ &RM_Leg }, pRR{ &RR_Leg }
	{
		for (int i = 0; i < 6; ++i)
		{
			Robots::RobotBase::pLegs[i] = static_cast<Robots::LegBase *>(pLegs[i]);
		}

		this->scriptPool().add<aris::dynamic::Script>("default_script");
	}

	void RobotTypeIV::SetFixFeet(const char* fixFeet)
	{
		for (int i = 0; i < 6; ++i)
		{
			if (fixFeet[i] == '0')
			{
				pLegs[i]->sf().activate(false);
			}
			else
			{
				pLegs[i]->sf().activate();
			}
		}
	}
	const char* RobotTypeIV::FixFeet() const
	{
		thread_local static char fixFeet[7]{ "000000" };

		for (int i = 0; i < 6; ++i)
		{
			if (pLegs[i]->sf().active())
			{
				fixFeet[i] = '1';
			}
			else
			{
				fixFeet[i] = '0';
			}
		}

		return fixFeet;
	}
	void RobotTypeIV::SetActiveMotion(const char* activeMotion)
	{
		for (int i = 0; i < 6; ++i)
		{
			for (int j = 0; j < 3; ++j)
			{
				if (activeMotion[i * 3 + j] == '0')
				{
					pLegs[i]->motionAt(j).activate(false);
					pLegs[i]->forceAt(j).activate();
				}
				else
				{
					pLegs[i]->motionAt(j).activate();
					pLegs[i]->forceAt(j).activate(false);
				}
			}
		}

	}
	const char* RobotTypeIV::ActiveMotion() const
	{
		thread_local static char activeMotion[19]{ "000000000000000000" };

		for (int i = 0; i < 6; ++i)
		{
			for (int j = 0; j < 3; ++j)
			{
				if (pLegs[i]->motionAt(j).active())
				{
					activeMotion[i * 3 + j] = '1';
				}
				else
				{
					activeMotion[i * 3 + j] = '0';
				}
			}
		}

		return activeMotion;
	}

	auto RobotTypeIV::loadXml(const aris::core::XmlElement &ele)->void
	{
		Model::loadXml(ele);

		/*Update Parts*/
		body_ = partPool().find("MainBody");
		force_sensor_mak_ = body_->markerPool().find("ForceSensorMak");

		for (int j = 0; j < 6; ++j)
		{
			pLegs[j]->p1a_ = partPool().find(pLegs[j]->name() + "_P1a");
			pLegs[j]->p2a_ = partPool().find(pLegs[j]->name() + "_P2a");
			pLegs[j]->p3a_ = partPool().find(pLegs[j]->name() + "_P3a");
			pLegs[j]->thigh_ = partPool().find(pLegs[j]->name() + "_Thigh");
			pLegs[j]->p2b_ = partPool().find(pLegs[j]->name() + "_P2b");
			pLegs[j]->p3b_ = partPool().find(pLegs[j]->name() + "_P3b");
		}

		// Update Markers //
		for (int j = 0; j < 6; ++j)
		{
			pLegs[j]->base_mak_id_ = body().markerPool().find(pLegs[j]->name() + "_Base");

			pLegs[j]->u1i_ = body().markerPool().find(pLegs[j]->name() + "_U1i");
			pLegs[j]->u2i_ = body().markerPool().find(pLegs[j]->name() + "_U2i");
			pLegs[j]->u3i_ = body().markerPool().find(pLegs[j]->name() + "_U3i");

			pLegs[j]->u1j_ = pLegs[j]->p1a().markerPool().find("U1j");
			pLegs[j]->u2j_ = pLegs[j]->p2a().markerPool().find("U2j");
			pLegs[j]->u3j_ = pLegs[j]->p3a().markerPool().find("U3j");

			pLegs[j]->p1i_ = pLegs[j]->thigh().markerPool().find("P1i");
			pLegs[j]->p2i_ = pLegs[j]->p2b().markerPool().find("P2i");
			pLegs[j]->p3i_ = pLegs[j]->p3b().markerPool().find("P3i");

			pLegs[j]->p1j_ = pLegs[j]->p1a().markerPool().find("P1j");
			pLegs[j]->p2j_ = pLegs[j]->p2a().markerPool().find("P2j");
			pLegs[j]->p3j_ = pLegs[j]->p3a().markerPool().find("P3j");

			pLegs[j]->sfi_ = pLegs[j]->thigh().markerPool().find("Sfi");
			pLegs[j]->s2i_ = pLegs[j]->thigh().markerPool().find("S2i");
			pLegs[j]->s3i_ = pLegs[j]->thigh().markerPool().find("S3i");

			pLegs[j]->s2j_ = pLegs[j]->p2b().markerPool().find("S2j");
			pLegs[j]->s3j_ = pLegs[j]->p3b().markerPool().find("S3j");
			pLegs[j]->sfj_ = ground().markerPool().find(pLegs[j]->name() + "_Sfj");
		}

		// Update Joints //
		for (int j = 0; j < 6; ++j)
		{
			pLegs[j]->u1_ = jointPool().find(pLegs[j]->name() + "_U1");
			pLegs[j]->u2_ = jointPool().find(pLegs[j]->name() + "_U2");
			pLegs[j]->u3_ = jointPool().find(pLegs[j]->name() + "_U3");
			pLegs[j]->p1_ = jointPool().find(pLegs[j]->name() + "_P1");
			pLegs[j]->p2_ = jointPool().find(pLegs[j]->name() + "_P2");
			pLegs[j]->p3_ = jointPool().find(pLegs[j]->name() + "_P3");
			pLegs[j]->sf_ = jointPool().find(pLegs[j]->name() + "_Sf");
			pLegs[j]->s2_ = jointPool().find(pLegs[j]->name() + "_S2");
			pLegs[j]->s3_ = jointPool().find(pLegs[j]->name() + "_S3");
		}

		// Update Motions //
		for (int j = 0; j < 6; ++j)
		{
			pLegs[j]->m1_ = motionPool().find(pLegs[j]->name() + "_M1");
			pLegs[j]->m2_ = motionPool().find(pLegs[j]->name() + "_M2");
			pLegs[j]->m3_ = motionPool().find(pLegs[j]->name() + "_M3");
		}

		// Update Forces //
		for (int j = 0; j < 6; ++j)
		{
			pLegs[j]->f1_ = forcePool().find(pLegs[j]->name() + "_F1");
			pLegs[j]->f2_ = forcePool().find(pLegs[j]->name() + "_F2");
			pLegs[j]->f3_ = forcePool().find(pLegs[j]->name() + "_F3");
		}

		// Update Dimension Variables //
		for (int i = 0; i < 6; ++i)
		{
			double pm[4][4];
			s_inv_pm_dot_pm(*pLegs[i]->base().prtPm(), *pLegs[i]->u2i().prtPm(), *pm);
			*const_cast<double *>(&pLegs[i]->U2x) = pm[0][3];
			*const_cast<double *>(&pLegs[i]->U2y) = pm[1][3];
			*const_cast<double *>(&pLegs[i]->U2z) = pm[2][3];
			s_inv_pm_dot_pm(*pLegs[i]->base().prtPm(), *pLegs[i]->u3i().prtPm(), *pm);
			*const_cast<double *>(&pLegs[i]->U3x) = pm[0][3];
			*const_cast<double *>(&pLegs[i]->U3y) = pm[1][3];
			*const_cast<double *>(&pLegs[i]->U3z) = pm[2][3];
			*const_cast<double *>(&pLegs[i]->S2x) = pLegs[i]->s2i().prtPm()[0][3];
			*const_cast<double *>(&pLegs[i]->S2y) = pLegs[i]->s2i().prtPm()[1][3];
			*const_cast<double *>(&pLegs[i]->S2z) = pLegs[i]->s2i().prtPm()[2][3];
			*const_cast<double *>(&pLegs[i]->S3x) = pLegs[i]->s3i().prtPm()[0][3];
			*const_cast<double *>(&pLegs[i]->S3y) = pLegs[i]->s3i().prtPm()[1][3];
			*const_cast<double *>(&pLegs[i]->S3z) = pLegs[i]->s3i().prtPm()[2][3];
			*const_cast<double *>(&pLegs[i]->Sfx) = pLegs[i]->sfi().prtPm()[0][3];
			*const_cast<double *>(&pLegs[i]->Sfy) = pLegs[i]->sfi().prtPm()[1][3];
			*const_cast<double *>(&pLegs[i]->Sfz) = pLegs[i]->sfi().prtPm()[2][3];

			*const_cast<double *>(&pLegs[i]->D1) = pLegs[i]->U2z;
			*const_cast<double *>(&pLegs[i]->H1) = pLegs[i]->U2y;
			*const_cast<double *>(&pLegs[i]->D2) = pLegs[i]->s2i().prtPm()[2][3];
			*const_cast<double *>(&pLegs[i]->H2) = pLegs[i]->s2i().prtPm()[1][3];
		}

		// Update robot to current Pee //
		for (int i = 0; i < 6; ++i)
		{
			pLegs[i]->sfi().update();
			double pe[6];
			pLegs[i]->sfi().getPe(pe);
			pLegs[i]->SetPee(pe);
		}



	}
	auto RobotTypeIV::saveXml(aris::core::XmlElement &xml_ele)const->void
	{
		Model::saveXml(xml_ele);

		auto prt_xml_ele = xml_ele.FirstChildElement("Part");

		std::vector<std::string> leg_name_vec{"LF", "LM", "LR", "RF", "RM", "RR"};

		for (auto &name : leg_name_vec)
		{
			prt_xml_ele->FirstChildElement((name+"_P1a").c_str())->SetAttribute("inertia", "P1aGamma");
			prt_xml_ele->FirstChildElement((name+"_P1a").c_str())->SetAttribute("graphic_file_path", "${P1a_graphic}");
			prt_xml_ele->FirstChildElement((name+"_P1a").c_str())->FirstChildElement("ChildMarker")->FirstChildElement("P1j")->SetAttribute("pe", "P1jpe");
			prt_xml_ele->FirstChildElement((name+"_P1a").c_str())->FirstChildElement("ChildMarker")->FirstChildElement("U1j")->SetAttribute("pe", "U1jpe");
			prt_xml_ele->FirstChildElement((name+"_P2a").c_str())->SetAttribute("inertia", "P2aGamma");
			prt_xml_ele->FirstChildElement((name+"_P2a").c_str())->SetAttribute("graphic_file_path", "${P23a_graphic}");
			prt_xml_ele->FirstChildElement((name+"_P2a").c_str())->FirstChildElement("ChildMarker")->FirstChildElement("P2j")->SetAttribute("pe", "P2jpe");
			prt_xml_ele->FirstChildElement((name+"_P2a").c_str())->FirstChildElement("ChildMarker")->FirstChildElement("U2j")->SetAttribute("pe", "U2jpe");
			prt_xml_ele->FirstChildElement((name+"_P2b").c_str())->SetAttribute("inertia", "P2bGamma");
			prt_xml_ele->FirstChildElement((name+"_P2b").c_str())->SetAttribute("graphic_file_path", "${P23b_graphic}");
			prt_xml_ele->FirstChildElement((name+"_P2b").c_str())->FirstChildElement("ChildMarker")->FirstChildElement("P2i")->SetAttribute("pe", "P2ipe");
			prt_xml_ele->FirstChildElement((name+"_P2b").c_str())->FirstChildElement("ChildMarker")->FirstChildElement("S2j")->SetAttribute("pe", "S2jpe");
			prt_xml_ele->FirstChildElement((name+"_P3a").c_str())->SetAttribute("inertia", "P3aGamma");
			prt_xml_ele->FirstChildElement((name+"_P3a").c_str())->SetAttribute("graphic_file_path", "${P23a_graphic}");
			prt_xml_ele->FirstChildElement((name+"_P3a").c_str())->FirstChildElement("ChildMarker")->FirstChildElement("P3j")->SetAttribute("pe", "P3jpe");
			prt_xml_ele->FirstChildElement((name+"_P3a").c_str())->FirstChildElement("ChildMarker")->FirstChildElement("U3j")->SetAttribute("pe", "U3jpe");
			prt_xml_ele->FirstChildElement((name+"_P3b").c_str())->SetAttribute("inertia", "P3bGamma");
			prt_xml_ele->FirstChildElement((name+"_P3b").c_str())->SetAttribute("graphic_file_path", "${P23b_graphic}");
			prt_xml_ele->FirstChildElement((name+"_P3b").c_str())->FirstChildElement("ChildMarker")->FirstChildElement("P3i")->SetAttribute("pe", "P3ipe");
			prt_xml_ele->FirstChildElement((name+"_P3b").c_str())->FirstChildElement("ChildMarker")->FirstChildElement("S3j")->SetAttribute("pe", "S3jpe");
			prt_xml_ele->FirstChildElement((name+"_Thigh").c_str())->SetAttribute("inertia", "ThighGamma");
			prt_xml_ele->FirstChildElement((name+"_Thigh").c_str())->SetAttribute("graphic_file_path", "${P1b_graphic}");
			prt_xml_ele->FirstChildElement((name+"_Thigh").c_str())->FirstChildElement("ChildMarker")->FirstChildElement("P1i")->SetAttribute("pe", "P1ipe");
			prt_xml_ele->FirstChildElement((name+"_Thigh").c_str())->FirstChildElement("ChildMarker")->FirstChildElement("S2i")->SetAttribute("pe", "S2ipe");
			prt_xml_ele->FirstChildElement((name+"_Thigh").c_str())->FirstChildElement("ChildMarker")->FirstChildElement("S3i")->SetAttribute("pe", "S3ipe");
			prt_xml_ele->FirstChildElement((name+"_Thigh").c_str())->FirstChildElement("ChildMarker")->FirstChildElement("Sfi")->SetAttribute("pe", "Sfipe");

			prt_xml_ele->FirstChildElement("MainBody")->FirstChildElement("ChildMarker")->FirstChildElement((name + "_Base").c_str())->SetAttribute("pe", (name + "pe").c_str());
			prt_xml_ele->FirstChildElement("MainBody")->FirstChildElement("ChildMarker")->FirstChildElement((name + "_U1i").c_str())->SetAttribute("pe", "U1ipe");
			prt_xml_ele->FirstChildElement("MainBody")->FirstChildElement("ChildMarker")->FirstChildElement((name + "_U2i").c_str())->SetAttribute("pe", "U2ipe");
			prt_xml_ele->FirstChildElement("MainBody")->FirstChildElement("ChildMarker")->FirstChildElement((name + "_U3i").c_str())->SetAttribute("pe", "U3ipe");
			prt_xml_ele->FirstChildElement("MainBody")->FirstChildElement("ChildMarker")->FirstChildElement((name + "_U1i").c_str())->SetAttribute("relative_to", (name + "_Base").c_str());
			prt_xml_ele->FirstChildElement("MainBody")->FirstChildElement("ChildMarker")->FirstChildElement((name + "_U2i").c_str())->SetAttribute("relative_to", (name + "_Base").c_str());
			prt_xml_ele->FirstChildElement("MainBody")->FirstChildElement("ChildMarker")->FirstChildElement((name + "_U3i").c_str())->SetAttribute("relative_to", (name + "_Base").c_str());
		}
		
		prt_xml_ele->FirstChildElement("MainBody")->SetAttribute("inertia", "BodyGamma");
		prt_xml_ele->FirstChildElement("MainBody")->SetAttribute("graphic_file_path", "${Body_graphic}");

		auto mot_xml_ele = xml_ele.FirstChildElement("Motion");
		for (auto i = mot_xml_ele->FirstChildElement(); i != nullptr; i = i->NextSiblingElement())
		{
			i->SetAttribute("frc_coe", "Mot_friction");
		}


	};

	auto RobotTypeIV::simToAdams(const std::string &adams_file, const aris::dynamic::PlanFunc &func, const aris::dynamic::PlanParamBase &param, int ms_dt)->aris::dynamic::SimResult
	{
		double begin_pee[18], begin_peb[6];
		this->GetPee(begin_pee);
		this->GetPeb(begin_peb);
		this->saveDynEle("before_robotTypeI_simToAdams");
		


		enum STATE { STAND, SUSPEND, MOVE,} last_state[6], this_state[6];
		std::int32_t sim_time{ 0 };
		double last_Pee[6][3];
		
		scriptPool().at(0).clear();
		
		auto getState = [&]()->void
		{
			int stand_num = 0;
			for (auto i = 0; i < 6; ++i)
			{
				double Pee_loc[3];
				pLegs[i]->GetPee(Pee_loc);

				bool is_equal = aris::dynamic::s_is_equal(3, last_Pee[i], Pee_loc, 1e-10);

				if ((is_equal) && (stand_num<3))
				{
					this_state[i] = STAND;
					stand_num++;
				}
				else if (is_equal)
				{
					this_state[i] = SUSPEND;
					stand_num++;
				}
				else
				{
					this_state[i] = MOVE;
				}

				pLegs[i]->GetPee(last_Pee[i]);
			}

			if (stand_num < 3)throw std::runtime_error("can't sim gait because at some time the stand leg num is less than 3");
		};
		
		//起始位置
		this->GetPee(*last_Pee);

		param.count = 0;
		func(*this, param);

		getState();
		
		for (auto i = 0; i < 6; ++i)
		{
			switch (this_state[i])
			{
			case STAND:
			{
				aris::dynamic::DynEle *ele_group[]{ &pLegs[i]->m1(),&pLegs[i]->f2(),&pLegs[i]->f3() };
				for (auto &ele : ele_group)scriptPool().at(0).act(*ele, false);
				break;
			}
			case SUSPEND:
			{
				aris::dynamic::DynEle *ele_group[]{ &pLegs[i]->m1(),&pLegs[i]->m2(),&pLegs[i]->m3() };
				for (auto &ele : ele_group)scriptPool().at(0).act(*ele, false);
				break;
			}
			case MOVE:
			{
				aris::dynamic::DynEle *ele_group[]{ &pLegs[i]->sf(),&pLegs[i]->f1(),&pLegs[i]->f2(),&pLegs[i]->f3() };
				for (auto &ele : ele_group)scriptPool().at(0).act(*ele, false);
				break;
			}
			}
		}
		std::copy_n(this_state, 6, last_state);
		sim_time++;

		//其他位置
		
		for (param.count = 1; true; ++param.count)
		{
			auto is_sim = func(*this, param);

			getState();

			bool is_change_topology = false;
			for (auto i = 0; i < 6; ++i) if (last_state[i] != this_state[i]) is_change_topology = true;

			if (is_change_topology)
			{
				scriptPool().at(0).sim(sim_time, ms_dt);
				sim_time = 0;
			}
			
			for (auto i = 0; i < 6; ++i)
			{
				switch (last_state[i])
				{
				case STAND:
				{
					switch (this_state[i])
					{
					case STAND: break;
					case SUSPEND:
					{
						scriptPool().at(0).act(pLegs[i]->m2(), false);
						scriptPool().at(0).act(pLegs[i]->m3(), false);
						scriptPool().at(0).act(pLegs[i]->f2(), true);
						scriptPool().at(0).act(pLegs[i]->f3(), true);
						break;
					}
					case MOVE:
					{
						scriptPool().at(0).act(pLegs[i]->sf(), false);
						scriptPool().at(0).act(pLegs[i]->m1(), true);
						scriptPool().at(0).act(pLegs[i]->f1(), false);
						break;
					}
					}
					break;
				}
				case SUSPEND:
				{
					switch (this_state[i])
					{
					case STAND: 
					{
						scriptPool().at(0).act(pLegs[i]->m2(), true);
						scriptPool().at(0).act(pLegs[i]->m3(), true);
						scriptPool().at(0).act(pLegs[i]->f2(), false);
						scriptPool().at(0).act(pLegs[i]->f3(), false);
						break;
					}
					case SUSPEND:break;
					case MOVE:
					{
						scriptPool().at(0).act(pLegs[i]->sf(), false);
						scriptPool().at(0).act(pLegs[i]->m1(), true);
						scriptPool().at(0).act(pLegs[i]->f1(), false);
						scriptPool().at(0).act(pLegs[i]->m2(), true);
						scriptPool().at(0).act(pLegs[i]->f2(), false);
						scriptPool().at(0).act(pLegs[i]->m3(), true);
						scriptPool().at(0).act(pLegs[i]->f3(), false);
						break;
					}
					}
					break;
				}
				case MOVE:
				{
					switch (this_state[i])
					{
					case STAND: 
					{
						scriptPool().at(0).act(pLegs[i]->sf(), true);
						scriptPool().at(0).act(pLegs[i]->m1(), false);
						scriptPool().at(0).act(pLegs[i]->f1(), true);
						scriptPool().at(0).aln(pLegs[i]->sfj(), pLegs[i]->sfi());
						break;
					}
					case SUSPEND:
					{
						scriptPool().at(0).act(pLegs[i]->sf(), true);
						scriptPool().at(0).act(pLegs[i]->m1(), false);
						scriptPool().at(0).act(pLegs[i]->f1(), true);
						scriptPool().at(0).act(pLegs[i]->m2(), false);
						scriptPool().at(0).act(pLegs[i]->f2(), true);
						scriptPool().at(0).act(pLegs[i]->m3(), false);
						scriptPool().at(0).act(pLegs[i]->f3(), true);
						scriptPool().at(0).aln(pLegs[i]->sfj(), pLegs[i]->sfi());
						break;
					}
					case MOVE:break;
					}
					break;
				}
					
				}
			}

			std::copy_n(this_state, 6, last_state);

			sim_time++;

			if (!is_sim) 
			{
				scriptPool().at(0).sim(sim_time, ms_dt);
				break;
			}
		}

		this->loadDynEle("before_robotTypeI_simToAdams");
		this->SetPee(begin_pee);
		this->SetPeb(begin_peb);

		return this->aris::dynamic::Model::simToAdams(adams_file, func, param, ms_dt, &scriptPool().at(0));
	}
}