#ifndef ROBOT_TYPE_IV_H
#define ROBOT_TYPE_IV_H

#include <aris.h>
#include <Robot_Base.h>
#include <Robot_Gait.h>

namespace Robots
{
	class RobotTypeIV;
	class LegIV :public Robots::LegBase
	{
	public:
		auto partAt(std::size_t id)->aris::dynamic::Part& { return *prt_array_[id]; };
		auto markerAt(std::size_t id)->aris::dynamic::Marker& { return *mak_array_[id]; };
		auto jointAt(std::size_t id)->aris::dynamic::Joint& { return *jnt_array_[id]; };
		auto motionAt(std::size_t id)->aris::dynamic::Motion& { return *mot_array_[id];};
		auto forceAt(std::size_t id)->aris::dynamic::SingleComponentForce& { return static_cast<aris::dynamic::SingleComponentForce&>(*fce_array_[id]); };

		auto l1()->aris::dynamic::Part& { return static_cast<aris::dynamic::Part&>(*l1_); };
		auto l2()->aris::dynamic::Part& { return static_cast<aris::dynamic::Part&>(*l2_); };
        auto l3()->aris::dynamic::Part& { return static_cast<aris::dynamic::Part&>(*l3_); };
		auto l4()->aris::dynamic::Part& { return static_cast<aris::dynamic::Part&>(*l4_); };

		auto r01i()->aris::dynamic::Marker& { return static_cast<aris::dynamic::Marker&>(*r01i_); };
		auto r01j()->aris::dynamic::Marker& { return static_cast<aris::dynamic::Marker&>(*r01j_); };
		auto r14i()->aris::dynamic::Marker& { return static_cast<aris::dynamic::Marker&>(*r14i_); };
		auto r14j()->aris::dynamic::Marker& { return static_cast<aris::dynamic::Marker&>(*r14j_); };
        auto r02i()->aris::dynamic::Marker& { return static_cast<aris::dynamic::Marker&>(*r02i_); };
        auto r02j()->aris::dynamic::Marker& { return static_cast<aris::dynamic::Marker&>(*r02j_); };
        auto r23i()->aris::dynamic::Marker& { return static_cast<aris::dynamic::Marker&>(*r23i_); };
        auto r23j()->aris::dynamic::Marker& { return static_cast<aris::dynamic::Marker&>(*r23j_); };
        auto r34i()->aris::dynamic::Marker& { return static_cast<aris::dynamic::Marker&>(*r34i_); };
        auto r34j()->aris::dynamic::Marker& { return static_cast<aris::dynamic::Marker&>(*r34j_); };
        auto foot()->aris::dynamic::Marker& { return static_cast<aris::dynamic::Marker&>(*foot_); };

		auto r01()->aris::dynamic::RevoluteJoint& { return static_cast<aris::dynamic::RevoluteJoint&>(*r01_); };
        auto r14()->aris::dynamic::RevoluteJoint& { return static_cast<aris::dynamic::RevoluteJoint&>(*r14_); };
        auto r02()->aris::dynamic::RevoluteJoint& { return static_cast<aris::dynamic::RevoluteJoint&>(*r02_); };
        auto r23()->aris::dynamic::RevoluteJoint& { return static_cast<aris::dynamic::RevoluteJoint&>(*r23_); };
        auto r34()->aris::dynamic::RevoluteJoint& { return static_cast<aris::dynamic::RevoluteJoint&>(*r34_); };

        auto m1()->aris::dynamic::SingleComponentMotion& { return static_cast<aris::dynamic::SingleComponentMotion&>(*m1_); };
		auto m2()->aris::dynamic::SingleComponentMotion& { return static_cast<aris::dynamic::SingleComponentMotion&>(*m2_); };
		auto f1()->aris::dynamic::SingleComponentForce& { return static_cast<aris::dynamic::SingleComponentForce&>(*f1_); };
		auto f2()->aris::dynamic::SingleComponentForce& { return static_cast<aris::dynamic::SingleComponentForce&>(*f2_); };
		
		//void GetdJacOverPee(double *dJi_x, double *dJi_y, double *dJi_z, const char *relativeCoordinate="G")const;


	private:
		LegIV(const char *Name, RobotTypeIV* pRobot);
		virtual ~LegIV() = default;

		//void FastDyn();
		//
		//void GetFin(double *fIn) const;
		//void GetFinDyn(double *fIn) const;
		//void GetFinFrc(double *fIn) const;

		virtual void calculate_from_pEE();
		virtual void calculate_from_pIn();
		//virtual void calculate_from_vEE();
		//virtual void calculate_from_vIn();
		//virtual void calculate_from_aEE();
		//virtual void calculate_from_aIn();

		//virtual void calculate_jac();
		//virtual void calculate_diff_jac();

		//void _CalCdByPos();
		//void _CalCdByPlen();
		//void _CalCdByPlen2();
		//void _CalVarByCd();
		//void _CalPartByVar();

		//void _CalVcdByVpos();
		//void _CalVcdByVplen();
		//void _CalVvarByVcd();
		//void _CalVpartByVvar();

		//void _CalAcdByApos();
		//void _CalAcdByAplen();
		//void _CalAvarByAcd();
		//void _CalApartByAvar();

	private:
		typedef aris::dynamic::Part* PartPtr;
		union
		{
			PartPtr prt_array_[4];
			struct
			{
				PartPtr l1_, l2_, l3_, l4_;
			};
		};
		typedef aris::dynamic::Marker* MarkerPtr;
		union
		{
			MarkerPtr mak_array_[11];
			struct
			{
				MarkerPtr r01i_, r01j_, r14i_, r14j_, r02i_, r02j_, r23i_, r23j_, r34i_, r34j_, foot_;
			};
		};

		typedef aris::dynamic::Joint* JointPtr;
		union
		{
			JointPtr jnt_array_[5];
			struct
			{
				JointPtr r01_, r14_, r02_, r23_, r34_;
			};
		};
		typedef aris::dynamic::Motion* MotionPtr;
		union
		{
			MotionPtr mot_array_[2];
			struct
			{
				MotionPtr m1_, m2_;
			};
		};
		typedef aris::dynamic::Force* ForcePtr;
		union
		{
			ForcePtr fce_array_[2];
			struct
			{
				ForcePtr f1_, f2_;
			};
		};
		
		//
		//union
		//{
		//	double fIn_dyn[3];
		//	struct
		//	{
		//		double f1_dyn;
		//		double f2_dyn;
		//		double f3_dyn;
		//	};
		//};
		//union
		//{
		//	double fIn_frc[3];
		//	struct
		//	{
		//		double f1_frc;
		//		double f2_frc;
		//		double f3_frc;
		//	};
		//};

	private:
		RobotTypeIV *pRobot;

		const double U2x{ 0 }, U2y{ 0 }, U2z{ 0 }, U3x{ 0 }, U3y{ 0 }, U3z{ 0 };
		const double S2x{ 0 }, S2y{ 0 }, S2z{ 0 }, S3x{ 0 }, S3y{ 0 }, S3z{ 0 };
		const double Sfx{ 0 }, Sfy{ 0 }, Sfz{ 0 };

		const double D1{ 0 }, H1{ 0 }, D2{ 0 }, H2{ 0 };

		double a1, b1, va1, vb1, aa1, ab1;
		double x2, y2, z2, x3, y3, z3;
		double a2, b2, a3, b3;
		double sa1, ca1, sb1, cb1, sa2, ca2, sb2, cb2, sa3, ca3, sb3, cb3;
		double pa1, qa1, pb1, qb1, pa2, qa2, pb2, qb2, pa3, qa3, pb3, qb3;

		double vx2, vy2, vz2, vx3, vy3, vz3;
		double va2, vb2, va3, vb3;

		double ax2, ay2, az2, ax3, ay3, az3;
		double aa2, ab2, aa3, ab3;

		double H11, H12, H21, H22;
		double k21, k22, k23, k31, k32, k33;
		double vk21, vk22, vk23, vk31, vk32, vk33;
		double J1[3][3], J2[3][3], vJ1[3][3], vJ2[3][3];
		double inv_J1[3][3], inv_J2[3][3];

		double _C[36][36];
		double _c_M[36][4];

		friend class RobotTypeIV;
	};
	class RobotTypeIV :public Robots::RobotBase
	{
	public:
		RobotTypeIV();
		~RobotTypeIV() = default;
		
		virtual auto loadXml(const aris::core::XmlElement &xml_ele)->void override;
		virtual auto saveXml(aris::core::XmlElement &xml_ele)const->void override;
		using Model::loadXml;
		using Model::saveXml;

		//void GetFin(double *Fin) const;
		//void GetFinDyn(double *Fin) const;
		//void GetFinFrc(double *Fin) const;

		//void FastDyn();
		//virtual void dyn()override;

		void SetFixFeet(const char* fix_feet);
		const char* FixFeet() const;
		void SetActiveMotion(const char* active_motion);
		const char* ActiveMotion() const;

		virtual void kinFromPin()override
		{
			double Pin[18];
			for (int i = 0; i < 18; ++i)
			{
				Pin[i] = motionPool().at(i).motPos();
			}

			double pe[6];
			this->GetPeb(pe);
			SetPinFixFeet(Pin, FixFeet(), ActiveMotion(), pe);
		};
		virtual void kinFromVin()override
		{
			double Vin[18];
			for (int i = 0; i < 18; ++i)
			{
				Vin[i] = motionPool().at(i).motVel();
			}
			SetVinFixFeet(Vin, FixFeet(), ActiveMotion());
		};
		auto simToAdams(const std::string &adams_file, const aris::dynamic::PlanFunc &fun, const aris::dynamic::PlanParamBase &param, int ms_dt)->aris::dynamic::SimResult;
	
	public:
		union
		{
			struct
			{
				LegIV *const pLF;
				LegIV *const pLM;
				LegIV *const pLR;
				LegIV *const pRF;
				LegIV *const pRM;
				LegIV *const pRR;
			};
			LegIV *const pLegs[6];
		};

	private:
		LegIV LF_Leg{ "LF", this };
		LegIV LM_Leg{ "LM", this };
		LegIV LR_Leg{ "LR", this };
		LegIV RF_Leg{ "RF", this };
		LegIV RM_Leg{ "RM", this };
		LegIV RR_Leg{ "RR", this };

		friend class LegIV;
	};


}

#endif