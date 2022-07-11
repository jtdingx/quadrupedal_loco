/**
NLPClass.h

Description:	Header file of NLPClass
*/
#pragma once

#include "QP/QPBaseClass.h"
#include <iostream>
#include <Eigen/Dense>
#include <math.h>
#include <fstream>
#include <time.h>
#include "Robotpara/robot_const_para_config.h"

#include <vector> 

// #include "KMP/kmp.h"
// #include <armadillo>



using namespace Eigen;
using namespace std;
// using namespace arma;



/// constant variable defintion
const int _footstepsnumber = 27;       //  number of _footstepnumber
//const int _footstepsnumber = 15;       //ball hit
const double _dt = 0.025;                //sampling time
const int _nh = 30;                    /// =PreviewT/_dt: number of sampling time for predictive window: <= 2*_nT;
const double _tstep = 0.7;              ///step period
const int _nT = round(_tstep/_dt);      /// _tstep/_dt)  the number of one step cycle
const int _nsum = (_footstepsnumber-1)*_nT; /// number of whole control loop
const double _height_offset = 0.000;     
const double _height_squat_time = 1;


class NLPClass : public QPBaseClass
{
public:
	NLPClass();
	virtual ~NLPClass() {};
	
	// /******************* KMP class preparation **************/
    // kmp kmp_leg_L;
	// kmp kmp_leg_R;

	// int    _inDim_kmp; 	      		    //input dimension
	// int    _outDim_kmp; 	      		    //output dimension
	// int    _pvFlag_kmp;			    // output: pos (and vel)
	// ///////////// adjust KMP parameters
	// double _lamda_kmp, _kh_kmp;	    	    //set kmp parameters 
	// vec    _query_kmp;            	    // input
	// vec    _mean_kmp;  	            // output:mean
	// mat    _data_kmp;
	

	/*********************************************** For NLPClass definition *********************/
	
	//void FootStepNumberInputs(int footstepsnumber);
	void FootStepInputs(double stepwidth, double steplength, double stepheight);	
	void Initialize();

	////////////////=================================================//////////////////
	/// for step timing optimization
	Eigen::Matrix<double, 38, 1>  step_timing_opti_loop(int i,Eigen::Matrix<double,18,1> estimated_state, Eigen::Vector3d _Rfoot_location_feedback, Eigen::Vector3d _Lfoot_location_feedback,double lamda, bool _stopwalking);
	int Indexfind(double goalvari, int xyz);
	void solve_stepping_timing(); 		


	///////////////=================================================/////////////////////////////
	////// for trajectory intepolation
	void Foot_trajectory_solve(int j_index, bool _stopwalking);
    Eigen::Matrix<double, 18, 1> Foot_trajectory_solve_mod2(int j_index,bool _stopwalking);
    
    
    
	int Get_maximal_number(double dtx);	
	int Get_maximal_number_reference();
	/// when no mpc is used, the CoM height trajectory should be generated by interpolation!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
	//////////////////////////////////////////////////////////////////////////////
	void CoM_height_solve(int j_index, bool _stopwalking, int ntdx);	
	Eigen::Matrix<double, 9, 1> X_CoM_position_squat(int walktime, double dt_sample);
	
	// current state based on the past one and two actual sampling time;
	Vector3d XGetSolution_CoM_position(int walktime, double dt_sample, Eigen::Vector3d body_in1, Eigen::Vector3d body_in2, Eigen::Vector3d body_in3);
	Vector3d XGetSolution_Foot_positionR(int walktime, double dt_sample, Eigen::Vector3d body_in1, Eigen::Vector3d body_in2, Eigen::Vector3d body_in3);
	Vector3d XGetSolution_Foot_positionL(int walktime, double dt_sample, Eigen::Vector3d body_in1, Eigen::Vector3d body_in2, Eigen::Vector3d body_in3);	
	Vector3d XGetSolution_body_inclination(int walktime, double dt_sample, Eigen::Vector3d body_in1, Eigen::Vector3d body_in2, Eigen::Vector3d body_in3);
	// Eigen::Matrix<double,6,1> XGetSolution_Foot_position_KMP(int walktime, double dt_sample, int j_index);
	// Eigen::Matrix<double,6,1> XGetSolution_Foot_position_KMP_faster(int walktime, double dt_sample, int j_index);
	
	
	
	int _j_period;

	Eigen::Matrix<double,_footstepsnumber,1> _tx;	
	Eigen::Matrix<double,_footstepsnumber,1> _ts, _td;



////////// generate the COM trajectory during the DSP
	int _nTdx;
	
	
	int _bjx1, _bjx2, _mx;	
	int right_support;
	
	int _j_count;
	
	Vector3d _F_R, _F_L,_M_R,_M_L;
	Vector3d _ZMPxy_realx;
	Vector4d _zmp_rela_vari;
	
	
	Vector3d _comxyzx,_comvxyzx,_comaxyzx,_thetaxyx,_thetavxyx,_thetaaxyx,_Lfootxyzx,_Rfootxyzx;	
	
	void Zmp_distributor(int walktime, double dt_sample);
	
	
	
	/////**********************save the optimal trajectory generated by MPC********************///
	void File_wl_steptiming();	
	void File_wl();
	
	int _n_loop_omit;	
	
	
	int _method_flag;
	std::string _robot_name;
	double _robot_mass;
	double _lift_height;	
	
	// /////**********************save the optimal trajectory generated by kmp********************///
	// void File_wl_kmp();
	

	//Eigen::Matrix<double,32,_nsum> CoMMM_ZMP_foot;
	
// 	Eigen::Matrix4d AAAaaa1;
	double _ZMP_ratio;
	double _t_end_walking;
	double _t_end_footstep;
	int _n_end_walking;
	
	Eigen::Vector4d  _comy_matrix_inv;

	
	///////////////for polynomial intepolation
	Eigen::Matrix<double, 4,4>  _AAA_inv;
	
	void solve_AAA_inv(Eigen::Matrix<double, 4, 1> t_plana); 
	
	Eigen::Matrix<double, 7, 7> solve_AAA_inv_x(Eigen::Vector3d t_plan); 
	Eigen::Matrix<double, 4, 4> solve_AAA_inv2(Eigen::Vector3d t_plan);
	  
	
	////for lowel level control: ZMP optimal distribution
	Matrix<double,3,3> _Co_L, _Co_R;
		 
	void Force_torque_calculate(Vector3d comxyzx1,Vector3d comaxyzx1,Vector3d thetaaxyx1,Vector3d Lfootxyz,Vector3d Rfootxyz);

	void zmp_interpolation(int t_int,int walktime, double dt_sample);
		
	///// MPC_com_reference/////
//	_comx_center_ref_lip;
	Eigen::Matrix<double, _nsum,1> _comx_ref_lip,_comy_ref_lip, _comvx_ref_lip, _comvy_ref_lip; 
	
	void Ref_com_lip_offline();
	void Ref_com_lip_update(int j_current);	
	
	double RobotPara_totalmass;
	double RobotPara_HALF_HIP_WIDTH;
	double RobotPara_dt;
	double RobotPara_Tstep;
	double RobotPara_Z_C;   
	double RobotPara_g;
	double RobotPara_FOOT_WIDTH;
	
	Eigen::Matrix<double, 18, 1> XGetSolution_Foot_rotation(const int walktime, const double dt_sample,int j_index);
	Vector3d _Lfoot_r, _Rfoot_r,_Lfoot_rv, _Rfoot_rv,_Lfoot_ra, _Rfoot_ra;	


private:    
        /// update objective function and constraints matrices for step timing optimization
	void step_timing_object_function(int i);  
  
	void step_timing_constraints(int i);  
	
	
    //parameters declaration  
    // all the variable for step timing optimization
    Eigen::Matrix<double,_footstepsnumber,1> _steplength, _stepwidth, _stepheight,_lift_height_ref;
    Eigen::Matrix<double,_footstepsnumber,1> _Lxx_ref, _Lyy_ref;  ///reference steplengh and stepwidth
    Eigen::Matrix<double,_footstepsnumber,1> _footx_ref, _footy_ref, _footz_ref;    //footstep locations generation
	Eigen::Matrix<double,_footstepsnumber,1> _footx_offline, _footy_offline, _footz_offline;    //footstep locations generation
	/// support foot location feedbacke for step time optimization	
	Eigen::Matrix<double,_footstepsnumber,1> _footx_real_feed, _footy_real_feed;	        
        
        	 
	Eigen::Matrix<double,_nsum,1> _t;	            ///whole sampling time sequence

	Eigen::Matrix<double,1,_nsum> _comx, _comvx, _comax;
	Eigen::Matrix<double,1,_nsum> _comy, _comvy, _comay;
	Eigen::Matrix<double,1,_nsum> _comz, _comvz, _comaz;
	Eigen::Matrix<double,1,_nsum> _Lxx_ref_real, _Lyy_ref_real,_Ts_ref_real; 
	
	/// para
	double _hcom;
	double _g;	
	double _Wn;
	double _Wndt;
	
    
    Eigen::Matrix<double,_nsum,1> _t_whole;
    
	Eigen::Matrix<double,1,_nsum> _px, _py, _pz;
	Eigen::Matrix<double,1,_nsum> _zmpvx, _zmpvy;
	Eigen::Matrix<double,1,_footstepsnumber> _COMx_is, _COMx_es, _COMvx_is;
	Eigen::Matrix<double,1,_footstepsnumber> _COMy_is, _COMy_es, _COMvy_is;
	Eigen::Matrix<double,1,_nsum> _comx_feed, _comvx_feed,_comax_feed;
	Eigen::Matrix<double,1,_nsum> _comy_feed, _comvy_feed,_comay_feed;
        
	//optimal variables
	Eigen::Matrix<double,4,_nsum> _Vari_ini;
	Eigen::Matrix<double,4,1> _vari_ini;

	// weight coefficient	
	double _aax,_aay,_aaxv,_aayv,_bbx,_bby,_rr1,_rr2;
	double _aax1,_aay1,_bbx1,_bby1,_rr11,_rr21;
	
	//constraints on step timing parameters
	double _t_min, _t_max;
	
	// swing foot velocity constraints  parameters
	double _footx_vmax, _footx_vmin,_footy_vmax,_footy_vmin;	
	
	// swing foot velocity constraints  parameters
	double _comax_max, _comax_min,_comay_max, _comay_min;	


	
	// %%% physical	
	double _mass,  _rad,  _j_ini;	
	
	//external force
        double _FX, _FY;	
	double _t_last;
	double _det_xa,_det_ya;
	double _det_xv,_det_yv;
	double _det_xp,_det_yp;
	
	Eigen::Matrix<double,1,_nsum> _tcpu;	
	
	
	

	
	int xyz0; //flag for find function 
	int xyz1;  
	int xyz2;	
    
    //footz reference: matrix operations
    Eigen::Matrix<double,_nsum,1> _Zsc;	    
	
	int _periond_i; ///
	
	int _ki, _k_yu;
	double _Tk;
	
	double _Lxx_refx,_Lyy_refy,_Lxx_refx1,_Lyy_refy1;
	double _tr1_ref,_tr2_ref,_tr1_ref1,_tr2_ref1;
	
	double _tr1_min,_tr2_min,_tr1_max,_tr2_max,_tr11_min,_tr21_min,_tr11_max,_tr21_max;
	
	Eigen::Matrix<double,1,4> _SS1,_SS2,_SS3,_SS4;
	
	Eigen::Matrix<double,1,1> _comvx_endref,_comvy_endref;
	
	Eigen::Matrix<double,1,1> _AxO,_BxO,_Cx, _Axv,_Bxv,_Cxv;
	Eigen::Matrix<double,1,1> _AyO,_ByO,_Cy, _Ayv,_Byv,_Cyv;
	
	Eigen::Matrix<double,4,4> _SQ_goal0,_SQ_goal,_SQ_goal1,_SQ_goal20,_SQ_goal2,_SQ_goal3;
	Eigen::Matrix<double,4,1> _Sq_goal,_Sq_goal1,_Sq_goal2,_Sq_goal3;
	Eigen::Matrix<double,4,4> _Ax,_Ay;
	Eigen::Matrix<double,1,1> _Bx,_By;
	Eigen::Matrix<double,1,1> _ixi,_iyi;	
	
	//constraints on step timing parameters
	Eigen::Matrix<double,1,4> _trx1_up, _trx1_lp,_trx2_up, _trx2_lp,_trx3_up, _trx3_lp,_trx4_up, _trx4_lp;	
	Eigen::Matrix<double,1,1> _det_trx1_up, _det_trx1_lp,_det_trx2_up,_det_trx2_lp,_det_trx3_up, _det_trx3_lp,_det_trx4_up, _det_trx4_lp;		
	Eigen::Matrix<double,4,4> _trx;
	Eigen::Matrix<double,4,1> _det_trx;
	
	// tr1 & tr2: equation constraints
	Eigen::Matrix<double,1,4> _trx12, _trx121;	
	Eigen::Matrix<double,1,1> _det_trx12, _det_trx121;		
	Eigen::Matrix<double,1,4> _trxx;
	Eigen::Matrix<double,1,1> _det_trxx;	
	
	Eigen::Matrix<double,4,4> _trx12_up2, _trx12_lp2;
	Eigen::Matrix<double,4,1> _trx12_up1, _trx12_lp1;
	Eigen::Matrix<double,1,1> _det_trx12_up, _det_trx12_lp;
	
	
	///foot location constraints 
	double _footx_max, _footx_min,_footy_max,_footy_min;	
	Eigen::Matrix<double,1,4> _h_lx_up,  _h_lx_lp, _h_ly_up, _h_ly_lp,_h_lx_up1, _h_lx_lp1, _h_ly_up1, _h_ly_lp1;
	Eigen::Matrix<double,1,1> _det_h_lx_up,_det_h_lx_lp,_det_h_ly_up,_det_h_ly_lp,_det_h_lx_up1,_det_h_lx_lp1,_det_h_ly_up1,_det_h_ly_lp1;
	Eigen::Matrix<double,4,4> _h_lx_upx;
	Eigen::Matrix<double,4,1> _det_h_lx_upx;


	///foot location constraints 	
	Eigen::Matrix<double,1,4> _h_lvx_up,  _h_lvx_lp, _h_lvy_up, _h_lvy_lp,_h_lvx_up1, _h_lvx_lp1, _h_lvy_up1, _h_lvy_lp1;
	Eigen::Matrix<double,1,1> _det_h_lvx_up,_det_h_lvx_lp,_det_h_lvy_up,_det_h_lvy_lp,_det_h_lvx_up1,_det_h_lvx_lp1,_det_h_lvy_up1,_det_h_lvy_lp1;
	Eigen::Matrix<double,4,4> _h_lvx_upx;
	Eigen::Matrix<double,4,1> _det_h_lvx_upx;	
	
	/// CoM accelearation boundary
	double _AA, _CCx, _BBx, _CCy, _BBy,_AA1x,_AA2x,_AA3x,_AA1y,_AA2y,_AA3y;
	Eigen::Matrix<double,1,4> _CoM_lax_up,  _CoM_lax_lp,  _CoM_lay_up,  _CoM_lay_lp;
	Eigen::Matrix<double,1,1> _det_CoM_lax_up,  _det_CoM_lax_lp,  _det_CoM_lay_up,  _det_CoM_lay_lp;
	Eigen::Matrix<double,4,4> _CoM_lax_upx;
	Eigen::Matrix<double,4,1> _det_CoM_lax_upx;	
	
	
	
	/// CoM velocity_inremental boundary
	double _VAA, _VCCx, _VBBx, _VCCy, _VBBy,_VAA1x,_VAA2x,_VAA3x,_VAA1y,_VAA2y,_VAA3y;
	Eigen::Matrix<double,1,4> _CoM_lvx_up,  _CoM_lvx_lp,  _CoM_lvy_up,  _CoM_lvy_lp;
	Eigen::Matrix<double,1,1> _det_CoM_lvx_up,  _det_CoM_lvx_lp,  _det_CoM_lvy_up,  _det_CoM_lvy_lp;
	Eigen::Matrix<double,4,4> _CoM_lvx_upx;
	Eigen::Matrix<double,4,1> _det_CoM_lvx_upx;	
	
	/// CoM initial velocity_ boundary
	double _VAA1x1,_VAA2x1,_VAA3x1,_VAA1y1,_VAA2y1,_VAA3y1;
	Eigen::Matrix<double,1,4> _CoM_lvx_up1,  _CoM_lvx_lp1,  _CoM_lvy_up1,  _CoM_lvy_lp1;
	Eigen::Matrix<double,1,1> _det_CoM_lvx_up1,  _det_CoM_lvx_lp1,  _det_CoM_lvy_up1,  _det_CoM_lvy_lp1;
	Eigen::Matrix<double,4,4> _CoM_lvx_upx1;
	Eigen::Matrix<double,4,1> _det_CoM_lvx_upx1;	
	
	//DCM constraints
	double _cpx_max, _cpx_min,_cpy_max, _cpy_min, _lp, _W_max, _W_min;
	Eigen::Matrix<double,1,4> _cp_x_up,  _cp_x_min,  _cp_y_up,  _cp_y_min;
	Eigen::Matrix<double,1,1> _det_cp_x_up,  _det_cp_x_min,  _det_cp_y_up,  _det_cp_y_min, _VCCx_matrix,_VCCy_matrix;
	Eigen::Matrix<double,4,4> _h_cp;
	Eigen::Matrix<double,4,1> _det_cp;
	
	Eigen::Matrix<double,4,4> _cp_x_up2,  _cp_x_lp2,  _cp_y_up2,  _cp_y_lp2;
	Eigen::Matrix<double,4,1> _cp_x_up1,  _cp_x_lp1,  _cp_y_up1,  _cp_y_lp1;
	Eigen::Matrix<double,1,1> _det_cp_x_up1,  _det_cp_x_lp1,  _det_cp_y_up1,  _det_cp_y_lp1;	
	
	
	/// Constraints transformer
	vector <Eigen::MatrixXd> H2_q_xy;
	Eigen::Matrix<double,8,4> H1_q_xy;
	Eigen::Matrix<double,8,1> F_quadratci;
	
	Eigen::Matrix<double,5,5> _W_goal;
	
	
	vector <Eigen::MatrixXd> cin_quad, cin_aff;
	Eigen::Matrix<double,24,4> _A_q1;
	Eigen::Matrix<double,24,1> _b_q1;        
		
	void fun_sdr_qudratic_constraints();
	void fun_sdr_affine_constraints();

	
	/// for foot trajectory generation
	Eigen::Matrix<double,_nh,1> _t_f;	

	int _bjxx;	
	Eigen::Matrix<double, 3,_footstepsnumber> _footxyz_real;
	
	Eigen::Matrix<double, 1,_nsum> _Lfootx, _Lfooty,_Lfootz, _Lfootvx, _Lfootvy,_Lfootvz, _Lfootax, _Lfootay,_Lfootaz;
	Eigen::Matrix<double, 1,_nsum> _Rfootx, _Rfooty,_Rfootz, _Rfootvx, _Rfootvy,_Rfootvz, _Rfootax, _Rfootay,_Rfootaz;	
        double _ry_left_right;	
	
	
	////result CoM_foot_trajection_generation
	Eigen::Matrix<double,3,_nsum> _CoM_position_optimal;
	Eigen::Matrix<double,3,_nsum> _torso_angle_optimal;
	Eigen::Matrix<double,3,_nsum> _L_foot_optition_optimal;
	Eigen::Matrix<double,3,_nsum> _R_foot_optition_optimal;
	Eigen::Matrix<double,3,_footstepsnumber> _foot_location_optimal;	
	
	
	
	// /////%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
	// //// for kMP swing generation: private variable for leg status storage	
	// /////%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
	
	// Eigen::Matrix<double, 1,10000> _Lfootx_kmp, _Lfooty_kmp,_Lfootz_kmp, _Lfootvx_kmp, _Lfootvy_kmp,_Lfootvz_kmp;
	// Eigen::Matrix<double, 1,10000> _Rfootx_kmp, _Rfooty_kmp,_Rfootz_kmp, _Rfootvx_kmp, _Rfootvy_kmp,_Rfootvz_kmp;

	
	///%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%5
	// other variables for step height+angular optimization
	///%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
	
	Eigen::RowVectorXd _zmpx_real, _zmpy_real;		
	Eigen::RowVectorXd _thetax, _thetavx, _thetaax;
	Eigen::RowVectorXd _thetay, _thetavy, _thetaay;	
	Eigen::RowVectorXd _thetaz, _thetavz, _thetaaz;	
	
	Eigen::RowVectorXd _torquex_real, _torquey_real;	
    
    Eigen::RowVectorXd _dcmx_real, _dcmy_real;
    
	
	// CoM+angular momentum state and contro input

	std::vector<Eigen::Vector3d> _footsteps;
	
	// initial parameters for MPC

	Eigen::MatrixXd _Hcom1;

	Eigen::VectorXd _ggg;

	
	Eigen::VectorXd _footx_real, _footy_real, _footz_real;
	Eigen::VectorXd _footx_real_next, _footy_real_next, _footz_real_next;
	Eigen::VectorXd _footx_real_next1, _footy_real_next1, _footz_real_next1;
	



	Eigen::MatrixXd _Lfootxyz, _Rfootxyz;
	
	
// predictive model control_tracking with time_varying height		
	Eigen::VectorXd _bjx, _tnx;
	
	Eigen::MatrixXd _v_i, _VV_i;
	
	int _n_vis;
	
	
	int xxx, xxx1,xxx2;
	

protected:
	void Rfooty_plan(int arg1);
// 	footx_ref_comx_feed(int i);
};






