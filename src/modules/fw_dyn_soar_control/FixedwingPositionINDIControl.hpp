/**
 * Implementation of a generic incremental position controller 
 * using incremental nonlinear dynamic inversion and differential flatness
 * for a fixed wing aircraft during dynamic soaring cycles.
 * The controller directly outputs actuator deflections for ailerons, elevator and rudder.
 *
 * @author Marvin Harms <marv@teleport.ch>
 */

// use inclusion guards
#ifndef FIXEDWINGPOSITIONINDICONTROL_HPP_
#define FIXEDWINGPOSITIONINDICONTROL_HPP_

#include <float.h>

#include <sstream>
#include <vector>
#include <array>
#include <drivers/drv_hrt.h>
#include "fw_att_control/ecl_pitch_controller.h"
#include "fw_att_control/ecl_roll_controller.h"
#include "fw_att_control/ecl_wheel_controller.h"
#include "fw_att_control/ecl_yaw_controller.h"
#include <lib/ecl/geo/geo.h>
#include <lib/l1/ECL_L1_Pos_Controller.hpp>
#include <lib/landing_slope/Landingslope.hpp>
#include <lib/mathlib/mathlib.h>
#include <lib/perf/perf_counter.h>
#include <px4_platform_common/px4_config.h>
#include <px4_platform_common/defines.h>
#include <px4_platform_common/module.h>
#include <px4_platform_common/module_params.h>
#include <px4_platform_common/posix.h>
#include <px4_platform_common/px4_work_queue/WorkItem.hpp>
#include <uORB/Publication.hpp>
#include <uORB/PublicationMulti.hpp>
#include <uORB/Subscription.hpp>
#include <uORB/SubscriptionCallback.hpp>
#include <uORB/topics/vehicle_command.h>
#include <uORB/topics/vehicle_control_mode.h>
#include <uORB/topics/airspeed_validated.h>
#include <uORB/topics/airflow_aoa.h>
#include <uORB/topics/airflow_slip.h>
#include <uORB/topics/manual_control_setpoint.h>
#include <uORB/topics/parameter_update.h>
#include <uORB/topics/vehicle_air_data.h>
#include <uORB/topics/vehicle_attitude.h>
#include <uORB/topics/vehicle_attitude_setpoint.h>
#include <uORB/topics/vehicle_angular_velocity.h>
#include <uORB/topics/vehicle_rates_setpoint.h>
#include <uORB/topics/vehicle_angular_acceleration.h>
#include <uORB/topics/vehicle_angular_acceleration_setpoint.h>
#include <uORB/topics/vehicle_global_position.h>
#include <uORB/topics/vehicle_local_position.h>
#include <uORB/topics/vehicle_odometry.h>
#include <uORB/topics/home_position.h>
#include <uORB/topics/vehicle_acceleration.h>
#include <uORB/topics/vehicle_land_detected.h>
#include <uORB/topics/vehicle_local_position_setpoint.h>
#include <uORB/topics/rate_ctrl_status.h>
#include <uORB/topics/vehicle_status.h>
#include <uORB/topics/vehicle_status_flags.h>
#include <uORB/topics/actuator_controls.h>
#include <uORB/topics/soaring_controller_heartbeat.h>
#include <uORB/topics/soaring_controller_status.h>
#include <uORB/topics/offboard_control_mode.h>
#include <uORB/topics/wind.h>
#include <uORB/uORB.h>
#include <iostream>
#include <fstream>


using namespace time_literals;

using matrix::Dcmf;
using matrix::Eulerf;
using matrix::Quatf;
using matrix::Vector;
using matrix::Matrix3f;
using matrix::Vector3f;


class FixedwingPositionINDIControl final : public ModuleBase<FixedwingPositionINDIControl>, public ModuleParams,
	public px4::WorkItem
{
public:
	FixedwingPositionINDIControl();
	~FixedwingPositionINDIControl() override;

	/** @see ModuleBase */
	static int task_spawn(int argc, char *argv[]);

	/** @see ModuleBase */
	static int custom_command(int argc, char *argv[]);

	/** @see ModuleBase */
	static int print_usage(const char *reason = nullptr);

	bool init();

private:
	void Run() override;

	orb_advert_t	_mavlink_log_pub{nullptr};

	// make the main task run, whenever a new body rate becomes available
	uORB::SubscriptionCallbackWorkItem _vehicle_angular_velocity_sub{this, ORB_ID(vehicle_angular_velocity)};

	uORB::SubscriptionInterval _parameter_update_sub{ORB_ID(parameter_update), 1_s};

    // Subscriptions
	uORB::Subscription _vehicle_status_sub{ORB_ID(vehicle_status)};						// vehicle status
	uORB::Subscription _airspeed_validated_sub{ORB_ID(airspeed_validated)};             // airspeed 
    uORB::Subscription _airflow_aoa_sub{ORB_ID(airflow_aoa)};                           // angle of attack
    uORB::Subscription _airflow_slip_sub{ORB_ID(airflow_slip)};                         // angle of sideslip
    uORB::Subscription _vehicle_local_position_sub{ORB_ID(vehicle_local_position)};     // local NED position
    uORB::Subscription _home_position_sub{ORB_ID(home_position)};						// home position
    uORB::Subscription _vehicle_attitude_sub{ORB_ID(vehicle_attitude)};                 // vehicle attitude
    uORB::Subscription _vehicle_angular_acceleration_sub{ORB_ID(vehicle_angular_acceleration)}; // vehicle body accel
	uORB::Subscription _soaring_controller_status_sub{ORB_ID(soaring_controller_status)};			// vehicle status flags
	
    // Publishers
	uORB::Publication<actuator_controls_s>							_actuators_0_pub;
	uORB::Publication<vehicle_attitude_setpoint_s>					_attitude_sp_pub;
	uORB::Publication<vehicle_rates_setpoint_s>						_angular_vel_sp_pub{ORB_ID(vehicle_rates_setpoint)};
	uORB::Publication<vehicle_angular_acceleration_setpoint_s>		_angular_accel_sp_pub{ORB_ID(vehicle_angular_acceleration_setpoint)};
	uORB::PublicationMulti<rate_ctrl_status_s>						_rate_ctrl_status_pub{ORB_ID(rate_ctrl_status)};
	uORB::Publication<soaring_controller_heartbeat_s>				_soaring_controller_heartbeat_pub{ORB_ID(soaring_controller_status)};
	uORB::Publication<offboard_control_mode_s>						_offboard_control_mode_pub{ORB_ID(offboard_control_mode)};

    // Message structs
	vehicle_angular_acceleration_setpoint_s _angular_accel_sp {};
	actuator_controls_s			_actuators {};			// actuator commands
	manual_control_setpoint_s	_manual_control_setpoint {};			///< r/c channel data
	vehicle_local_position_s	_local_pos {};			///< vehicle local position
	vehicle_attitude_s			_attitude {};			///< vehicle attitude
	vehicle_attitude_setpoint_s	_attitude_sp {};		///< vehicle attitude setpoint
	vehicle_angular_velocity_s 	_angular_vel {};		///< vehicle angular velocity
	vehicle_rates_setpoint_s 	_angular_vel_sp {};		///< vehicle angular velocity setpoint
	vehicle_angular_acceleration_s	_angular_accel {};	///< vehicle angular acceleration
	home_position_s				_home_pos {};			///< home position
	map_projection_reference_s 	_global_local_proj_ref{};
	vehicle_control_mode_s		_control_mode {};		///< control mode
	offboard_control_mode_s		_offboard_control_mode {};	///< offboard control mode
	vehicle_status_s		    _vehicle_status {};		///< vehicle status
	soaring_controller_status_s	_soaring_controller_status {};	///< soaring controller status
	soaring_controller_heartbeat_s	_soaring_controller_heartbeat{};	///< soaring controller hrt

	// parameter struct
	DEFINE_PARAMETERS(
		// aircraft params
		(ParamFloat<px4::params::FW_INERTIA_ROLL>) _param_fw_inertia_roll,
		(ParamFloat<px4::params::FW_INERTIA_PITCH>) _param_fw_inertia_pitch,
		(ParamFloat<px4::params::FW_INERTIA_YAW>) _param_fw_inertia_yaw,
		(ParamFloat<px4::params::FW_MASS>) _param_fw_mass,
		(ParamFloat<px4::params::FW_WING_AREA>) _param_fw_wing_area,
		(ParamFloat<px4::params::RHO>) _param_rho,
		// aerodynamic params
		(ParamFloat<px4::params::C_L0>) _param_fw_c_l0,
		(ParamFloat<px4::params::C_L1>) _param_fw_c_l1,
		(ParamFloat<px4::params::C_D0>) _param_fw_c_d0,
		(ParamFloat<px4::params::C_D1>) _param_fw_c_d1,
		(ParamFloat<px4::params::C_D2>) _param_fw_c_d2,
		(ParamFloat<px4::params::AOA_OFFSET>) _param_aoa_offset,
		// filter params
		(ParamFloat<px4::params::FILTER_A1>) _param_filter_a1,
		(ParamFloat<px4::params::FILTER_A2>) _param_filter_a2,
		(ParamFloat<px4::params::FILTER_B1>) _param_filter_b1,
		(ParamFloat<px4::params::FILTER_B2>) _param_filter_b2,
		(ParamFloat<px4::params::FILTER_B3>) _param_filter_b3,
		// controller params
		(ParamFloat<px4::params::K_X_ROLL>) _param_k_x_roll,
		(ParamFloat<px4::params::K_X_PITCH>) _param_k_x_pitch,
		(ParamFloat<px4::params::K_X_YAW>) _param_k_x_yaw,
		(ParamFloat<px4::params::K_V_ROLL>) _param_k_v_roll,
		(ParamFloat<px4::params::K_V_PITCH>) _param_k_v_pitch,
		(ParamFloat<px4::params::K_V_YAW>) _param_k_v_yaw,
		(ParamFloat<px4::params::K_A_ROLL>) _param_k_a_roll,
		(ParamFloat<px4::params::K_A_PITCH>) _param_k_a_pitch,
		(ParamFloat<px4::params::K_A_YAW>) _param_k_a_yaw,
		(ParamFloat<px4::params::K_Q_ROLL>) _param_k_q_roll,
		(ParamFloat<px4::params::K_Q_PITCH>) _param_k_q_pitch,
		(ParamFloat<px4::params::K_Q_YAW>) _param_k_q_yaw,
		(ParamFloat<px4::params::K_W_ROLL>) _param_k_w_roll,
		(ParamFloat<px4::params::K_W_PITCH>) _param_k_w_pitch,
		(ParamFloat<px4::params::K_W_YAW>) _param_k_w_yaw,
		(ParamFloat<px4::params::K_ACT_ROLL>) _param_k_act_roll,
		(ParamFloat<px4::params::K_ACT_PITCH>) _param_k_act_pitch,
		(ParamFloat<px4::params::K_ACT_YAW>) _param_k_act_yaw,
		// location params
		(ParamFloat<px4::params::ORIGIN_LAT>) _param_origin_lat,
		(ParamFloat<px4::params::ORIGIN_LON>) _param_origin_lon,
		(ParamFloat<px4::params::ORIGIN_ALT>) _param_origin_alt

	)


	perf_counter_t	_loop_perf;				///< loop performance counter

	// estimator reset counters
	uint8_t _pos_reset_counter{0};				///< captures the number of times the estimator has reset the horizontal position
	uint8_t _alt_reset_counter{0};				///< captures the number of times the estimator has reset the altitude state

	
	// Update our local parameter cache.
	int		parameters_update();

	// Update subscriptions
	void        wind_poll();
	void		airspeed_poll();
	void		airflow_aoa_poll();
	void 		airflow_slip_poll();

	void		vehicle_local_position_poll();
	void		vehicle_attitude_poll();
	void		vehicle_angular_velocity_poll();
	void		vehicle_angular_acceleration_poll();
	
	void		control_update();
	void 		manual_control_setpoint_poll();
	void		vehicle_command_poll();
	void		vehicle_control_mode_poll();
	void		vehicle_status_poll();
	void		soaring_controller_status_poll();

	//
	void		status_publish();

	const int _num_points = 30;				// number of points on the precomputed trajectory
	const static size_t _num_basis_funs = 16;			// number of basis functions used for the trajectory approximation

	// controller methods
	
	void _read_trajectory_coeffs_csv(std::string filename);				// read in the correct coefficients of the appropriate trajectory
	void _set_wind_estimate(Vector3f wind);
	float _get_closest_t(Vector3f pos);				// get the normalized time, at which the reference path is closest to the current position
	Vector<float, _num_basis_funs> _get_basis_funs(float t=0);			// compute the vector of basis functions at normalized time t in [0,1]
	Vector<float, _num_basis_funs> _get_d_dt_basis_funs(float t=0);	// compute the vector of basis function gradients at normalized time t in [0,1]
	Vector<float, _num_basis_funs> _get_d2_dt2_basis_funs(float t=0);	// compute the vector of basis function curvatures at normalized time t in [0,1]
	void _load_basis_coefficients();		// load the coefficients of the current path approximation
	Vector3f _get_position_ref(float t=0);	// get the reference position on the current path, at normalized time t in [0,1]
	Vector3f _get_velocity_ref(float t=0, float T=1);	// get the reference velocity on the current path, at normalized time t in [0,1], with an intended cycle time of T
	Vector3f _get_acceleration_ref(float t=0, float T=1);	// get the reference acceleration on the current path, at normalized time t in [0,1], with an intended cycle time of T
	Quatf _get_attitude_ref(float t=0, float T=1);	// get the reference attitude on the current path, at normalized time t in [0,1], with an intended cycle time of T
	Vector3f _get_angular_velocity_ref(float t=0, float T=1);	// get the reference angular velocity on the current path, at normalized time t in [0,1], with an intended cycle time of T
	Vector3f _get_angular_acceleration_ref(float t=0, float T=1);	// get the reference angular acceleration on the current path, at normalized time t in [0,1], with an intended cycle time of T
	Quatf _get_attitude(Vector3f vel, Vector3f f);	// get the attitude to produce force f while flying with velocity vel
	Vector3f _compute_NDI_stage_1(Vector3f pos_ref, Vector3f vel_ref, Vector3f acc_ref, Vector3f omega_ref, Vector3f alpha_ref);
	Vector3f _compute_NDI_stage_2(Vector3f ctrl);
	Vector3f _apply_LP_filter(Vector3f new_input, Vector<Vector3f, 3>  &old_input, Vector<Vector3f, 2>  &old_output);
	Vector3f _compute_actuator_deflections(Vector3f ctrl);

	// control variables
	Vector<float, _num_basis_funs> _basis_coeffs_x = {};				// coefficients of the current path
	Vector<float, _num_basis_funs> _basis_coeffs_y = {};				// coefficients of the current path
	Vector<float, _num_basis_funs> _basis_coeffs_z = {};				// coefficients of the current path
	Vector3f _alpha_sp;
	Vector3f _wind_estimate;
	Matrix3f _K_x;
	Matrix3f _K_v;
	Matrix3f _K_a;
	Matrix3f _K_q;
	Matrix3f _K_w;
	Vector3f _pos;		// current position
	Vector3f _vel;		// current velocity
	Vector3f _acc;		// current acceleration
	Quatf _att;			// attitude quaternion
	Vector3f _omega;	// angular rate vector
	Vector3f _alpha;	// angular acceleration vector
	Matrix3f _K_actuators;	// diagonal actuator control gain matrix
	hrt_abstime _last_run{0};

	// filter variables
	Vector<Vector3f, 3> _f_list;	// force
	Vector<Vector3f, 3> _m_list;	// moment
	Vector<Vector3f, 3> _w_list;	// body rates
	Vector<Vector3f, 3> _a_list;	// linear accel
	Vector<Vector3f, 3> _l_list;	// angular accel
	Vector<Vector3f, 2> _f_lpf_list;
	Vector<Vector3f, 2> _m_lpf_list;
	Vector<Vector3f, 2> _w_lpf_list;	// body rates	
	Vector<Vector3f, 2> _a_lpf_list;
	Vector<Vector3f, 2> _l_lpf_list;

	// parameter variables
	Matrix3f _inertia {};
	float _mass;
	float _area;
	float _rho;
	float _C_L0;
	float _C_L1;
	float _C_D0;
	float _C_D1;
	float _C_D2;
	float _aoa_offset;
	float _a1;
	float _a2;
	float _b1;
	float _b2;
	float _b3;
	// trajecotry origin in WGS84
	float _origin_lat;
	float _origin_lon;
	float _origin_alt;
	// trajecotry origin in current NED local frame
	float _origin_N;
	float _origin_E;
	float _origin_D;

	bool _airspeed_valid{false};				///< flag if a valid airspeed estimate exists
	hrt_abstime _airspeed_last_valid{0};			///< last time airspeed was received. Used to detect timeouts.
	float _airspeed{0.0f};

	bool _aoa_valid{false};				///< flag if a valid AoA estimate exists
	hrt_abstime _aoa_last_valid{0};			///< last time Aoa was received. Used to detect timeouts.
	float _aoa{0.0f};

	bool _slip_valid{false};				///< flag if a valid AoA estimate exists
	hrt_abstime _slip_last_valid{0};			///< last time Aoa was received. Used to detect timeouts.
	float _slip{0.0f};

	// helper variables
	Dcmf _R_ned_to_enu;	// rotation matrix from NED to ENU frame
	Dcmf _R_enu_to_ned;	// rotation matrix from ENU to NED frame
	Vector3f _zero_crossing_local_pos;	// vector denoting the zero crossing of the trajectories in NED frame

};



#endif // FIXEDWINGPOSITIONINDICONTROL_HPP_