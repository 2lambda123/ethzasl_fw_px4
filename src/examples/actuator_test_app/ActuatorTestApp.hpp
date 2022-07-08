/****************************************************************************
 *
 *   Copyright (c) 2020 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

#pragma once

#include <lib/perf/perf_counter.h>

#include <px4_platform_common/defines.h>
#include <px4_platform_common/module.h>
#include <px4_platform_common/module_params.h>
#include <px4_platform_common/posix.h>
#include <px4_platform_common/px4_work_queue/ScheduledWorkItem.hpp>

#include <mathlib/math/Limits.hpp>

#include <uORB/Publication.hpp>
#include <uORB/Subscription.hpp>
#include <uORB/SubscriptionCallback.hpp>

#include <uORB/topics/parameter_update.h>
#include <uORB/topics/input_rc.h>
#include <uORB/topics/actuator_controls.h>
#include <uORB/topics/orb_test.h>
#include <uORB/topics/vehicle_air_data.h>
#include <uORB/topics/adc_report.h>

enum FAREWELL_MODE {
	FAREWELL_MODE_IDLE,
	FAREWELL_MODE_SOARING,
	FAREWELL_MODE_THERMAL
};

class ActuatorTestApp : public ModuleBase<ActuatorTestApp>, public ModuleParams, public px4::ScheduledWorkItem
{
public:
	ActuatorTestApp();
	~ActuatorTestApp() override;

	/** @see ModuleBase */
	static int task_spawn(int argc, char *argv[]);

	/** @see ModuleBase */
	static int custom_command(int argc, char *argv[]);

	/** @see ModuleBase */
	static int print_usage(const char *reason = nullptr);

	bool init();

	int print_status() override;

private:
	void Run() override;

	void parameters_update(bool force);

	FAREWELL_MODE setCurrentMode(hrt_abstime time, const FAREWELL_MODE current_mode);

	// messages we listen to
	uORB::SubscriptionInterval _parameter_update_sub{ORB_ID(parameter_update)};
	uORB::Subscription _input_rc_sub{ORB_ID(input_rc)};
	uORB::Subscription _adc_report_sub{ORB_ID(adc_report)};
	uORB::Subscription _vehicle_air_data_sub{ORB_ID(vehicle_air_data)};

	// messages we publish to
	uORB::Publication<actuator_controls_s>	_actuators_0_pub{ORB_ID(actuator_controls_0)};

	perf_counter_t	_loop_perf{perf_alloc(PC_ELAPSED, MODULE_NAME": cycle")};
	perf_counter_t	_loop_interval_perf{perf_alloc(PC_INTERVAL, MODULE_NAME": interval")};

	hrt_abstime _last_run{0};
	hrt_abstime _last_modechange{0};

	// pub-sub message structs
	actuator_controls_s _actuators_0 {};
	input_rc_s _input_rc {};

	bool _triggered{false};

	int _step_count_0{0};
	float _step_duration_0{0.0f};
	float _step_min_0{0.0f};
	float _step_max_0{0.0f};

	bool _enable_0{false};
	float _timer_0{0.0f};
	float _delta_step_0{0.0f};
	float _u_0{0.0f};
	float _previous_baro_pressure{0.0};
	float _previous_adc_reading{0.0};

	int _step_count_1{0};
	float _step_duration_1{0.0f};
	float _step_min_1{0.0f};
	float _step_max_1{0.0f};

	bool _enable_1{false};
	float _timer_1{0.0f};
	float _delta_step_1{0.0f};
	float _u_1{0.0f};

	FAREWELL_MODE _control_mode_current{FAREWELL_MODE_THERMAL}; // used to check if the mode has changed

	DEFINE_PARAMETERS(
		(ParamInt<px4::params::ATA_TRIGGERED>) _param_ata_triggered,
		(ParamInt<px4::params::ATA_STP_COUNT_0>) _param_ata_step_count_0,
		(ParamFloat<px4::params::ATA_STP_DUR_0>) _param_ata_step_duration_0,
		(ParamFloat<px4::params::ATA_STP_MIN_0>) _param_ata_step_min_0,
		(ParamFloat<px4::params::ATA_STP_MAX_0>) _param_ata_step_max_0,
		(ParamInt<px4::params::ATA_STP_COUNT_1>) _param_ata_step_count_1,
		(ParamFloat<px4::params::ATA_STP_DUR_1>) _param_ata_step_duration_1,
		(ParamFloat<px4::params::ATA_STP_MIN_1>) _param_ata_step_min_1,
		(ParamFloat<px4::params::ATA_STP_MAX_1>) _param_ata_step_max_1
	)
};
