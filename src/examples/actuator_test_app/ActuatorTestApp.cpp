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

#include "ActuatorTestApp.hpp"

#include <drivers/drv_hrt.h>

using namespace time_literals;

ActuatorTestApp::ActuatorTestApp() :
	ModuleParams(nullptr),
	ScheduledWorkItem(MODULE_NAME, px4::wq_configurations::test1)
{
	parameters_update(true);
}

ActuatorTestApp::~ActuatorTestApp()
{
	perf_free(_loop_perf);
	perf_free(_loop_interval_perf);
}

bool ActuatorTestApp::init()
{
	ScheduleOnInterval(10_ms); // 10ms interval, 100Hz rate

	return true;
}

void ActuatorTestApp::parameters_update(bool force)
{
	// Check if parameters have changed
	if (_parameter_update_sub.updated() || force) {
		// clear update
		parameter_update_s param_update;
		_parameter_update_sub.copy(&param_update);

		ModuleParams::updateParams();

		_triggered = _param_ata_triggered.get();

		// actuator 0
		_step_count_0 = _param_ata_step_count_0.get();

		_enable_0 = false;

		if (_step_count_0 >= 0) {
			_enable_0 = true;
		}

		_step_duration_0 = fmaxf(_param_ata_step_duration_0.get(), 0.01f);
		_step_min_0 = math::constrain(_param_ata_step_min_0.get(), -1.0f, 1.0f);
		_step_max_0 = math::constrain(_param_ata_step_max_0.get(), _step_min_0, 1.0f);

		_delta_step_0 = 0.0f;

		if (_step_count_0 > 0) {
			_delta_step_0 = (_step_max_0 - _step_min_0) / _step_count_0;
		}

		// actuator 1
		_step_count_1 = _param_ata_step_count_1.get();

		_enable_1 = false;

		if (_step_count_1 >= 0) {
			_enable_1 = true;
		}

		_step_duration_1 = fmaxf(_param_ata_step_duration_1.get(), 0.01f);
		_step_min_1 = math::constrain(_param_ata_step_min_1.get(), -1.0f, 1.0f);
		_step_max_1 = math::constrain(_param_ata_step_max_1.get(), _step_min_1, 1.0f);

		_delta_step_1 = 0.0f;

		if (_step_count_1 > 0) {
			_delta_step_1 = (_step_max_1 - _step_min_1) / _step_count_1;
		}

	}
}

void ActuatorTestApp::Run()
{
	if (should_exit()) {
		ScheduleClear();
		exit_and_cleanup();
		return;
	}

	perf_begin(_loop_perf);
	perf_count(_loop_interval_perf);

	parameters_update(false);

	// update subscriptions
	_input_rc_sub.update(&_input_rc);	// update RC input

	// timing
	const hrt_abstime now = hrt_absolute_time();
	const float dt = math::constrain(((now - _last_run) / 1e6f), 0.0002f, 0.02f);
	_last_run = now;

	// control stuff via system parameters
	if (_triggered) {

		_timer_0 += dt;
		_timer_1 += dt;

		_u_0 = cos(_timer_0);
		_u_1 = sin(_timer_0);

	} else {

		_u_0 = _step_min_0;
		_timer_0 = 0.0f;

		_u_1 = _step_min_1;
		_timer_1 = 0.0f;
	}

	_actuators_0.control[0] = math::constrain(_u_0, -1.0f, 1.0f);
	_actuators_0.control[1] = math::constrain(_u_1, -1.0f, 1.0f);

	// control stuff via RC inputs
	if (!_input_rc.rc_lost) {
		_actuators_0.control[2] = (_input_rc.values[0] - 1500.0f) / 500.0f;
	}

	// publish actuator commands
	_actuators_0.timestamp = hrt_absolute_time();
	_actuators_0_pub.publish(_actuators_0);

	perf_end(_loop_perf);
}

int ActuatorTestApp::task_spawn(int argc, char *argv[])
{
	ActuatorTestApp *instance = new ActuatorTestApp();

	if (instance) {
		_object.store(instance);
		_task_id = task_id_is_work_queue;

		if (instance->init()) {
			return PX4_OK;
		}

	} else {
		PX4_ERR("alloc failed");
	}

	delete instance;
	_object.store(nullptr);
	_task_id = -1;

	return PX4_ERROR;
}

int ActuatorTestApp::print_status()
{
	perf_print_counter(_loop_perf);
	perf_print_counter(_loop_interval_perf);
	return 0;
}

int ActuatorTestApp::custom_command(int argc, char *argv[])
{
	return print_usage("unknown command");
}

int ActuatorTestApp::print_usage(const char *reason)
{
	if (reason) {
		PX4_WARN("%s\n", reason);
	}

	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description
Module to execute hardcoded actuator motion for testing purposes

)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("actuator_test_app", "testing");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();

	return 0;
}

extern "C" __EXPORT int actuator_test_app_main(int argc, char *argv[])
{
	return ActuatorTestApp::main(argc, argv);
}
