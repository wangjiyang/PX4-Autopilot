/****************************************************************************
 *
 *   Copyright (c) 2026 PX4 Development Team. All rights reserved.
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
 ****************************************************************************/

/**
 * @file DroidRc.cpp
 *
 * Synthetic pilot input for the phone demo: publishes a centered-stick
 * manual_control_setpoint at 25 Hz (roll/pitch/yaw = 0, throttle = mid).
 * In MANUAL/STABILIZED mode this makes the attitude controller demand a
 * level attitude, so tilting the phone produces visible corrective motor
 * commands - which is exactly what the gimbal/motor demo UI shows.
 *
 * The phone has no RC receiver and no joystick; without this module the
 * mc control stack would sit idle for lack of pilot input.
 */

#include <px4_platform_common/px4_config.h>
#include <px4_platform_common/defines.h>
#include <px4_platform_common/module.h>
#include <px4_platform_common/posix.h>

#include <drivers/drv_hrt.h>
#include <uORB/Publication.hpp>
#include <uORB/topics/manual_control_setpoint.h>

class DroidRc : public ModuleBase<DroidRc>
{
public:
	DroidRc() = default;
	~DroidRc() override = default;

	static int task_spawn(int argc, char *argv[]);
	static DroidRc *instantiate(int argc, char *argv[]);
	static int custom_command(int argc, char *argv[]);
	static int print_usage(const char *reason = nullptr);

	void run() override;
	int print_status() override;

private:
	uORB::Publication<manual_control_setpoint_s> _manual_pub{ORB_ID(manual_control_setpoint)};
	unsigned _published{0};
};

void DroidRc::run()
{
	while (!should_exit()) {
		px4_usleep(40000); // 25 Hz

		manual_control_setpoint_s msp{};
		msp.timestamp_sample = hrt_absolute_time();
		msp.valid = true;
		msp.data_source = manual_control_setpoint_s::SOURCE_MAVLINK_0;
		msp.roll = 0.f;
		msp.pitch = 0.f;
		msp.yaw = 0.f;
		msp.throttle = 0.f; // mid stick = ~50% thrust in manual mode
		msp.sticks_moving = false;
		msp.timestamp = hrt_absolute_time();

		_manual_pub.publish(msp);
		_published++;
	}
}

int DroidRc::print_status()
{
	PX4_INFO("published %u centered-stick setpoints", _published);
	return 0;
}

int DroidRc::task_spawn(int argc, char *argv[])
{
	_task_id = px4_task_spawn_cmd("droid_rc",
				      SCHED_DEFAULT,
				      SCHED_PRIORITY_DEFAULT,
				      2000,
				      (px4_main_t)&run_trampoline,
				      (char *const *)argv);

	if (_task_id < 0) {
		_task_id = -1;
		return -errno;
	}

	return 0;
}

DroidRc *DroidRc::instantiate(int argc, char *argv[])
{
	return new DroidRc();
}

int DroidRc::custom_command(int argc, char *argv[])
{
	return print_usage("unknown command");
}

int DroidRc::print_usage(const char *reason)
{
	if (reason) {
		PX4_WARN("%s\n", reason);
	}

	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description
Publishes a centered-stick manual_control_setpoint (25 Hz) so the
multicopter control stack has pilot input on a phone without RC:
level-attitude demand, mid throttle.
)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("droid_rc", "driver");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();

	return 0;
}

extern "C" __EXPORT int droid_rc_main(int argc, char *argv[])
{
	return DroidRc::main(argc, argv);
}
