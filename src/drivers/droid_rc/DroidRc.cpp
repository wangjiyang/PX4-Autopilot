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
 * Pilot input for the phone: publishes manual_control_setpoint at 25 Hz.
 *
 * Two sources, auto-selected:
 *  - external: a GCS joystick (QGroundControl "virtual joystick" sends
 *    MAVLink MANUAL_CONTROL; mavlink_receiver publishes it on
 *    manual_control_input). Fresh external samples are passed through
 *    unchanged, so a phone + QGC is a complete pilot loop.
 *  - internal fallback: centered sticks (roll/pitch/yaw = 0, throttle mid),
 *    tagged SOURCE_RC so the UI can tell the two apart. In MANUAL mode this
 *    demands a level attitude - the bench gimbal/motor demo.
 *
 * The "land" command (virtual throttle ramp) overrides the throttle of
 * either source: the landing button must always win.
 */

#include <px4_platform_common/px4_config.h>
#include <px4_platform_common/defines.h>
#include <px4_platform_common/module.h>
#include <px4_platform_common/posix.h>

#include <drivers/drv_hrt.h>
#include <lib/geo/geo.h>
#include <math.h>
#include <matrix/math.hpp>
#include <stdlib.h>
#include <string.h>
#include <uORB/Publication.hpp>
#include <uORB/Subscription.hpp>
#include <uORB/topics/manual_control_setpoint.h>
#include <uORB/topics/orbit_status.h>
#include <uORB/topics/vehicle_attitude.h>
#include <uORB/topics/vehicle_command.h>
#include <uORB/topics/vehicle_local_position.h>
#include <uORB/topics/vehicle_status.h>

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
	static constexpr float LAND_RAMP_S = 5.f;          // throttle mid -> min
	static constexpr uint64_t EXT_TIMEOUT_US = 800000; // external joystick considered lost

	uORB::Publication<manual_control_setpoint_s> _manual_pub{ORB_ID(manual_control_setpoint)};
	// external pilot input (QGC virtual joystick via mavlink_receiver)
	uORB::Subscription _input_subs[3] {
		{ORB_ID(manual_control_input), 0},
		{ORB_ID(manual_control_input), 1},
		{ORB_ID(manual_control_input), 2},
	};
	manual_control_setpoint_s _ext{};
	bool _ext_seen{false};
	// sticks_moving is normally computed by the manual_control module (not
	// running here); commander's COM_RC_OVERRIDE stick-takeover from auto
	// modes depends on it, so derive it from consecutive external samples
	manual_control_setpoint_s _ext_prev{};
	hrt_abstime _last_stick_move{0};
	unsigned _published{0};
	unsigned _published_ext{0};

	// "landing profile": ramp the virtual throttle stick down (real AUTO_LAND
	// needs a height estimate the phone doesn't have indoors)
	volatile bool _land_requested{false};
	hrt_abstime _land_start{0};
};

void DroidRc::run()
{
	while (!should_exit()) {
		px4_usleep(40000); // 25 Hz

		const hrt_abstime now = hrt_absolute_time();

		// latest external joystick sample, if any
		for (auto &sub : _input_subs) {
			manual_control_setpoint_s in;

			while (sub.update(&in)) {
				if (in.valid && in.data_source != manual_control_setpoint_s::SOURCE_RC
				    && in.timestamp_sample > _ext.timestamp_sample) {
					const float delta = fabsf(in.roll - _ext_prev.roll)
							    + fabsf(in.pitch - _ext_prev.pitch)
							    + fabsf(in.yaw - _ext_prev.yaw)
							    + fabsf(in.throttle - _ext_prev.throttle);

					if (_ext_seen && delta > 0.02f) {
						_last_stick_move = in.timestamp_sample;
					}

					_ext_prev = in;
					_ext = in;
					_ext_seen = true;
				}
			}
		}

		const bool ext_fresh = _ext_seen && (now - _ext.timestamp_sample) < EXT_TIMEOUT_US;

		manual_control_setpoint_s msp{};

		if (ext_fresh) {
			// pass the GCS joystick through (note: between GCS messages the
			// last sample is re-published with fresh timestamps for up to
			// EXT_TIMEOUT_US - a replay, indistinguishable from a held stick)
			msp = _ext;
			// hold "moving" ~500 ms past the last real movement (hysteresis
			// like the upstream manual_control module)
			msp.sticks_moving = (_last_stick_move != 0) && (now - _last_stick_move < 500000);

		} else {
			// centered-stick fallback, tagged SOURCE_RC so consumers can
			// tell "internal hover demand" from a real joystick
			msp.data_source = manual_control_setpoint_s::SOURCE_RC;
			msp.roll = 0.f;
			msp.pitch = 0.f;
			msp.yaw = 0.f;
			msp.throttle = 0.f; // mid stick = ~50% thrust in manual mode
			msp.sticks_moving = false;
		}

		if (_land_requested) {
			if (_land_start == 0) {
				_land_start = now;
			}

			float t = (now - _land_start) / (LAND_RAMP_S * 1e6f);

			if (t > 1.f) {
				t = 1.f;
			}

			msp.throttle = -t; // ramp mid (0) -> min (-1), overrides any source

		} else {
			_land_start = 0;
		}

		msp.valid = true;
		msp.timestamp_sample = now;
		msp.timestamp = now;

		_manual_pub.publish(msp);
		_published++;

		if (ext_fresh) {
			_published_ext++;
		}
	}
}

int DroidRc::print_status()
{
	PX4_INFO("published %u setpoints (%u passed through from external joystick)",
		 _published, _published_ext);
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
	if (argc > 0 && is_running()) {
		if (!strcmp(argv[0], "land")) {
			get_instance()->_land_requested = true;
			PX4_INFO("landing profile: throttle ramping down over %.0f s", (double)LAND_RAMP_S);
			return 0;
		}

		if (!strcmp(argv[0], "hover")) {
			get_instance()->_land_requested = false;
			PX4_INFO("throttle back to hover (mid stick)");
			return 0;
		}

		// orbit [radius_m] [speed_mps]: circle a point <radius> ahead of the
		// nose, front locked to the circle center (the "orbit a building
		// with the camera on it" demo). Real FlightTaskOrbit does the flying.
		if (!strcmp(argv[0], "orbit")) {
			const float radius = (argc > 1) ? atof(argv[1]) : 8.f;
			const float speed = (argc > 2) ? atof(argv[2]) : 2.f;

			uORB::Subscription lpos_sub{ORB_ID(vehicle_local_position)};
			vehicle_local_position_s lpos{};

			if (!lpos_sub.copy(&lpos) || !lpos.xy_valid || !lpos.xy_global) {
				PX4_ERR("orbit needs a valid global local-position reference");
				return 1;
			}

			uORB::Subscription att_sub{ORB_ID(vehicle_attitude)};
			vehicle_attitude_s att{};

			if (!att_sub.copy(&att)) {
				PX4_ERR("orbit: no attitude");
				return 1;
			}

			const float yaw = matrix::Eulerf(matrix::Quatf(att.q)).psi();
			const float center_n = lpos.x + radius * cosf(yaw);
			const float center_e = lpos.y + radius * sinf(yaw);

			MapProjection proj(lpos.ref_lat, lpos.ref_lon);
			double lat = 0.0, lon = 0.0;
			proj.reproject(center_n, center_e, lat, lon);

			uORB::Subscription vstatus_sub{ORB_ID(vehicle_status)};
			vehicle_status_s vstatus{};
			vstatus_sub.copy(&vstatus);

			vehicle_command_s cmd{};
			cmd.command = vehicle_command_s::VEHICLE_CMD_DO_ORBIT;
			cmd.param1 = radius;
			cmd.param2 = speed;
			cmd.param3 = orbit_status_s::ORBIT_YAW_BEHAVIOUR_HOLD_FRONT_TO_CIRCLE_CENTER;
			cmd.param4 = NAN;
			cmd.param5 = lat;
			cmd.param6 = lon;
			cmd.param7 = lpos.ref_alt - lpos.z; // hold current altitude (AMSL)
			cmd.target_system = vstatus.system_id;
			cmd.target_component = vstatus.component_id;
			cmd.source_system = vstatus.system_id;
			cmd.source_component = vstatus.component_id;
			cmd.from_external = false;
			cmd.timestamp = hrt_absolute_time();

			uORB::Publication<vehicle_command_s> cmd_pub{ORB_ID(vehicle_command)};
			cmd_pub.publish(cmd);

			PX4_INFO("orbit: center %.1fm ahead (N %.1f E %.1f), r=%.1fm v=%.1fm/s",
				 (double)radius, (double)center_n, (double)center_e,
				 (double)radius, (double)speed);
			return 0;
		}
	}

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
	PRINT_MODULE_USAGE_COMMAND_DESCR("land", "ramp virtual throttle to minimum (landing profile)");
	PRINT_MODULE_USAGE_COMMAND_DESCR("hover", "virtual throttle back to mid stick");
	PRINT_MODULE_USAGE_COMMAND_DESCR("orbit", "orbit a point ahead: orbit [radius_m] [speed_mps]");
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();

	return 0;
}

extern "C" __EXPORT int droid_rc_main(int argc, char *argv[])
{
	return DroidRc::main(argc, argv);
}
