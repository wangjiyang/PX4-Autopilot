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
 * @file DroidStatus.cpp
 *
 * Debug/status telemetry for the hosting Android app: periodically packs
 * the flight controller's key state (attitude, rates, sensor sample rates,
 * GPS, health flags, control outputs) into a fixed 144-byte little-endian
 * struct (v2) sent to local UDP 127.0.0.1:14560, where the app's debug UI
 * renders it.
 *
 * Layout must byte-for-byte match the Status class in MainActivity.java.
 */

#include <px4_platform_common/px4_config.h>
#include <px4_platform_common/defines.h>
#include <px4_platform_common/module.h>
#include <px4_platform_common/posix.h>

#include <drivers/drv_hrt.h>
#include <matrix/math.hpp>

#include <uORB/Subscription.hpp>
#include <uORB/topics/actuator_motors.h>
#include <uORB/topics/sensor_accel.h>
#include <uORB/topics/sensor_gps.h>
#include <uORB/topics/sensor_gyro.h>
#include <uORB/topics/sensor_mag.h>
#include <uORB/topics/vehicle_angular_velocity.h>
#include <uORB/topics/vehicle_attitude.h>
#include <uORB/topics/vehicle_status.h>
#include <uORB/topics/vehicle_thrust_setpoint.h>
#include <uORB/topics/vehicle_torque_setpoint.h>

#include <arpa/inet.h>
#include <errno.h>
#include <netinet/in.h>
#include <string.h>
#include <sys/socket.h>
#include <unistd.h>

static constexpr uint16_t DROID_STATUS_PORT = 14560;

#pragma pack(push, 1)
struct droid_status_pkt {
	uint8_t  magic0;          //   0 'D'
	uint8_t  magic1;          //   1 'S'
	uint8_t  version;         //   2 = 1
	uint8_t  armed;           //   3
	uint32_t seq;             //   4
	uint64_t timestamp_us;    //   8
	float    roll;            //  16 rad
	float    pitch;           //  20
	float    yaw;             //  24
	float    rollspeed;       //  28 rad/s
	float    pitchspeed;      //  32
	float    yawspeed;        //  36
	float    acc[3];          //  40 m/s^2 (latest sensor_accel)
	float    mag[3];          //  52 gauss
	float    accel_rate_hz;   //  64
	float    gyro_rate_hz;    //  68
	float    mag_rate_hz;     //  72
	float    gps_rate_hz;     //  76
	double   lat_deg;         //  80
	double   lon_deg;         //  88
	float    alt_m;           //  96
	float    eph;             // 100
	uint8_t  fix_type;        // 104
	uint8_t  satellites_used; // 105
	uint8_t  attitude_valid;  // 106 fresh vehicle_attitude (<500 ms)
	uint8_t  preflight_pass;  // 107 vehicle_status.pre_flight_checks_pass
	uint8_t  nav_state;       // 108
	uint8_t  arming_state;    // 109
	uint16_t reserved;        // 110
	// --- version 2: control chain outputs (gimbal/motor demo) ---
	float    torque_sp[3];    // 112 normalized torque setpoint (roll/pitch/yaw)
	float    thrust_sp;       // 124 normalized collective thrust demand (0..1, up)
	float    motor[4];        // 128 actuator_motors control[0..3], 0..1 (-1 = stopped)
};
#pragma pack(pop)

static_assert(sizeof(droid_status_pkt) == 144, "droid_status_pkt must be 144 bytes");

class DroidStatus : public ModuleBase<DroidStatus>
{
public:
	DroidStatus() = default;
	~DroidStatus() override = default;

	static int task_spawn(int argc, char *argv[]);
	static DroidStatus *instantiate(int argc, char *argv[]);
	static int custom_command(int argc, char *argv[]);
	static int print_usage(const char *reason = nullptr);

	void run() override;
	int print_status() override;

private:
	uORB::Subscription _att_sub{ORB_ID(vehicle_attitude)};
	uORB::Subscription _angvel_sub{ORB_ID(vehicle_angular_velocity)};
	uORB::Subscription _accel_sub{ORB_ID(sensor_accel)};
	uORB::Subscription _gyro_sub{ORB_ID(sensor_gyro)};
	uORB::Subscription _mag_sub{ORB_ID(sensor_mag)};
	uORB::Subscription _gps_sub{ORB_ID(sensor_gps)};
	uORB::Subscription _vstatus_sub{ORB_ID(vehicle_status)};
	uORB::Subscription _torque_sub{ORB_ID(vehicle_torque_setpoint)};
	uORB::Subscription _thrust_sub{ORB_ID(vehicle_thrust_setpoint)};
	uORB::Subscription _motors_sub{ORB_ID(actuator_motors)};

	unsigned _sent{0};
	unsigned _send_errors{0};
};

void DroidStatus::run()
{
	int fd = socket(AF_INET, SOCK_DGRAM, 0);

	if (fd < 0) {
		PX4_ERR("socket failed: %s", strerror(errno));
		return;
	}

	struct sockaddr_in dst {};

	dst.sin_family = AF_INET;

	dst.sin_addr.s_addr = htonl(INADDR_LOOPBACK);

	dst.sin_port = htons(DROID_STATUS_PORT);

	droid_status_pkt pkt{};
	pkt.magic0 = 'D';
	pkt.magic1 = 'S';
	pkt.version = 2;

	for (int i = 0; i < 4; i++) {
		pkt.motor[i] = -1.f; // stopped until actuator_motors says otherwise
	}

	// sample counters for rate estimation over a 1 s window
	unsigned accel_cnt = 0, gyro_cnt = 0, mag_cnt = 0, gps_cnt = 0;
	hrt_abstime window_start = hrt_absolute_time();
	hrt_abstime last_att_time = 0;

	vehicle_attitude_s att{};
	vehicle_angular_velocity_s angvel{};
	sensor_accel_s accel{};
	sensor_mag_s mag{};
	sensor_gps_s gps{};
	vehicle_status_s vstatus{};

	unsigned iter = 0;

	while (!should_exit()) {
		// Drain at 50 Hz: readers snapped to the queue tail can pick up at
		// most ORB_QUEUE_LENGTH (8 for accel/gyro) messages per cycle, so a
		// 10 Hz drain would cap the measured rates at 80 Hz. Send at 10 Hz.
		px4_usleep(20000);
		iter++;

		// drain subscriptions, counting samples for the rate window
		while (_accel_sub.update(&accel)) { accel_cnt++; }

		sensor_gyro_s gyro;

		while (_gyro_sub.update(&gyro)) { gyro_cnt++; }

		while (_mag_sub.update(&mag)) { mag_cnt++; }

		while (_gps_sub.update(&gps)) { gps_cnt++; }

		if (_att_sub.update(&att)) {
			last_att_time = hrt_absolute_time();
		}

		_angvel_sub.update(&angvel);
		_vstatus_sub.update(&vstatus);

		vehicle_torque_setpoint_s torque;

		if (_torque_sub.update(&torque)) {
			pkt.torque_sp[0] = torque.xyz[0];
			pkt.torque_sp[1] = torque.xyz[1];
			pkt.torque_sp[2] = torque.xyz[2];
		}

		vehicle_thrust_setpoint_s thrust;

		if (_thrust_sub.update(&thrust)) {
			// NED body frame: hover thrust points up = -Z
			pkt.thrust_sp = -thrust.xyz[2];
		}

		actuator_motors_s motors;

		if (_motors_sub.update(&motors)) {
			for (int i = 0; i < 4; i++) {
				// NaN = motor stopped (disarmed) - report as -1
				pkt.motor[i] = PX4_ISFINITE(motors.control[i]) ? motors.control[i] : -1.f;
			}
		}

		if (iter % 5 != 0) {
			continue;
		}

		const hrt_abstime now = hrt_absolute_time();
		const float window_s = (now - window_start) / 1e6f;

		if (window_s >= 1.f) {
			pkt.accel_rate_hz = accel_cnt / window_s;
			pkt.gyro_rate_hz = gyro_cnt / window_s;
			pkt.mag_rate_hz = mag_cnt / window_s;
			pkt.gps_rate_hz = gps_cnt / window_s;
			accel_cnt = gyro_cnt = mag_cnt = gps_cnt = 0;
			window_start = now;
		}

		const matrix::Eulerf euler(matrix::Quatf(att.q));
		pkt.roll = euler.phi();
		pkt.pitch = euler.theta();
		pkt.yaw = euler.psi();
		pkt.rollspeed = angvel.xyz[0];
		pkt.pitchspeed = angvel.xyz[1];
		pkt.yawspeed = angvel.xyz[2];
		pkt.acc[0] = accel.x;
		pkt.acc[1] = accel.y;
		pkt.acc[2] = accel.z;
		pkt.mag[0] = mag.x;
		pkt.mag[1] = mag.y;
		pkt.mag[2] = mag.z;
		pkt.lat_deg = gps.latitude_deg;
		pkt.lon_deg = gps.longitude_deg;
		pkt.alt_m = (float)gps.altitude_msl_m;
		pkt.eph = gps.eph;
		pkt.fix_type = gps.fix_type;
		pkt.satellites_used = gps.satellites_used;
		pkt.attitude_valid = (last_att_time != 0) && (now - last_att_time < 500000);
		pkt.preflight_pass = vstatus.pre_flight_checks_pass;
		pkt.nav_state = vstatus.nav_state;
		pkt.arming_state = vstatus.arming_state;
		pkt.armed = (vstatus.arming_state == vehicle_status_s::ARMING_STATE_ARMED);
		pkt.timestamp_us = now;
		pkt.seq++;

		if (sendto(fd, &pkt, sizeof(pkt), 0, (struct sockaddr *)&dst, sizeof(dst)) == (ssize_t)sizeof(pkt)) {
			_sent++;

		} else {
			if (_send_errors == 0) {
				PX4_WARN("sendto failed: %s", strerror(errno));
			}

			_send_errors++;
		}
	}

	close(fd);
}

int DroidStatus::print_status()
{
	PX4_INFO("packets sent: %u, send errors: %u (udp 127.0.0.1:%u)",
		 _sent, _send_errors, DROID_STATUS_PORT);
	return 0;
}

int DroidStatus::task_spawn(int argc, char *argv[])
{
	_task_id = px4_task_spawn_cmd("droid_status",
				      SCHED_DEFAULT,
				      SCHED_PRIORITY_DEFAULT,
				      2600,
				      (px4_main_t)&run_trampoline,
				      (char *const *)argv);

	if (_task_id < 0) {
		_task_id = -1;
		return -errno;
	}

	return 0;
}

DroidStatus *DroidStatus::instantiate(int argc, char *argv[])
{
	return new DroidStatus();
}

int DroidStatus::custom_command(int argc, char *argv[])
{
	return print_usage("unknown command");
}

int DroidStatus::print_usage(const char *reason)
{
	if (reason) {
		PX4_WARN("%s\n", reason);
	}

	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description
Streams flight controller status (attitude, rates, sensor sample rates,
GPS, health flags) to the hosting Android app's debug UI over local UDP
(127.0.0.1:14560) at 10 Hz.
)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("droid_status", "driver");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();

	return 0;
}

extern "C" __EXPORT int droid_status_main(int argc, char *argv[])
{
	return DroidStatus::main(argc, argv);
}
