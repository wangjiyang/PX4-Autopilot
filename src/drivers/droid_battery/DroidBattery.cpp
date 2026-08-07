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
 * @file DroidBattery.cpp
 *
 * Battery bridge for Android: the hosting app reads the phone battery
 * (BatteryManager) and sends it here over local UDP; this module publishes
 * battery_status so the commander's low-battery warning/failsafe chain runs
 * against the real phone battery.
 *
 * The warning level is derived from the reported percentage using the
 * standard BAT_LOW_THR / BAT_CRIT_THR / BAT_EMERGEN_THR parameters
 * (fractions, e.g. 0.15), because the phone reports state-of-charge
 * directly - a voltage-curve estimate would be redundant and worse.
 */

#include <px4_platform_common/px4_config.h>
#include <px4_platform_common/defines.h>
#include <px4_platform_common/module.h>
#include <px4_platform_common/posix.h>

#include <drivers/drv_hrt.h>
#include <parameters/param.h>
#include <uORB/Publication.hpp>
#include <uORB/topics/battery_status.h>

#include <arpa/inet.h>
#include <math.h>
#include <netinet/in.h>
#include <string.h>
#include <sys/socket.h>
#include <unistd.h>

static constexpr uint16_t DROID_BATT_PORT = 15551;

#pragma pack(push, 1)
struct droid_batt_pkt {
	uint8_t magic0;     //  0 'D'
	uint8_t magic1;     //  1 'B'
	uint8_t version;    //  2 = 1
	uint8_t charging;   //  3 0/1
	float   voltage_v;  //  4
	float   current_a;  //  8 positive = discharging, -1 = unknown
	float   pct;        // 12 state of charge 0..100
	float   temp_c;     // 16 NAN = unknown
};
#pragma pack(pop)

static_assert(sizeof(droid_batt_pkt) == 20, "droid_batt_pkt must be 20 bytes");

class DroidBattery : public ModuleBase<DroidBattery>
{
public:
	DroidBattery() = default;
	~DroidBattery() override = default;

	static int task_spawn(int argc, char *argv[]);
	static DroidBattery *instantiate(int argc, char *argv[]);
	static int custom_command(int argc, char *argv[]);
	static int print_usage(const char *reason = nullptr);

	void run() override;
	int print_status() override;

private:
	int _fd{-1};
	unsigned _packet_count{0};
	unsigned _publish_count{0};
	unsigned _reject_count{0};
	uint8_t _last_warning{battery_status_s::BATTERY_WARNING_NONE};

	// keepalive state: commander latches battery_unhealthy if the newest
	// battery_status is >5 s old while connected=true
	battery_status_s _last_batt{};
	bool _have_last{false};
	hrt_abstime _last_rx{0};
	bool _disconnect_sent{false};

	uORB::Publication<battery_status_s> _batt_pub{ORB_ID(battery_status)};
};

void DroidBattery::run()
{
	_fd = socket(AF_INET, SOCK_DGRAM, 0);

	if (_fd < 0) {
		PX4_ERR("socket failed: %s", strerror(errno));
		return;
	}

	struct sockaddr_in addr {};

	addr.sin_family = AF_INET;

	addr.sin_addr.s_addr = htonl(INADDR_LOOPBACK);

	addr.sin_port = htons(DROID_BATT_PORT);

	if (bind(_fd, (struct sockaddr *)&addr, sizeof(addr)) != 0) {
		PX4_ERR("bind 127.0.0.1:%u failed: %s", DROID_BATT_PORT, strerror(errno));
		close(_fd);
		_fd = -1;
		return;
	}

	struct timeval tv {};

	tv.tv_sec = 1;

	setsockopt(_fd, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));

	// warning thresholds as remaining fraction
	float low_thr = 0.15f, crit_thr = 0.07f, emergen_thr = 0.05f;
	param_get(param_find("BAT_LOW_THR"), &low_thr);
	param_get(param_find("BAT_CRIT_THR"), &crit_thr);
	param_get(param_find("BAT_EMERGEN_THR"), &emergen_thr);

	PX4_INFO("listening on udp://127.0.0.1:%u (thresholds %.2f/%.2f/%.2f)",
		 DROID_BATT_PORT, (double)low_thr, (double)crit_thr, (double)emergen_thr);

	while (!should_exit()) {
		droid_batt_pkt pkt;
		ssize_t n = recv(_fd, &pkt, sizeof(pkt), MSG_TRUNC);

		if (n < 0) {
			if (errno == EAGAIN || errno == EWOULDBLOCK || errno == EINTR) {
				// recv timeout. A silent Java feed must not latch commander's
				// battery_unhealthy (>5 s stale while connected=true): bridge
				// short gaps by republishing the last state, then declare a
				// deliberate disconnect after 30 s.
				if (_have_last) {
					const hrt_abstime since = hrt_absolute_time() - _last_rx;

					if (since < 30 * 1000000UL) {
						_last_batt.timestamp = hrt_absolute_time();
						_batt_pub.publish(_last_batt);

					} else if (!_disconnect_sent) {
						PX4_WARN("battery feed lost for 30 s, reporting disconnected");
						_last_batt.connected = false;
						_last_batt.warning = battery_status_s::BATTERY_WARNING_NONE;
						_last_batt.timestamp = hrt_absolute_time();
						_batt_pub.publish(_last_batt);
						_disconnect_sent = true;
					}
				}

				continue; // then re-check should_exit()
			}

			PX4_ERR("recv failed: %s", strerror(errno));
			break;
		}

		if (n != sizeof(pkt) || pkt.magic0 != 'D' || pkt.magic1 != 'B' || pkt.version != 1) {
			_reject_count++;
			continue;
		}

		_packet_count++;

		if (!(pkt.pct >= 0.f && pkt.pct <= 100.f) || !(pkt.voltage_v >= 0.f && pkt.voltage_v < 60.f)) {
			_reject_count++;
			continue;
		}

		const float remaining = pkt.pct / 100.f;

		// NaN/garbage current must not propagate into battery_status
		const float current = PX4_ISFINITE(pkt.current_a) ? pkt.current_a : -1.f;

		battery_status_s batt{};
		batt.connected = true;
		batt.voltage_v = pkt.voltage_v;
		batt.current_a = current;
		batt.current_average_a = current;
		batt.remaining = remaining;
		batt.scale = 1.f;
		batt.time_remaining_s = NAN;
		batt.temperature = pkt.temp_c;
		batt.cell_count = 1;
		batt.source = battery_status_s::BATTERY_SOURCE_EXTERNAL;
		batt.id = 1;

		if (remaining <= emergen_thr) {
			batt.warning = battery_status_s::BATTERY_WARNING_EMERGENCY;

		} else if (remaining <= crit_thr) {
			batt.warning = battery_status_s::BATTERY_WARNING_CRITICAL;

		} else if (remaining <= low_thr) {
			batt.warning = battery_status_s::BATTERY_WARNING_LOW;

		} else {
			batt.warning = battery_status_s::BATTERY_WARNING_NONE;
		}

		if (batt.warning != _last_warning) {
			PX4_INFO("phone battery %.0f%% -> warning level %u", (double)pkt.pct, batt.warning);
			_last_warning = batt.warning;
		}

		batt.timestamp = hrt_absolute_time();
		_batt_pub.publish(batt);
		_publish_count++;
		_last_batt = batt;
		_have_last = true;
		_last_rx = batt.timestamp;
		_disconnect_sent = false;
	}

	close(_fd);
	_fd = -1;
}

int DroidBattery::print_status()
{
	PX4_INFO("packets: %u published: %u rejected: %u (udp 127.0.0.1:%u)",
		 _packet_count, _publish_count, _reject_count, DROID_BATT_PORT);
	return 0;
}

int DroidBattery::task_spawn(int argc, char *argv[])
{
	_task_id = px4_task_spawn_cmd("droid_battery",
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

DroidBattery *DroidBattery::instantiate(int argc, char *argv[])
{
	return new DroidBattery();
}

int DroidBattery::custom_command(int argc, char *argv[])
{
	return print_usage("unknown command");
}

int DroidBattery::print_usage(const char *reason)
{
	if (reason) {
		PX4_WARN("%s\n", reason);
	}

	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description
Publishes battery_status from the phone battery: the hosting Android app
sends BatteryManager readings over local UDP (127.0.0.1:15551).
)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("droid_battery", "driver");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();

	return 0;
}

extern "C" __EXPORT int droid_battery_main(int argc, char *argv[])
{
	return DroidBattery::main(argc, argv);
}
