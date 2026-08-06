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
 * @file DroidGps.cpp
 *
 * GNSS bridge for Android: listens on a local UDP port for fix packets
 * sent by the Java side (android.location.LocationManager) and publishes
 * them as sensor_gps.
 *
 * Packet format: 64 bytes, little-endian, packed — must match
 * GpsBridge.java in the flyphone app.
 */

#include <px4_platform_common/px4_config.h>
#include <px4_platform_common/defines.h>
#include <px4_platform_common/module.h>
#include <px4_platform_common/posix.h>
#include <px4_platform_common/getopt.h>

#include <drivers/drv_hrt.h>
#include <uORB/PublicationMulti.hpp>
#include <uORB/topics/sensor_gps.h>

#include <arpa/inet.h>
#include <math.h>
#include <netinet/in.h>
#include <string.h>
#include <sys/socket.h>
#include <unistd.h>

static constexpr uint16_t DROID_GPS_DEFAULT_PORT = 15550;

#pragma pack(push, 1)
struct droid_gps_pkt {
	uint64_t time_utc_usec;    //  0
	double   lat_deg;          //  8
	double   lon_deg;          // 16
	double   alt_msl_m;        // 24
	float    eph;              // 32
	float    epv;              // 36
	float    s_variance_m_s;   // 40
	float    vel_n_m_s;        // 44
	float    vel_e_m_s;        // 48
	float    vel_d_m_s;        // 52
	float    cog_rad;          // 56
	uint8_t  fix_type;         // 60
	uint8_t  satellites_used;  // 61
	uint8_t  vel_ned_valid;    // 62
	uint8_t  pad;              // 63
};
#pragma pack(pop)

static_assert(sizeof(droid_gps_pkt) == 64, "droid_gps_pkt must be 64 bytes");

class DroidGps : public ModuleBase<DroidGps>
{
public:
	DroidGps(uint16_t port) : _port(port) {}
	~DroidGps() override = default;

	static int task_spawn(int argc, char *argv[]);
	static DroidGps *instantiate(int argc, char *argv[]);
	static int custom_command(int argc, char *argv[]);
	static int print_usage(const char *reason = nullptr);

	void run() override;
	int print_status() override;

private:
	static constexpr uint32_t DEVICE_ID_GPS = 11468804;

	uint16_t _port;
	int _fd{-1};
	unsigned _packet_count{0};
	unsigned _publish_count{0};
	hrt_abstime _last_fix_time{0};

	uORB::PublicationMulti<sensor_gps_s> _sensor_gps_pub{ORB_ID(sensor_gps)};
};

void DroidGps::run()
{
	_fd = socket(AF_INET, SOCK_DGRAM, 0);

	if (_fd < 0) {
		PX4_ERR("socket failed: %s", strerror(errno));
		return;
	}

	struct sockaddr_in addr {};

	addr.sin_family = AF_INET;

	addr.sin_addr.s_addr = htonl(INADDR_LOOPBACK);

	addr.sin_port = htons(_port);

	if (bind(_fd, (struct sockaddr *)&addr, sizeof(addr)) != 0) {
		PX4_ERR("bind 127.0.0.1:%u failed: %s", _port, strerror(errno));
		close(_fd);
		_fd = -1;
		return;
	}

	struct timeval tv {};

	tv.tv_sec = 1;

	setsockopt(_fd, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));

	PX4_INFO("listening on udp://127.0.0.1:%u", _port);

	while (!should_exit()) {
		droid_gps_pkt pkt;
		ssize_t n = recv(_fd, &pkt, sizeof(pkt), 0);

		if (n < 0) {
			// EAGAIN: recv timeout, loop to re-check should_exit()
			continue;
		}

		if (n != sizeof(pkt)) {
			PX4_WARN("short packet: %zd bytes", n);
			continue;
		}

		_packet_count++;

		sensor_gps_s gps{};
		gps.timestamp_sample = hrt_absolute_time();
		gps.device_id = DEVICE_ID_GPS;
		gps.latitude_deg = pkt.lat_deg;
		gps.longitude_deg = pkt.lon_deg;
		gps.altitude_msl_m = pkt.alt_msl_m;
		gps.altitude_ellipsoid_m = pkt.alt_msl_m;
		gps.time_utc_usec = pkt.time_utc_usec;
		gps.fix_type = pkt.fix_type;
		gps.eph = pkt.eph;
		gps.epv = pkt.epv;
		gps.hdop = pkt.eph;
		gps.vdop = pkt.epv;
		gps.s_variance_m_s = pkt.s_variance_m_s;
		gps.c_variance_rad = 0.5f;
		gps.vel_n_m_s = pkt.vel_n_m_s;
		gps.vel_e_m_s = pkt.vel_e_m_s;
		gps.vel_d_m_s = pkt.vel_d_m_s;
		gps.vel_m_s = sqrtf(pkt.vel_n_m_s * pkt.vel_n_m_s + pkt.vel_e_m_s * pkt.vel_e_m_s);
		gps.cog_rad = pkt.cog_rad;
		gps.vel_ned_valid = (pkt.vel_ned_valid != 0);
		gps.satellites_used = pkt.satellites_used;
		gps.heading = NAN;
		gps.heading_offset = NAN;
		gps.heading_accuracy = 0.f;
		gps.timestamp = hrt_absolute_time();

		_sensor_gps_pub.publish(gps);
		_publish_count++;
		_last_fix_time = gps.timestamp;
	}

	close(_fd);
	_fd = -1;
}

int DroidGps::print_status()
{
	PX4_INFO("port: %u, packets received: %u, published: %u", _port, _packet_count, _publish_count);

	if (_last_fix_time > 0) {
		PX4_INFO("last fix: %.1f s ago", (double)(hrt_elapsed_time(&_last_fix_time)) / 1e6);

	} else {
		PX4_INFO("no fix received yet");
	}

	return 0;
}

int DroidGps::task_spawn(int argc, char *argv[])
{
	_task_id = px4_task_spawn_cmd("droid_gps",
				      SCHED_DEFAULT,
				      SCHED_PRIORITY_SLOW_DRIVER,
				      2048,
				      (px4_main_t)&run_trampoline,
				      (char *const *)argv);

	if (_task_id < 0) {
		_task_id = -1;
		return -errno;
	}

	return 0;
}

DroidGps *DroidGps::instantiate(int argc, char *argv[])
{
	uint16_t port = DROID_GPS_DEFAULT_PORT;

	int myoptind = 1;
	int ch;
	const char *myoptarg = nullptr;

	while ((ch = px4_getopt(argc, argv, "p:", &myoptind, &myoptarg)) != EOF) {
		switch (ch) {
		case 'p':
			port = (uint16_t)strtoul(myoptarg, nullptr, 10);
			break;

		default:
			break;
		}
	}

	return new DroidGps(port);
}

int DroidGps::custom_command(int argc, char *argv[])
{
	return print_usage("unknown command");
}

int DroidGps::print_usage(const char *reason)
{
	if (reason) {
		PX4_WARN("%s\n", reason);
	}

	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description
GNSS bridge for Android. Listens on a local UDP port for 64-byte fix
packets sent by the Java app layer (LocationManager) and publishes
sensor_gps.
)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("droid_gps", "driver");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_PARAM_INT('p', DROID_GPS_DEFAULT_PORT, 1024, 65535, "UDP listen port", true);
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();

	return 0;
}

extern "C" __EXPORT int droid_gps_main(int argc, char *argv[])
{
	return DroidGps::main(argc, argv);
}
