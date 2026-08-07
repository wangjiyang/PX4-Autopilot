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

#include "DroidSensors.hpp"

#include <drivers/drv_hrt.h>
#include <parameters/param.h>
#include <px4_platform_common/getopt.h>

#include <math.h>

DroidSensors::DroidSensors()
{
	_px4_accel.set_range(16.f * CONSTANTS_ONE_G);
}

hrt_abstime DroidSensors::map_timestamp(int slot, int64_t event_timestamp_ns)
{
	// ASensorEvent.timestamp is CLOCK_BOOTTIME in ns; hrt is CLOCK_MONOTONIC.
	// Both clocks tick at the same rate while the device is awake, so the
	// offset is constant between suspends - track it as the MINIMUM observed
	// latency (now - ev). Re-estimating it on every delivery-jitter spike
	// (earlier approach) stretched/compressed dt between adjacent samples;
	// the EKF integrated gyro over those wrong dts, saw phantom rotation and
	// learned absurd gyro biases (0.17 rad/s observed on a static phone).
	const int64_t ev_us = event_timestamp_ns / 1000;
	const int64_t now = (int64_t)hrt_absolute_time();
	const int64_t candidate = now - ev_us;
	int64_t &off = _ts_offset_us[slot];

	if (off == OFFSET_UNSET || candidate < off) {
		// tighter mapping (lower delivery latency observed, or the clocks
		// diverged across a suspend): adopt immediately, keeps mapped <= now
		off = candidate;

	} else {
		// drift extremely slowly toward higher-latency observations so a
		// one-off tight outlier cannot pin the offset forever
		off += (candidate - off) >> 10;
	}

	const int64_t mapped_signed = ev_us + off;

	if (mapped_signed <= 0) {
		return 0;
	}

	const uint64_t mapped = (uint64_t)mapped_signed;

	// Some HALs deliver out-of-order batches. Downstream (vehicle_imu)
	// integrates delta_velocity/dt, so squeezing timestamps to stay
	// monotonic corrupts the integration - drop such samples instead.
	if (mapped <= _last_mapped_us[slot]) {
		return 0;
	}

	_last_mapped_us[slot] = mapped;
	return mapped;
}

void DroidSensors::handle_event(const ASensorEvent &ev)
{
	int slot;

	switch (ev.type) {
	case ASENSOR_TYPE_ACCELEROMETER: slot = 0; break;

	case ASENSOR_TYPE_GYROSCOPE: slot = 1; break;

	case ASENSOR_TYPE_MAGNETIC_FIELD: slot = 2; break;

	case ASENSOR_TYPE_PRESSURE: slot = 3; break;

	default: return;
	}

	const hrt_abstime now = map_timestamp(slot, ev.timestamp);

	if (now == 0) {
		// out-of-order sample dropped
		return;
	}

	switch (ev.type) {
	case ASENSOR_TYPE_ACCELEROMETER:
		// Android: m/s^2, device frame -> PX4 FRD
		_px4_accel.update(now, ev.acceleration.y, ev.acceleration.x, -ev.acceleration.z);
		_accel_count++;
		break;

	case ASENSOR_TYPE_GYROSCOPE:
		// Android: rad/s
		_px4_gyro.update(now, ev.vector.y, ev.vector.x, -ev.vector.z);
		_gyro_count++;
		break;

	case ASENSOR_TYPE_MAGNETIC_FIELD:
		// Android: microtesla -> Gauss (1 G = 100 uT)
		_px4_mag.update(now, ev.magnetic.y * 0.01f, ev.magnetic.x * 0.01f, -ev.magnetic.z * 0.01f);
		_mag_count++;
		break;

	case ASENSOR_TYPE_PRESSURE: {
			// Android: hPa -> Pa
			sensor_baro_s baro{};
			baro.timestamp_sample = now;
			baro.device_id = DEVICE_ID_BARO;
			baro.pressure = ev.pressure * 100.f;
			baro.temperature = 25.f; // not provided by the pressure sensor
			baro.timestamp = hrt_absolute_time();
			_sensor_baro_pub.publish(baro);
			_baro_count++;
			break;
		}

	default:
		break;
	}
}

void DroidSensors::run()
{
	ALooper *looper = ALooper_prepare(ALOOPER_PREPARE_ALLOW_NON_CALLBACKS);

	if (looper == nullptr) {
		PX4_ERR("ALooper_prepare failed");
		return;
	}

	ASensorManager *mgr = ASensorManager_getInstanceForPackage("com.droid.px4");

	if (mgr == nullptr) {
		PX4_ERR("ASensorManager unavailable");
		return;
	}

	ASensorEventQueue *queue = ASensorManager_createEventQueue(mgr, looper, 1, nullptr, nullptr);

	if (queue == nullptr) {
		PX4_ERR("failed to create sensor event queue");
		return;
	}

	const struct {
		int type;
		const char *name;
		int32_t interval_us;
	} wanted[] = {
		{ASENSOR_TYPE_ACCELEROMETER, "accel", IMU_INTERVAL_US},
		{ASENSOR_TYPE_GYROSCOPE,     "gyro",  IMU_INTERVAL_US},
		{ASENSOR_TYPE_MAGNETIC_FIELD, "mag",  20000},
		{ASENSOR_TYPE_PRESSURE,      "baro",  50000},
	};

	int enabled = 0;

	for (const auto &w : wanted) {
		const ASensor *s = ASensorManager_getDefaultSensor(mgr, w.type);

		if (s == nullptr) {
			PX4_WARN("no %s sensor on this device", w.name);
			continue;
		}

		int32_t min_delay = ASensor_getMinDelay(s); // us

		if (ASensorEventQueue_enableSensor(queue, s) != 0) {
			PX4_ERR("failed to enable %s", w.name);
			continue;
		}

		int32_t interval = (w.interval_us > min_delay) ? w.interval_us : min_delay;
		ASensorEventQueue_setEventRate(queue, s, interval);
		PX4_INFO("%s: %s, min_delay %d us, requested %d us", w.name, ASensor_getName(s),
			 (int)min_delay, (int)interval);
		enabled++;
	}

	if (enabled == 0) {
		PX4_ERR("no sensors enabled, exiting");
		ASensorManager_destroyEventQueue(mgr, queue);
		return;
	}

	// Adapt the estimator configuration to what this phone actually has.
	// This runs before 'ekf2 start' in px4.config, so the params take
	// effect for the estimator's whole lifetime.
	{
		const bool has_mag = ASensorManager_getDefaultSensor(mgr, ASENSOR_TYPE_MAGNETIC_FIELD) != nullptr;
		const bool has_baro = ASensorManager_getDefaultSensor(mgr, ASENSOR_TYPE_PRESSURE) != nullptr;

		// only force mag-less mode when the hardware is missing; when a mag
		// exists the choice is left to px4.config (indoor demos disable mag
		// fusion anyway - disturbed fields cause visible EKF yaw resets)
		if (!has_mag) {
			int32_t mag_type = 5; // none
			param_set(param_find("EKF2_MAG_TYPE"), &mag_type);
			PX4_WARN("no magnetometer: EKF runs mag-less, yaw will drift");
		}

		int32_t has_baro_i = has_baro ? 1 : 0;
		param_set(param_find("SYS_HAS_BARO"), &has_baro_i);
		param_set(param_find("EKF2_BARO_CTRL"), &has_baro_i);

		if (!has_baro) {
			PX4_INFO("no barometer: EKF baro fusion disabled");
		}
	}

	while (!should_exit()) {
		int events = 0;
		void *data = nullptr;
		ALooper_pollOnce(200 /* ms */, nullptr, &events, &data);

		ASensorEvent ev[32];
		ssize_t n;

		while ((n = ASensorEventQueue_getEvents(queue, ev, 32)) > 0) {
			for (ssize_t i = 0; i < n; i++) {
				handle_event(ev[i]);
			}
		}
	}

	for (const auto &w : wanted) {
		const ASensor *s = ASensorManager_getDefaultSensor(mgr, w.type);

		if (s != nullptr) {
			ASensorEventQueue_disableSensor(queue, s);
		}
	}

	ASensorManager_destroyEventQueue(mgr, queue);
}

int DroidSensors::print_status()
{
	PX4_INFO("events: accel %u, gyro %u, mag %u, baro %u",
		 _accel_count, _gyro_count, _mag_count, _baro_count);
	return 0;
}

int DroidSensors::task_spawn(int argc, char *argv[])
{
	_task_id = px4_task_spawn_cmd("droid_sensors",
				      SCHED_DEFAULT,
				      SCHED_PRIORITY_FAST_DRIVER,
				      2600,
				      (px4_main_t)&run_trampoline,
				      (char *const *)argv);

	if (_task_id < 0) {
		_task_id = -1;
		return -errno;
	}

	return 0;
}

DroidSensors *DroidSensors::instantiate(int argc, char *argv[])
{
	return new DroidSensors();
}

int DroidSensors::custom_command(int argc, char *argv[])
{
	return print_usage("unknown command");
}

int DroidSensors::print_usage(const char *reason)
{
	if (reason) {
		PX4_WARN("%s\n", reason);
	}

	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description
Bridge from Android sensors (ASensorManager NDK API) to PX4 uORB topics.
Publishes sensor_accel, sensor_gyro, sensor_mag and sensor_baro from the
phone's built-in sensors.
)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("droid_sensors", "driver");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();

	return 0;
}

extern "C" __EXPORT int droid_sensors_main(int argc, char *argv[])
{
	return DroidSensors::main(argc, argv);
}
