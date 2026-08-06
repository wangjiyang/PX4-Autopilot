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
 * @file DroidSensors.hpp
 *
 * Android sensor bridge: reads the phone's accelerometer, gyroscope,
 * magnetometer and barometer through the NDK ASensorManager API and
 * publishes them as PX4 uORB sensor topics.
 *
 * Axis convention: the phone lies with the screen facing up and the top
 * edge of the screen pointing forward (vehicle nose).
 *   PX4 X (forward) = +Android Y
 *   PX4 Y (right)   = +Android X
 *   PX4 Z (down)    = -Android Z
 * (proper rotation, det = +1)
 */

#pragma once

#include <px4_platform_common/px4_config.h>
#include <px4_platform_common/defines.h>
#include <px4_platform_common/module.h>
#include <px4_platform_common/posix.h>

#include <lib/drivers/accelerometer/PX4Accelerometer.hpp>
#include <lib/drivers/gyroscope/PX4Gyroscope.hpp>
#include <lib/drivers/magnetometer/PX4Magnetometer.hpp>
#include <uORB/PublicationMulti.hpp>
#include <uORB/topics/sensor_baro.h>

#include <android/looper.h>
#include <android/sensor.h>

class DroidSensors : public ModuleBase<DroidSensors>
{
public:
	DroidSensors();
	~DroidSensors() override = default;

	/** @see ModuleBase */
	static int task_spawn(int argc, char *argv[]);
	static DroidSensors *instantiate(int argc, char *argv[]);
	static int custom_command(int argc, char *argv[]);
	static int print_usage(const char *reason = nullptr);

	/** @see ModuleBase::run() */
	void run() override;

	int print_status() override;

private:
	// device ids: encoded like DRV_IMU_DEVTYPE_SIM on bus 1 addr 1 (see fake_imu)
	static constexpr uint32_t DEVICE_ID_IMU  = 1310988;
	static constexpr uint32_t DEVICE_ID_MAG  = 197388;
	static constexpr uint32_t DEVICE_ID_BARO = 6620428;

	// target IMU sample interval in microseconds: ask for the sensor's
	// fastest rate (clamped to min_delay at enable time); the HAL may
	// still deliver less (observed ~80 Hz on ovaltine's bmi26x)
	static constexpr int32_t IMU_INTERVAL_US = 2500;

	void handle_event(const ASensorEvent &ev);

	/** map an Android sensor event timestamp (CLOCK_BOOTTIME ns) into the
	 *  hrt time base; batched events keep their true sample spacing */
	hrt_abstime map_timestamp(int64_t event_timestamp_ns);

	uint64_t _ts_offset_us{0};

	PX4Accelerometer _px4_accel{DEVICE_ID_IMU};
	PX4Gyroscope     _px4_gyro{DEVICE_ID_IMU};
	PX4Magnetometer  _px4_mag{DEVICE_ID_MAG};

	uORB::PublicationMulti<sensor_baro_s> _sensor_baro_pub{ORB_ID(sensor_baro)};

	unsigned _accel_count{0};
	unsigned _gyro_count{0};
	unsigned _mag_count{0};
	unsigned _baro_count{0};
};
