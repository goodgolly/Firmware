/****************************************************************************
 *
 *   Copyright (C) 2008-2012 PX4 Development Team. All rights reserved.
 *   Author: Lorenz Meier <lm@inf.ethz.ch>
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

/**
 * @file multirotor_pos_control.c
 *
 * Skeleton for multirotor position controller
 */

#include <nuttx/config.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdbool.h>
#include <unistd.h>
#include <fcntl.h>
#include <errno.h>
#include <debug.h>
#include <termios.h>
#include <time.h>
#include <sys/prctl.h>
#include <drivers/drv_hrt.h>
#include <drivers/drv_tone_alarm.h>
#include <uORB/uORB.h>
#include <uORB/topics/sensor_combined.h>
#include <uORB/topics/parameter_update.h>
#include <uORB/topics/vehicle_status.h>
#include <uORB/topics/vehicle_attitude.h>
#include <uORB/topics/vehicle_status.h>
#include <uORB/topics/manual_control_setpoint.h>
#include <uORB/topics/vehicle_attitude_setpoint.h>
#include <uORB/topics/vehicle_local_position_setpoint.h>
#include <uORB/topics/vehicle_vicon_position.h>
#include <uORB/topics/vehicle_local_position.h>
#include <uORB/topics/debug_key_value.h>

#include <systemlib/systemlib.h>
#include <systemlib/perf_counter.h>
#include <poll.h>

#include "multirotor_pos_control_params.h"
#include <mavlink/mavlink_log.h>
#include <math.h>

static bool thread_should_exit = false;		/**< Deamon exit flag */
static bool thread_running = false;		/**< Deamon status flag */
static int deamon_task;				/**< Handle of deamon task / thread */

__EXPORT int multirotor_pos_control_main(int argc, char *argv[]);

/**
 * Mainloop of position controller.
 */
static int multirotor_pos_control_thread_main(int argc, char *argv[]);

/**
 * Print the correct usage.
 */
static void usage(const char *reason);

static void
usage(const char *reason)
{
	if (reason)
		fprintf(stderr, "%s\n", reason);
	fprintf(stderr, "usage: deamon {start|stop|status} [-p <additional params>]\n\n");
	exit(1);
}

/**
 * The deamon app only briefly exists to start
 * the background job. The stack size assigned in the
 * Makefile does only apply to this management task.
 * 
 * The actual stack size should be set in the call
 * to task_spawn().
 */
int multirotor_pos_control_main(int argc, char *argv[])
{
	if (argc < 1)
		usage("missing command");

	if (!strcmp(argv[1], "start")) {

		if (thread_running) {
			printf("multirotor pos control already running\n");
			/* this is not an error */
			exit(0);
		}

		thread_should_exit = false;
		deamon_task = task_spawn("multirotor pos control",
					 SCHED_DEFAULT,
					 SCHED_PRIORITY_MAX - 30,
					 4096,
					 multirotor_pos_control_thread_main,
					 (argv) ? (const char **)&argv[2] : (const char **)NULL);
		exit(0);
	}

	if (!strcmp(argv[1], "stop")) {
		thread_should_exit = true;
		exit(0);
	}

	if (!strcmp(argv[1], "status")) {
		if (thread_running) {
			printf("\tmultirotor pos control app is running\n");
		} else {
			printf("\tmultirotor pos control app not started\n");
		}
		exit(0);
	}

	usage("unrecognized command");
	exit(1);
}

static int
multirotor_pos_control_thread_main(int argc, char *argv[]){
	/* welcome user */
	printf("[multirotor pos control] Control started, taking over position control\n");
	int mavlink_fd = open(MAVLINK_LOG_DEVICE, 0);
	mavlink_log_info(mavlink_fd, "[multirotor pos control] Control started, taking over position control\n");

	/* structures */
	struct vehicle_attitude_s att;
	memset(&att, 0, sizeof(att));
	//struct vehicle_global_position_setpoint_s global_pos_sp;
	struct vehicle_local_position_setpoint_s local_pos_sp;
	memset(&local_pos_sp, 0, sizeof(local_pos_sp));
	struct vehicle_local_position_s local_pos_est;
	memset(&local_pos_est, 0, sizeof(local_pos_est));
	struct vehicle_vicon_position_s vicon_pos;
	memset(&vicon_pos, 0, sizeof(vicon_pos));
	struct manual_control_setpoint_s manual;
	memset(&manual, 0, sizeof(manual));
	struct vehicle_attitude_setpoint_s att_sp;
	memset(&att_sp, 0, sizeof(att_sp));
	struct vehicle_status_s vehicle_status;
	memset(&vehicle_status, 0, sizeof(vehicle_status));
	struct sensor_combined_s sensors;
	memset(&sensors, 0, sizeof(sensors));

	/* subscribe */
	int sensor_sub = orb_subscribe(ORB_ID(sensor_combined));
	int sub_params = orb_subscribe(ORB_ID(parameter_update));
	int att_sub = orb_subscribe(ORB_ID(vehicle_attitude));
	int manual_sub = orb_subscribe(ORB_ID(manual_control_setpoint));
	int vicon_pos_sub = orb_subscribe(ORB_ID(vehicle_vicon_position));
	//int global_pos_sp_sub = orb_subscribe(ORB_ID(vehicle_global_position_setpoint));
	int local_pos_sp_sub = orb_subscribe(ORB_ID(vehicle_local_position_setpoint));
	int local_pos_est_sub = orb_subscribe(ORB_ID(vehicle_local_position));
	int vehicle_status_sub = orb_subscribe(ORB_ID(vehicle_status));

	/* publish attitude setpoint */
	orb_advert_t att_sp_pub = orb_advertise(ORB_ID(vehicle_attitude_setpoint), &att_sp);

	static float rotMatrix[4] = {1.0f,  0.0f, 0.0f,  1.0f};
	static float pos_ctrl_gain_p = 0.8f;
	static float pos_ctrl_gain_d = 0.8f;
	static float z_ctrl_gain_p = 0.8f;
	static float z_ctrl_gain_d = 0.6f;
	static float z_pos_setpoint = -1.0f;
	const float pitch_limit = 0.33f;
	const float roll_limit = 0.33f;
	const float thrust_limit_upper = 0.5f;
	const float thrust_limit_lower = 0.1f;

	struct multirotor_position_control_params pos_params;
	struct multirotor_position_control_param_handles handle_pos_params;
	parameters_init(&handle_pos_params);
	parameters_update(&handle_pos_params, &pos_params);

	perf_counter_t interval_perf = perf_alloc(PC_INTERVAL, "multirotor_pos_control_interval");
	perf_counter_t mc_err_perf = perf_alloc(PC_COUNT, "multirotor_pos_control_err");

	struct pollfd fds[2] = {
					//{ .fd = vicon_pos_sub, .events = POLLIN }, //vicon_pos_sub
					{ .fd = sensor_sub, .events = POLLIN }, //ca. 130 Hz
					{ .fd = sub_params,   .events = POLLIN },
				};

	thread_running = true;
	uint64_t last_time = 0;

	while (!thread_should_exit) {
		/* wait for a vicon update update, check for exit condition every 500 ms */
				int ret = poll(fds, 2, 2000);
				if (ret < 0) {
					/* poll error, count it in perf */
					perf_count(mc_err_perf);
				} else if (ret == 0) {
					/* no return value, ignore */
				} else {
					if (fds[1].revents & POLLIN){
						/* read from param to clear updated flag */
						struct parameter_update_s update;
						orb_copy(ORB_ID(parameter_update), sub_params, &update);
						/* update parameters */
						parameters_update(&handle_pos_params, &pos_params);
						pos_ctrl_gain_d = pos_params.pos_d;
						pos_ctrl_gain_p = pos_params.pos_p;
						z_ctrl_gain_p = pos_params.height_p;
						z_ctrl_gain_d = pos_params.height_d;
						z_pos_setpoint = pos_params.height_sp;
						//printf("[multirotor_pos_control] pos_params.k1: %8.4f\t pos.params.k2: %8.4f\n", (double)(pos_params.k1), (double)(pos_params.k2));
					}
					if (fds[0].revents & POLLIN) {
						/*float dT = (hrt_absolute_time() - last_time) / 1000000.0f;
						last_time = hrt_absolute_time();
						static int printcounter = 0;
						if (printcounter == 50000) {
							printcounter = 0;
							printf("[posCTRL] dT: %8.4f\n", (double)(dT));
						}
						printcounter++;*/

						orb_copy(ORB_ID(manual_control_setpoint), manual_sub, &manual);
						orb_copy(ORB_ID(vehicle_attitude), att_sub, &att);
						orb_copy(ORB_ID(vehicle_local_position), local_pos_est_sub, &local_pos_est);
						orb_copy(ORB_ID(vehicle_local_position_setpoint), local_pos_sp_sub, &local_pos_sp);
						orb_copy(ORB_ID(vehicle_vicon_position), vicon_pos_sub, &vicon_pos);
						orb_copy(ORB_ID(vehicle_status), vehicle_status_sub, &vehicle_status);
						orb_copy(ORB_ID(sensor_combined), sensor_sub, &sensors);

						if (vehicle_status.state_machine == SYSTEM_STATE_AUTO) {
							/* ROLL & PITCH REGLER */
							float y_pos_setpoint = local_pos_sp.y;
							float x_pos_setpoint = local_pos_sp.x;
							float y_pos_err_earth = -(local_pos_est.y - y_pos_setpoint);
							float x_pos_err_earth = (local_pos_est.x - x_pos_setpoint);
							float y_vel_setpoint = 0.0f;
							float x_vel_setpoint = 0.0f;
							float y_vel_err_earth = -(local_pos_est.vy - y_vel_setpoint);
							float x_vel_err_earth = (local_pos_est.vx - x_vel_setpoint);
							/* rotMatrix is from body to earth*/
							rotMatrix[0] = cos(vicon_pos.yaw);
							rotMatrix[1] = -sin(vicon_pos.yaw);
							rotMatrix[2] = sin(vicon_pos.yaw);
							rotMatrix[3] = cos(vicon_pos.yaw);
							/* PD regler im earth frame, different sign because of Transformation from earth to body frame */
							float rollpos = (rotMatrix[0]*y_pos_err_earth-rotMatrix[1]*x_pos_err_earth)*pos_ctrl_gain_p;
							float rollvel = (rotMatrix[0]*y_vel_err_earth-rotMatrix[1]*x_vel_err_earth)*pos_ctrl_gain_d;
							float pitchpos = (-rotMatrix[2]*y_pos_err_earth+rotMatrix[3]*x_pos_err_earth)*pos_ctrl_gain_p;
							float pitchvel = (-rotMatrix[2]*y_vel_err_earth+rotMatrix[3]*x_vel_err_earth)*pos_ctrl_gain_d;
							float rolltot = rollpos + rollvel;
							float pitchtot = pitchpos + pitchvel;

							/* limit setpoints to maximal the values of the manual flight*/
							if((rolltot <= roll_limit) && (rolltot >= -roll_limit)){
								att_sp.roll_body = rolltot;
							}else{
								if(rolltot > roll_limit){
									att_sp.roll_body = roll_limit;
								}
								if(rolltot < -roll_limit){
									att_sp.roll_body = -roll_limit;
								}
							}
							if((pitchtot <= pitch_limit) && (pitchtot >= -pitch_limit)){
								att_sp.pitch_body = pitchtot;
							}else{
								if(pitchtot > pitch_limit){
									att_sp.pitch_body = pitch_limit;
								}
								if(pitchtot < -pitch_limit){
									att_sp.pitch_body = -pitch_limit;
								}
							}
							//OVERRIDE
							//att_sp.roll_body = manual.roll;
							//att_sp.pitch_body = manual.pitch;
							//END OVERRIDE

							/* YAW REGLER */
							if ((manual.yaw < -0.01f || 0.01f < manual.yaw) && manual.throttle > 0.3f) {
							att_sp.yaw_body = att_sp.yaw_body + manual.yaw * 0.0025f;
							} else if (manual.throttle <= 0.3f) {
							att_sp.yaw_body = att.yaw;
							}
							//att_sp.yaw_body = 0.0f;
							//printf("[multirotor_pos_control] vicon_pos.yaw: %8.4f\n", (double)(vicon_pos.yaw));

							/* Z REGLER, PD mit Feedforward */
							float z_vel_setpoint = local_pos_sp.z;
							float z_pos_err_earth = (local_pos_est.z - z_pos_setpoint);
							float z_vel_err_earth = (local_pos_est.vz - z_vel_setpoint);
							float z_ctrl_thrust_err = z_pos_err_earth*z_ctrl_gain_p + z_vel_err_earth*z_ctrl_gain_d;
							float z_ctrl_thrust_feedforward = 0.65f;
							float z_ctrl_thrust = z_ctrl_thrust_feedforward + z_ctrl_thrust_err;
							/* the throttle stick on the rc control limits the maximum thrust */
							float thrust_limit_upper = manual.throttle;
							if (z_ctrl_thrust >= thrust_limit_upper){
								z_ctrl_thrust = thrust_limit_upper;
								/*never go too low with the thrust, quadrotor may become uncontrollable */
							}else if(z_ctrl_thrust < thrust_limit_lower){
								z_ctrl_thrust = thrust_limit_lower;
							}
							//printf("[multirotor_att_control_main] height_ctrl_thrust: %8.4f\n", (double)(height_ctrl_thrust));
							att_sp.thrust = z_ctrl_thrust;
							att_sp.timestamp = hrt_absolute_time();

							/* publish new attitude setpoint */
							orb_publish(ORB_ID(vehicle_attitude_setpoint), att_sp_pub, &att_sp);

							/* measure in what intervals the controller runs */
							perf_count(interval_perf);
						} else {
							//manual control
						}
					} /* end of poll call for vicon updates */
				} /* end of poll return value check */
	}

	printf("[multirotor pos control] ending now...\n");
	mavlink_log_info(mavlink_fd, "[multirotor pos control] ending now...\n");
	thread_running = false;
	fflush(stdout);
	return 0;
}

