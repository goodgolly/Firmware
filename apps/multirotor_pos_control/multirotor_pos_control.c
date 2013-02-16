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
#include <uORB/topics/parameter_update.h>
#include <uORB/topics/vehicle_status.h>
#include <uORB/topics/vehicle_attitude.h>
#include <uORB/topics/manual_control_setpoint.h>
#include <uORB/topics/vehicle_attitude_setpoint.h>
//#include <uORB/topics/vehicle_local_position_setpoint.h>
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
multirotor_pos_control_thread_main(int argc, char *argv[])
{
	/* welcome user */
	printf("[multirotor pos control] Control started, taking over position control\n");
	int mavlink_fd = open(MAVLINK_LOG_DEVICE, 0);
	mavlink_log_info(mavlink_fd, "[multirotor pos control] Control started, taking over position control\n");

	/* structures */
	struct vehicle_status_s state;
	struct vehicle_attitude_s att;
	//struct vehicle_global_position_setpoint_s global_pos_sp;
	//struct vehicle_local_position_setpoint_s local_pos_sp;
	struct vehicle_local_position_s local_pos_est;
	struct vehicle_vicon_position_s vicon_pos;
	struct manual_control_setpoint_s manual;
	struct vehicle_attitude_setpoint_s att_sp;

	struct debug_key_value_s dbg1 = { .key = "x", .value = 0.0f };
	struct debug_key_value_s dbg2 = { .key = "vx", .value = 0.0f };

	/* subscribe to param changes */
	int sub_params = orb_subscribe(ORB_ID(parameter_update));
	/* subscribe to attitude, motor setpoints and system state */
	int att_sub = orb_subscribe(ORB_ID(vehicle_attitude));
	int state_sub = orb_subscribe(ORB_ID(vehicle_status));
	int manual_sub = orb_subscribe(ORB_ID(manual_control_setpoint));
	int vicon_pos_sub = orb_subscribe(ORB_ID(vehicle_vicon_position));
	//int global_pos_sp_sub = orb_subscribe(ORB_ID(vehicle_global_position_setpoint));
	//int local_pos_sp_sub = orb_subscribe(ORB_ID(vehicle_local_position_setpoint));
	int local_pos_est_sub = orb_subscribe(ORB_ID(vehicle_local_position));

	/* publish attitude setpoint */
	orb_advert_t att_sp_pub = orb_advertise(ORB_ID(vehicle_attitude_setpoint), &att_sp);

	orb_advert_t pub_dbg1 = orb_advertise(ORB_ID(debug_key_value), &dbg1);
	orb_advert_t pub_dbg2 = orb_advertise(ORB_ID(debug_key_value), &dbg2);

	thread_running = true;

	struct pollfd fds[2] = {
					{ .fd = vicon_pos_sub, .events = POLLIN }, //vicon_pos_sub
					{ .fd = sub_params,   .events = POLLIN },
				};

	int loopcounter = 0;
	int dbgCNT = 0;
	float rotMatrix[4] = {1.0f,  0.0f,
						  0.0f,  1.0f};

	float pos_ctrl_gain_p = 0.08f;
	float pos_ctrl_gain_d = 0.00f;
	float height_ctrl_gain_p = 0.1f;
	float height_sp = -1.5f;

	float pitch_limit = 0.33f;
	float roll_limit = 0.33f;
	float thrust_limit_upper = 0.5f;
	float thrust_limit_lower = 0.1f;

	struct multirotor_position_control_params pos_params;
	struct multirotor_position_control_param_handles handle_pos_params;
	parameters_init(&handle_pos_params);
	//parameters_update(&handle_pos_params, &pos_params);

	perf_counter_t interval_perf = perf_alloc(PC_INTERVAL, "multirotor_pos_control_interval");
	perf_counter_t mc_err_perf = perf_alloc(PC_COUNT, "multirotor_pos_control_err");

	uint64_t last_time = 0;

	/*bool yaw_offset_init = 0;
	unsigned yaw_correction_loop_cnt = 0;
	unsigned yaw_correction_loop_end = 50;
	float yaw_offset_rad = 0.0f;

	unsigned yaw_correction_circular_numElem = 5;
	unsigned yaw_correction_circular_pointer = 0;
	float circularBuffer[yaw_correction_circular_numElem];
	for (unsigned i = 0; i < yaw_correction_circular_numElem; i++){
		circularBuffer[i] = 0;
	}
	float yaw_offset_rad_temp = 0;
	*/

	float gain_k1 = 1.0f;
	float gain_k2 = 1.0f;

	/* override manual protection */


	while (!thread_should_exit) {
		/* wait for a vicon update update, check for exit condition every 500 ms */
		int ret = poll(fds, 2, 500);

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
				height_ctrl_gain_p = pos_params.height_p;
				height_sp = pos_params.height_sp;
				gain_k1 = pos_params.k1;
				gain_k2 = pos_params.k2;
				// dont forget baro here later
				printf("[multirotor_pos_control] pos_params.k1: %8.4f\t pos.params.k2: %8.4f\n", (double)(pos_params.k1), (double)(pos_params.k2));
			}
			/* only run controller if vicon changed */
			if (fds[0].revents & POLLIN) {
				float dT = (hrt_absolute_time() - last_time) / 1000000.0f;
				last_time = hrt_absolute_time();
				//printf("[multirotor_att_control_main] dT: %8.4f\n", (double)(dT));

				/* get a local copy of the vehicle state */
				orb_copy(ORB_ID(vehicle_status), state_sub, &state);
				/* get a local copy of manual setpoint */
				orb_copy(ORB_ID(manual_control_setpoint), manual_sub, &manual);
				/* get a local copy of attitude */
				orb_copy(ORB_ID(vehicle_attitude), att_sub, &att);
				/* get a local copy of local position */
				orb_copy(ORB_ID(vehicle_local_position), local_pos_est_sub, &local_pos_est);
				/* get a local copy of vicon_position */
				orb_copy(ORB_ID(vehicle_vicon_position), vicon_pos_sub, &vicon_pos);

				//dbg1.value = x_k;
				//orb_publish(ORB_ID(debug_key_value), pub_dbg1, &dbg1);
				//dbg2.value = vx_k;
				//orb_publish(ORB_ID(debug_key_value), pub_dbg2, &dbg2);
				//printf("before auto  x: %8.4f\tvx: %8.4f\ty: %8.4f\tvy: %8.4f\n", (double)(x_k), (double)(vx_k), (double)(y_k), (double)(vy_k));
				static int printcounter = 0;
				if (printcounter == 50) {
					printcounter = 0;
					printf("before auto state");
				}

				if (state.state_machine == SYSTEM_STATE_AUTO) {
					if(state.flag_useGPS){
						/* all GPS control here */
					}else{
						/* all VICON control here */
						static int printcounter = 0;
						if (printcounter == 50) {
							printcounter = 0;
							printf("after auto state x: %d cm\ty: %d cm\n", (int)(local_pos_est.x*100), (int)(local_pos_est.y*100));
						}
						printcounter++;
						/* ROLL & PITCH REGLER */
						float y_pos_setpoint = 0.0f;
						float x_pos_setpoint = 0.0f;
						float y_pos_err_earth = -(local_pos_est.y - y_pos_setpoint);
						float x_pos_err_earth = (local_pos_est.x - x_pos_setpoint);

						float y_vel_setpoint = 0.0f;
						float x_vel_setpoint = 0.0f;
						float y_vel_err_earth = -(local_pos_est.vy - y_vel_setpoint);
						float x_vel_err_earth = (local_pos_est.vx - x_vel_setpoint);

						/* rotMatrix is from body to earth*/
						float rotMatrix[4] = {1.0f, 0.0f,
											  0.0f, 1.0f};
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
						/* checked that limitation works correctly */
						//printf("[multirotor_att_control_main] att_sp.roll_body: %8.4f\n", (double)(att_sp.roll_body));

						/* YAW REGLER */
						/*if ((manual.yaw < -0.01f || 0.01f < manual.yaw) && manual.throttle > 0.3f) {
						att_sp.yaw_body = att_sp.yaw_body + manual.yaw * 0.0025f;
						} else if (manual.throttle <= 0.3f) {
						att_sp.yaw_body = att.yaw;
						}*/
						att_sp.yaw_body = 0.0f;
						//printf("[multirotor_pos_control] vicon_pos.yaw: %8.4f\n", (double)(vicon_pos.yaw));

						/* THRUST REGLER, P mit Feedforward */
						float height_err = vicon_pos.z - height_sp;
						float height_ctrl_thrust_err = height_err * height_ctrl_gain_p;
						float height_ctrl_thrust_feedforward = 0.65f;
						float height_ctrl_thrust = height_ctrl_thrust_feedforward + height_ctrl_thrust_err;
						/* the throttle stick on the rc control limits the maximum thrust */
						thrust_limit_upper = manual.throttle;
						if (height_ctrl_thrust >= thrust_limit_upper){
							height_ctrl_thrust = thrust_limit_upper;
							/*never go too low with the thrust that it becomes uncontrollable */
						}else if(height_ctrl_thrust < thrust_limit_lower){
							height_ctrl_thrust = thrust_limit_lower;
						}
						//printf("[multirotor_att_control_main] height_ctrl_thrust: %8.4f\n", (double)(height_ctrl_thrust));
						att_sp.thrust = height_ctrl_thrust;
						att_sp.timestamp = hrt_absolute_time();

						/* publish new attitude setpoint */
						orb_publish(ORB_ID(vehicle_attitude_setpoint), att_sp_pub, &att_sp);

						/* measure in what intervals the controller runs */
						perf_count(interval_perf);
					} /* end state.flag_useGPS check*/
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

