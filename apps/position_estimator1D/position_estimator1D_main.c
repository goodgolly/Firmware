/****************************************************************************
 *
 *   Copyright (C) 2008-2012 PX4 Development Team. All rights reserved.
 *   Author: 	Damian Aregger	<daregger@student.ethz.ch>
 *   			Tobias Naegeli <naegelit@student.ethz.ch>
* 				Lorenz Meier <lm@inf.ethz.ch>
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
 * @file position_estimator_main.c
 * Model-identification based position estimator for multirotors
 */

#include <unistd.h>
#include <stdlib.h>
#include <stdio.h>
#include <stdbool.h>
#include <fcntl.h>
#include <float.h>
#include <string.h>
#include <nuttx/config.h>
#include <nuttx/sched.h>
#include <sys/prctl.h>
#include <termios.h>
#include <errno.h>
#include <limits.h>
#include <math.h>
#include <uORB/uORB.h>
#include <uORB/topics/sensor_combined.h>
#include <uORB/topics/parameter_update.h>
#include <uORB/topics/vehicle_attitude.h>
#include <uORB/topics/actuator_outputs.h>
#include <uORB/topics/actuator_controls_effective.h>
#include <uORB/topics/vehicle_status.h>
#include <uORB/topics/vehicle_vicon_position.h>
#include <uORB/topics/vehicle_local_position.h>
#include <uORB/topics/vehicle_local_position_setpoint.h>
#include <uORB/topics/vehicle_gps_position.h>
#include <mavlink/mavlink_log.h>
#include <poll.h>
#include <systemlib/geo/geo.h>

#include "position_estimator1D_params.h"
//#include <uORB/topics/debug_key_value.h>
#include "codegen/kalman_dlqe2.h"
#include "codegen/kalman_dlqe3.h"
#include "sounds.h"
#include <drivers/drv_tone_alarm.h>

static bool thread_should_exit = false;	/**< Deamon exit flag */
static bool thread_running = false;	/**< Deamon status flag */
static int position_estimator1D_task;	/**< Handle of deamon task / thread */

__EXPORT int position_estimator1D_main(int argc, char *argv[]);

int position_estimator1D_thread_main(int argc, char *argv[]);
float thrust2force(float thrust);
/**
 * Print the correct usage.
 */
static void usage(const char *reason);

static void
usage(const char *reason)
{
	if (reason)
		fprintf(stderr, "%s\n", reason);
	fprintf(stderr, "usage: position_estimator1D {start|stop|status} [-p <additional params>]\n\n");
	exit(1);
}

/**
 * The position_estimator1D_thread only briefly exists to start
 * the background job. The stack size assigned in the
 * Makefile does only apply to this management task.
 *
 * The actual stack size should be set in the call
 * to task_create().
 */
int position_estimator1D_main(int argc, char *argv[])
{
	if (argc < 1)
		usage("missing command");

	if (!strcmp(argv[1], "start")) {

		if (thread_running) {
			printf("position_estimator1D already running\n");
			/* this is not an error */
			exit(0);
		}

		thread_should_exit = false;
		position_estimator1D_task = task_spawn("position_estimator1D",
					 SCHED_RR,
					 SCHED_PRIORITY_MAX - 5,
					 4096,
					 position_estimator1D_thread_main,
					 (argv) ? (const char **)&argv[2] : (const char **)NULL);
		exit(0);
	}
	if (!strcmp(argv[1], "stop")) {
		thread_should_exit = true;
		exit(0);
	}

	if (!strcmp(argv[1], "status")) {
		if (thread_running) {
			printf("\tposition_estimator1D is running\n");
		} else {
			printf("\tposition_estimator1D not started\n");
		}
		exit(0);
	}

	usage("unrecognized command");
	exit(1);
}

/**
 * return force from thrust input
 * output is ThrustForce in z Direction of the body frame 0.4198 - 8.9559 N
 * thrust=302 is hoovering
 *
 * @param thrust 0 - 511
 */
float thrust2force(float thrust){
	//double thrust_d = (double)thrust;
	float force = 2.371190582616025f*1e-5*thrust*thrust+0.004587809331818f*thrust+0.419806660877117f;
	return force;
}


/****************************************************************************
 * main
 ****************************************************************************/
int position_estimator1D_thread_main(int argc, char *argv[])
{
	/* welcome user */
	printf("[position_estimator1D] started\n");
	static int mavlink_fd;
	mavlink_fd = open(MAVLINK_LOG_DEVICE, 0);
	mavlink_log_info(mavlink_fd, "[position_estimator1D] started");

	/* initialize values */
	static float z[3] = {0, 0, 0}; /* output variables from tangent plane mapping */
	static float rotMatrix[4] = {1.0f,  0.0f, 0.0f,  1.0f};
	static float x_x_aposteriori_k[3] = {1.0f, 0.0f, 0.0f};
	static float x_y_aposteriori_k[3] = {1.0f, 0.0f, 0.0f};
	static float x_z_aposteriori_k[3] = {1.0f, 0.0f, 0.0f};
	static float x_x_aposteriori[3] = {0.0f, 0.0f, 0.0f};
	static float x_y_aposteriori[3] = {1.0f, 0.0f, 0.0f};
	static float x_z_aposteriori[3] = {1.0f, 0.0f, 0.0f};
	const static float dT_const_120 = 1.0f/120.0f;
	const static float dT_const_50 = 1.0f/50.0f;

	//static float posUpdateFreq = 50.0f;
	static float addNoise = 0.0f;
	static float sigma = 0.0f;
	//computed from dlqe in matlab
	const static float K_vicon_130Hz[3] = {0.2151f, 2.9154f, 19.7599f};
	const static float K_vicon_50Hz[3] = {0.5297f, 0.9873f, 0.9201f};
	const static float K_baro[3] = {0.0248f, 0.0377f, 0.0287f};
	static float K[3] = {0.0f, 0.0f, 0.0f};
	int baro_loop_cnt = 0;
	int baro_loop_end = 70; /* measurement for 1 second */
	float p0_Pa = 0.0f; /* to determin while start up */
	float rho0 = 1.293f; /* standard pressure */
	const static float const_earth_gravity = 9.81f;

	static int viconCnt = 0;
	static int viconCntMax = 0;
	static float debug = 0.0f;
	static float posX = 0.0f;
	static float posY = 0.0f;
	static float posZ = 0.0f;

	static float acc_x_body = 0.0f;
	static float acc_y_body = 0.0f;
	static float acc_z_body = 0.0f;
	static float acc_x_e = 0.0f;
	static float acc_y_e = 0.0f;
	static float acc_z_e = 0.0f;
	static float flyingT = 200.0f;
	static float accThres = 0.001f;
	static float velDecay = 0.995f;

	static double lat_current = 0.0d; //[°]] --> 47.0
	static double lon_current = 0.0d; //[°]] -->8.5
	static double alt_current = 0.0d; //[m] above MSL

	/* Initialize filter */
	kalman_dlqe2_initialize();
	kalman_dlqe3_initialize();

	/* declare and safely initialize all structs */
	struct sensor_combined_s sensor;
	memset(&sensor, 0, sizeof(sensor));
	struct vehicle_attitude_s att;
	memset(&att, 0, sizeof(att));
	struct vehicle_status_s vehicle_status;
	memset(&vehicle_status, 0, sizeof(vehicle_status)); /* make sure that baroINITdone = false */
	struct vehicle_vicon_position_s vicon_pos;
	memset(&vicon_pos, 0, sizeof(vicon_pos));
	struct actuator_controls_effective_s act_eff;
	memset(&act_eff, 0, sizeof(act_eff));
	struct vehicle_gps_position_s gps;
	memset(&gps, 0, sizeof(gps));
	struct vehicle_local_position_s local_pos_est;
	memset(&local_pos_est, 0, sizeof(local_pos_est));

	/* subscribe */
	int sensor_sub = orb_subscribe(ORB_ID(sensor_combined));
	int sub_params = orb_subscribe(ORB_ID(parameter_update));
	int vehicle_attitude_sub = orb_subscribe(ORB_ID(vehicle_attitude));
	int actuator_sub_fd = orb_subscribe(ORB_ID(actuator_outputs_0));
	int vicon_pos_sub = orb_subscribe(ORB_ID(vehicle_vicon_position));
	int actuator_eff_sub = orb_subscribe(ORB_ID(actuator_controls_effective_0));
	int vehicle_gps_sub = orb_subscribe(ORB_ID(vehicle_gps_position));
	int vehicle_status_sub = orb_subscribe(ORB_ID(vehicle_status));

	/* advertise */
	orb_advert_t local_pos_est_pub = orb_advertise(ORB_ID(vehicle_local_position), &local_pos_est);

	struct position_estimator1D_params pos1D_params;
	struct position_estimator1D_param_handles pos1D_param_handles;
	/* initialize parameter handles */
	parameters_init(&pos1D_param_handles);

	bool local_flag_useGPS = false;
	bool local_flag_useBARO = false;
	bool local_flag_baroINITdone = false; /* in any case disable baroINITdone */
	/* FIRST PARAMETER READ at START UP*/
	struct parameter_update_s update;
	orb_copy(ORB_ID(parameter_update), sub_params, &update); /* read from param to clear updated flag */
	/* FIRST PARAMETER UPDATE */
	parameters_update(&pos1D_param_handles, &pos1D_params);
	local_flag_useBARO = ((pos1D_params.useBARO >= 0.9f) && (pos1D_params.useBARO <= 1.1f));
	local_flag_useGPS = ((pos1D_params.useGPS >= 0.9f) && (pos1D_params.useGPS <= 1.1f));
	viconCntMax = (int)(pos1D_params.viconDivider);
	sigma = pos1D_params.sigma;
	addNoise = pos1D_params.addNoise;
	/* END FIRST PARAMETER UPDATE */

	if(local_flag_useGPS){
		mavlink_log_info(mavlink_fd, "[pos_est1D] I'm using GPS");
		/* wait until gps signal turns valid, only then can we initialize the projection */
		while (gps.fix_type < 3) {
			struct pollfd fds1[2] = {
					{ .fd = vehicle_gps_sub, .events = POLLIN },
					{ .fd = sub_params,   .events = POLLIN },
			};

			/* wait for GPS updates, BUT READ VEHICLE STATUS (!)
			 * this choice is critical, since the vehicle status might not
			 * actually change, if this app is started after GPS lock was
			 * aquired.
			 */
			if (poll(fds1, 2, 5000)) {
				if (fds1[0].revents & POLLIN){
					/* Wait for the GPS update to propagate (we have some time) */
					usleep(5000);
					/* Read wether the vehicle status changed */
					orb_copy(ORB_ID(vehicle_gps_position), vehicle_gps_sub, &gps);
				}
				if (fds1[1].revents & POLLIN){
					/* Read out parameters to check for an update there, e.g. useGPS variable */
					/* read from param to clear updated flag */
					struct parameter_update_s update;
					orb_copy(ORB_ID(parameter_update), sub_params, &update);
					/* update parameters */
					parameters_update(&pos1D_param_handles, &pos1D_params);
					if(!((pos1D_params.useGPS >= 0.9f) && (pos1D_params.useGPS <= 1.1f))){
						local_flag_useGPS = false;
						mavlink_log_info(mavlink_fd, "[pos_est1D] Revert GPS - Goingt to VICON");
						break; /* leave gps fix type 3 while loop */
					}
				}
			}
			static int printcounter = 0;
			if (printcounter == 100) {
				printcounter = 0;
				printf("[pos_est1D] wait for GPS fix type 3\n");
			}
			printcounter++;
		}

		/* check again if useGPS was not aborted and only if not set up tangent plane map initialization*/
		if(local_flag_useGPS){
			/* get gps value for first initialization */
			orb_copy(ORB_ID(vehicle_gps_position), vehicle_gps_sub, &gps);
			lat_current = ((double)(gps.lat)) * 1e-7;
			lon_current = ((double)(gps.lon)) * 1e-7;
			alt_current = gps.alt * 1e-3;
			/* initialize coordinates */
			map_projection_init(lat_current, lon_current);
			/* publish global position messages only after first GPS message */
			printf("[pos_est1D] initialized projection with: lat: %.10f,  lon:%.10f\n", lat_current, lon_current);
		}
	}else{
		mavlink_log_info(mavlink_fd, "[pos_est1D] I'm NOT using GPS - I use VICON");
		/* onboard calculated position estimations */
	}
	uint64_t last_time = 0;
	thread_running = true;

	struct pollfd fds2[3] = {
		{ .fd = vehicle_gps_sub,   .events = POLLIN }, //130 Hz gemaess printf, 260 gemaess beepcounter
		{ .fd = vicon_pos_sub,   .events = POLLIN },
		{ .fd = sub_params,   .events = POLLIN },
	};

	/**< main_loop */
	while (!thread_should_exit) {
		int ret = poll(fds2, 3, 20);  //wait maximal this 20 ms = 50 Hz minimum rate
		if (ret < 0) {
			/* poll error */
		} else {
			if (fds2[0].revents & POLLIN) {
				/* new GPS value */
				orb_copy(ORB_ID(vehicle_gps_position), vehicle_gps_sub, &gps);
				/* do nothing at the moment*/
			}
			if (fds2[2].revents & POLLIN){
				/* new parameter */
				/* read from param to clear updated flag */
				struct parameter_update_s update;
				orb_copy(ORB_ID(parameter_update), sub_params, &update);
				/* update parameters */
				parameters_update(&pos1D_param_handles, &pos1D_params);
				local_flag_useBARO = ((pos1D_params.useBARO >= 0.9f) && (pos1D_params.useBARO <= 1.1f));
				local_flag_useGPS = ((pos1D_params.useGPS >= 0.9f) && (pos1D_params.useGPS <= 1.1f));
				viconCntMax = (int)(pos1D_params.viconDivider);
				sigma = pos1D_params.sigma;
				addNoise = pos1D_params.addNoise;
			}
			static float viconUpdate = 0.0f; /* default is no viconUpdate */
			if (fds2[1].revents & POLLIN) {
				/* new vicon position */
				orb_copy(ORB_ID(vehicle_vicon_position), vicon_pos_sub, &vicon_pos);
				posX = vicon_pos.x;
				posY = vicon_pos.y;
				posZ = vicon_pos.z;
				viconUpdate = 1.0f; /* set flag for vicon update */
			} /* end of poll call for vicon updates */

			/* Main estimator loop */
			orb_copy(ORB_ID(actuator_controls_effective_0), actuator_eff_sub, &act_eff);
			orb_copy(ORB_ID(vehicle_attitude), vehicle_attitude_sub, &att);
			orb_copy(ORB_ID(vehicle_status), vehicle_status_sub, &vehicle_status);
			orb_copy(ORB_ID(sensor_combined), sensor_sub, &sensor);
			// barometric pressure estimation at start up
			if (!local_flag_baroINITdone){
				// mean calculation over several measurements
				if(baro_loop_cnt<baro_loop_end) {
					p0_Pa += (sensor.baro_pres_mbar*100);
					baro_loop_cnt++;
				}else{
					p0_Pa /= (float)(baro_loop_cnt);
					local_flag_baroINITdone = true;
					char *baro_m_start = "barometer initialized with p0 = ";
					char p0_char[15];
					sprintf(p0_char, "%8.2f", p0_Pa/100);
					char *baro_m_end = " mbar";
					char str[80];
					strcpy(str,baro_m_start);
					strcat(str,p0_char);
					strcat(str,baro_m_end);
					mavlink_log_info(mavlink_fd, str);
				}
			}
			if(local_flag_useGPS){
				/* initialize map projection with the last estimate (not at full rate) */
				if (gps.fix_type > 2) {
					/* Project gps lat lon (Geographic coordinate system) to plane*/
					map_projection_project(((double)(gps.lat)) * 1e-7, ((double)(gps.lon)) * 1e-7, &(z[0]), &(z[1]));
					local_pos_est.x = z[0];
					local_pos_est.vx = 0.0f;
					local_pos_est.y = z[1];
					local_pos_est.vy = 0.0f;
					/* negative offset from initialization altitude */
					float z_est = 0.0f;
					if(local_flag_baroINITdone && local_flag_useBARO){
						//printf("use BARO\n");
						K[0] = K_baro[0];
						K[1] = K_baro[1];
						K[2] = K_baro[2];
						z_est = -p0_Pa*log(p0_Pa/(sensor.baro_pres_mbar*100))/(rho0*const_earth_gravity);
					}else{
						//printf("NOT use BARO\n");
						z_est = alt_current - (gps.alt) * 1e-3;
					}
					kalman_dlqe2(dT_const_120,K[0],K[1],K[2],x_z_aposteriori_k,z_est,x_z_aposteriori);
					memcpy(x_z_aposteriori_k, x_z_aposteriori, sizeof(x_z_aposteriori));
					local_pos_est.z = x_z_aposteriori_k[0];
					local_pos_est.vz = x_z_aposteriori_k[1];
					orb_publish(ORB_ID(vehicle_local_position), local_pos_est_pub, &local_pos_est);
				}
			}else{
				/* x-y-position/velocity estimation in earth frame = vicon frame */
				kalman_dlqe3(dT_const_50,K_vicon_50Hz[0],K_vicon_50Hz[1],K_vicon_50Hz[2],x_x_aposteriori_k,posX,viconUpdate,addNoise,sigma,x_x_aposteriori);
				memcpy(x_x_aposteriori_k, x_x_aposteriori, sizeof(x_x_aposteriori));
				kalman_dlqe3(dT_const_50,K_vicon_50Hz[0],K_vicon_50Hz[1],K_vicon_50Hz[2],x_y_aposteriori_k,posY,viconUpdate,addNoise,sigma,x_y_aposteriori);
				memcpy(x_y_aposteriori_k, x_y_aposteriori, sizeof(x_y_aposteriori));
				/* z-position/velocity estimation in earth frame = vicon frame */
				float z_est = 0.0f;
				float local_sigma = 0.0f;
				if(local_flag_baroINITdone && local_flag_useBARO){
					z_est = -p0_Pa*log(p0_Pa/(sensor.baro_pres_mbar*100))/(rho0*const_earth_gravity);
					K[0] = K_vicon_50Hz[0];
					K[1] = K_vicon_50Hz[1];
					K[2] = K_vicon_50Hz[2];
					viconUpdate = 1.0f; /* always enable the update, cause baro update = 200 Hz */
					local_sigma = 0.0f; /* don't add noise on barometer in any case */
				}else{
					z_est = posZ;
					K[0] = K_vicon_50Hz[0];
					K[1] = K_vicon_50Hz[1];
					K[2] = K_vicon_50Hz[2];
					local_sigma = sigma;
				}
				kalman_dlqe3(dT_const_50,K[0],K[1],K[2],x_z_aposteriori_k,z_est,viconUpdate,addNoise,local_sigma,x_z_aposteriori);
				memcpy(x_z_aposteriori_k, x_z_aposteriori, sizeof(x_z_aposteriori));
				local_pos_est.x = x_x_aposteriori_k[0];
				local_pos_est.vx = x_x_aposteriori_k[1];
				//local_pos_est.vx = local_pos_x; //getestet, funktioniert
				local_pos_est.y = x_y_aposteriori_k[0];
				local_pos_est.vy = x_y_aposteriori_k[1];
				local_pos_est.z = x_z_aposteriori_k[0];
				local_pos_est.vz = x_z_aposteriori_k[1];
				//local_pos_est.vz = debug;
				local_pos_est.timestamp = hrt_absolute_time();
				if((isfinite(x_x_aposteriori_k[0])) && (isfinite(x_x_aposteriori_k[1])) && (isfinite(x_y_aposteriori_k[0])) && (isfinite(x_y_aposteriori_k[1])) && (isfinite(x_z_aposteriori_k[0])) && (isfinite(x_z_aposteriori_k[1]))){
					orb_publish(ORB_ID(vehicle_local_position), local_pos_est_pub, &local_pos_est);
				}
			}
		} /* end of poll return value check */
	}

	printf("[pos_est1D] exiting.\n");
	mavlink_log_info(mavlink_fd, "[pos_est1D] exiting");
	thread_running = false;
	return 0;
}
