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
#include <uORB/topics/parameter_update.h>
//#include <uORB/topics/sensor_combined.h>
#include <uORB/topics/vehicle_attitude.h>
#include <uORB/topics/actuator_outputs.h>
#include <uORB/topics/actuator_controls_effective.h>
#include <uORB/topics/vehicle_status.h>
#include <uORB/topics/vehicle_vicon_position.h>
#include <uORB/topics/vehicle_local_position.h>
#include <uORB/topics/vehicle_gps_position.h>
#include <mavlink/mavlink_log.h>
#include <poll.h>
#include <systemlib/geo/geo.h>

#include "position_estimator1D_params.h"
//#include <uORB/topics/debug_key_value.h>
#include "codegen/kalman_dlqe2.h"



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
float thrust2force(float thrust)
{
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
	//static float mass = 0.448f;
	//static float g = 9.81f;

	//bool velocity_observe_x = 0;
	//bool velocity_observe_y = 0;
	//bool gps_update = 0;
	//bool stateFlying = 0;
	bool useGPS = true;
	bool gps_valid = false;
	static float z[3] = {0, 0, 0};

	float x_x_aposteriori_k[3] = {1.0f, 0.0f, 0.0f};
	float x_y_aposteriori_k[3] = {1.0f, 0.0f, 0.0f};

	float x_x_aposteriori[3] = {0.0f, 0.0f, 0.0f};
	float x_y_aposteriori[3] = {1.0f, 0.0f, 0.0f};

	//static float P_x_apriori[9] = {100.0f, 0.0f, 0.0f,
	//					   0.0f, 100.0f, 0.0f,
	//					   0.0f, 0.0f, 100.0f};
	//static float P_y_apriori[9] = {100.0f, 0.0f, 0.0f,
	//					   0.0f, 100.0f, 0.0f,
	//					   0.0f, 0.0f, 100.0f};

	//static float P_x_aposteriori[9] = {0.0f ,0.0f, 0.0f, 0.0f ,0.0f, 0.0f, 0.0f ,0.0f, 0.0f};
	//static float P_y_aposteriori[9] = {0.0f ,0.0f, 0.0f, 0.0f ,0.0f, 0.0f, 0.0f ,0.0f, 0.0f};

	const float dT_const = 1.0f/120.0f;
	const float ampl = 2.5f;
	const float st = 0.5f*dT_const*dT_const;

	const float Ad[9] = {1.0f, dT_const, st,
				   0.0f, 1.0f, dT_const,
				   0.0f, 0.0f, 1.0f};

	const float Adtransponiert[9] = 	{1.0f, 0.0f, 0.0f,
								dT_const, 1.0f, 0.0f,
								st, dT_const, 1.0f};
	//float Bd[3] = {0.5f*dT_const*dT_const, dT_const, 0.0f};
	const float Cd[3] = {1.0f, 0.0f, 0.0f};
	//const float cv = dT_const*dT_const*ampl*ampl;
	//const float cv2 = 1.0f;
	//const float mp = 0.000001f;

	//float Q[9] = {cv*st*st, cv*st*dT_const, cv*st,
	//			  cv*dT_const*st,cv*dT_const*dT_const, cv*dT_const,
	//			  cv*st, cv*dT_const, cv};

	//float R = mp;

	//computed from dlqe
	const float K[3] = {0.2151f, 2.9154f, 19.7599f};

	float acc_x_body = 0.0f;
	float acc_y_body = 0.0f;
	float acc_z_body = 0.0f;

	float acc_x_e = 0.0f;
	float acc_y_e = 0.0f;
	float acc_z_e = 0.0f;

	float z_x = 0.0f;
	float z_y = 0.0f;

	float flyingT = 200.0f;

	float accThres = 0.001f;
	float velDecay = 0.995f;

	float rotMatrix[4] = {1.0f,  0.0f,
						  0.0f,  1.0f};

	int loopcounter = 0;
	int debugCNT = 0;

	static double lat_current = 0.0d; //[°]] --> 47.0
	static double lon_current = 0.0d; //[°]] -->8.5
	static double alt_current = 0.0d; //[m] above MSL

	/* Initialize filter */
	kalman_dlqe2_initialize();

	struct vehicle_attitude_s att;
	struct vehicle_status_s vehicle_status;
	struct vehicle_vicon_position_s vicon_pos;
	struct actuator_controls_effective_s act_eff;
	struct vehicle_gps_position_s gps;
	gps.fix_type = 0;

	/* subscribe to param changes */
	int sub_params = orb_subscribe(ORB_ID(parameter_update));
	/* subscribe to attitude at 100 Hz */
	int vehicle_attitude_sub = orb_subscribe(ORB_ID(vehicle_attitude));
	/* subscribe to actuator_ouputs at ??? Hz */
	int actuator_sub_fd = orb_subscribe(ORB_ID(actuator_outputs_0));
	/* subscribe to vehicle_status ??? Hz */
	int vehicle_status_sub = orb_subscribe(ORB_ID(vehicle_status));
	/* subscribe to Vicon Position Hz */
	int vicon_pos_sub = orb_subscribe(ORB_ID(vehicle_vicon_position));
	/* actuator effective */
	int actuator_eff_sub = orb_subscribe(ORB_ID(actuator_controls_effective_0));
	/* GPS */
	int vehicle_gps_sub = orb_subscribe(ORB_ID(vehicle_gps_position));

	struct position_estimator1D_params pos1D_params;
	struct position_estimator1D_param_handles pos1D_param_handles;

	/* initialize parameter handles */
	parameters_init(&pos1D_param_handles);
	parameters_update(&pos1D_param_handles, &pos1D_params);

	//struct debug_key_value_s dbg1 = { .key = "dbg:x_pos_err", .value = 0.0f };
	//struct debug_key_value_s dbg2 = { .key = "dbg:y_pos_err", .value = 0.0f };

	//orb_advert_t pub_dbg1 = orb_advertise(ORB_ID(debug_key_value), &dbg1);
	//orb_advert_t pub_dbg2 = orb_advertise(ORB_ID(debug_key_value), &dbg2);

	/* one could wait for multiple topics with this technique, just using one here */
	struct pollfd fds[2] = {
		{ .fd = vehicle_attitude_sub,   .events = POLLIN },
		{ .fd = sub_params,   .events = POLLIN },
		/* there could be more file descriptors here, in the form like:
		 * { .fd = other_sub_fd,   .events = POLLIN },
		 */
	};

	uint64_t last_time = 0;

	if(useGPS){
		mavlink_log_info(mavlink_fd, "We are using GPS");
		/* wait until gps signal turns valid, only then can we initialize the projection */
		while (gps.fix_type < 3) {
			struct pollfd fds[1] = { {.fd = vehicle_gps_sub, .events = POLLIN} };

			/* wait for GPS updates, BUT READ VEHICLE STATUS (!)
			 * this choice is critical, since the vehicle status might not
			 * actually change, if this app is started after GPS lock was
			 * aquired.
			 */
			if (poll(fds, 1, 5000)) {
				/* Wait for the GPS update to propagate (we have some time) */
				usleep(5000);
				/* Read wether the vehicle status changed */
				orb_copy(ORB_ID(vehicle_gps_position), vehicle_gps_sub, &gps);
				gps_valid = (gps.fix_type > 2);
			}
			printf("wait for fix type 3");
		}

		/* get gps value for first initialization */
		orb_copy(ORB_ID(vehicle_gps_position), vehicle_gps_sub, &gps);
		lat_current = ((double)(gps.lat)) * 1e-7;
		lon_current = ((double)(gps.lon)) * 1e-7;
		alt_current = gps.alt * 1e-3;

		/* initialize coordinates */
		map_projection_init(lat_current, lon_current);

		/* publish global position messages only after first GPS message */
		printf("[multirotor position estimator] initialized projection with: lat: %.10f,  lon:%.10f\n", lat_current, lon_current);
	}else{
		mavlink_log_info(mavlink_fd, "We are NOT using GPS");
		/* onboard calculated position estimations */
	}
	struct vehicle_local_position_s local_pos_est = {
		.x = 0,
		.y = 0,
		.z = 0,
		.vx = 0,
		.vy = 0,
		.vz = 0
	};
	orb_advert_t local_pos_est_pub = orb_advertise(ORB_ID(vehicle_local_position), &local_pos_est);

	thread_running = true;

	/* main_loop */
	while (!thread_should_exit) {
		/* wait for sensor update of 1 file descriptor for 1000 ms (1 second) */
		int ret = poll(fds, 2, 500);  //wait maximal this time
		if (ret < 0) {

		} else if (ret == 0) {
			/* no return value, ignore */
		} else {
			if (fds[1].revents & POLLIN){
				/* read from param to clear updated flag */
				struct parameter_update_s update;
				orb_copy(ORB_ID(parameter_update), sub_params, &update);

				/* update parameters */
				parameters_update(&pos1D_param_handles, &pos1D_params);
				//Q[0] = pos1D_params.QQ[0];
				//Q[1] = 0.0f;
				//Q[2] = 0.0f;
				//Q[3] = 0.0f;
				//Q[4] = pos1D_params.QQ[1];
				//Q[5] = 0.0f;
				//Q[6] = 0.0f;
				//Q[7] = 0.0f;
				//Q[8] = pos1D_params.QQ[2];
				//R = pos1D_params.R;
				float useGPStemp = pos1D_params.useGPS;
				if(useGPStemp == 0.0f){
					useGPS = false;
				}else if(useGPStemp == 1.0f){
					useGPS = true;
				}
				velDecay = pos1D_params.velDecay;
				flyingT = pos1D_params.flyingT;
				accThres = pos1D_params.accThres;
				printf("[position_estimator1D] updated parameter: %8.4f\n", accThres);
			}
			if (fds[0].revents & POLLIN) {
				/* obtained data for the first file descriptor */

				/* copy actuator raw data into local buffer */
				orb_copy(ORB_ID(actuator_controls_effective_0), actuator_eff_sub, &act_eff);
				orb_copy(ORB_ID(vehicle_attitude), vehicle_attitude_sub, &att);
				orb_copy(ORB_ID(vehicle_status), vehicle_status_sub, &vehicle_status);
				if(useGPS){
					/* get a local copy of gps position */
					orb_copy(ORB_ID(vehicle_gps_position), vehicle_gps_sub, &gps);
				}else{
					/* get a local copy of local vicon position */
					orb_copy(ORB_ID(vehicle_vicon_position), vicon_pos_sub, &vicon_pos);
				}

				if(useGPS){
					/* initialize map projection with the last estimate (not at full rate) */
					if (gps.fix_type > 2) {
						/* Project gps lat lon (Geographic coordinate system) to plane*/
						map_projection_project(((double)(gps.lat)) * 1e-7, ((double)(gps.lon)) * 1e-7, &(z[0]), &(z[1]));
						local_pos_est.x = z[0];
						local_pos_est.y = z[1];
						/* negative offset from initialization altitude */
						local_pos_est.z = alt_current - (gps.alt) * 1e-3;
						orb_publish(ORB_ID(vehicle_local_position), local_pos_est_pub, &local_pos_est);
					}
				}else{
					kalman_dlqe2(dT_const,K[0],K[1],K[2],x_x_aposteriori_k,vicon_pos.x,x_x_aposteriori);
					memcpy(x_x_aposteriori_k, x_x_aposteriori, sizeof(x_x_aposteriori));
					kalman_dlqe2(dT_const,K[0],K[1],K[2],x_y_aposteriori_k,vicon_pos.y,x_y_aposteriori);
					memcpy(x_y_aposteriori_k, x_y_aposteriori, sizeof(x_y_aposteriori));
					local_pos_est.x = x_x_aposteriori_k[0];
					local_pos_est.vx = x_x_aposteriori_k[1];
					local_pos_est.y = x_y_aposteriori_k[0];
					local_pos_est.vy = x_y_aposteriori_k[1];
					local_pos_est.timestamp = hrt_absolute_time();
					orb_publish(ORB_ID(vehicle_local_position), local_pos_est_pub, &local_pos_est);
					//printf("[dqle2] x: %8.4f\t %8.4f\t y: %8.4f\t %8.4f\n", (double)(x_x_aposteriori[0]),(double)(x_x_aposteriori[1]), (double)(x_y_aposteriori[0]), (double)(x_y_aposteriori[1]));
				}

				static int printcounter = 0;
				if (printcounter == 200) {
					printcounter = 0;
					printf("local_pos_est x: %d cm\ty: %d cm\n", (int)(local_pos_est.x*100), (int)(local_pos_est.y*100));
				}
				printcounter++;
			} /* end of poll call for vicon updates */
		} /* end of poll return value check */
	}

	printf("[position_estimator1D] exiting.\n");
	mavlink_log_info(mavlink_fd, "[position_estimator1D] exiting");
	thread_running = false;
	return 0;
}
