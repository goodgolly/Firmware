/****************************************************************************
 *
 *   Copyright (C) 2012 PX4 Development Team. All rights reserved.
 *   Author: 	Damian Aregger <daregger@student.ethz.ch>
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

/*
 * @file position_estimator1D_params.c
 * 
 * Parameters for position_estimator1D
 */

#include "position_estimator1D_params.h"

/* Kalman Filter covariances */

/* gps process noise */
PARAM_DEFINE_FLOAT(POS_EST_Q11, 1e-1f);
PARAM_DEFINE_FLOAT(POS_EST_Q22, 1e-1f);
PARAM_DEFINE_FLOAT(POS_EST_Q33, 0.0f);

/* gps measurement noise standard deviation */
PARAM_DEFINE_FLOAT(POS_EST_SIGMA, 1.0f);
PARAM_DEFINE_FLOAT(POS_EST_R, 1.0f);

PARAM_DEFINE_FLOAT(POS_EST_useGPS, 0.0f);
PARAM_DEFINE_FLOAT(POS_EST_useBARO, 0.0f);
PARAM_DEFINE_FLOAT(POS_EST_accThres, 0.1f);
PARAM_DEFINE_FLOAT(POS_EST_flyingT, 230.0f);
PARAM_DEFINE_FLOAT(POS_EST_velDecay, 0.995f);

int parameters_init(struct position_estimator1D_param_handles *h)
{
	/* Kalman Filter Parameter*/
	h->q11 	=	param_find("POS_EST_Q11");
	h->q22 	=	param_find("POS_EST_Q22");
	h->q33 	=	param_find("POS_EST_Q33");
	h->sigma 	=	param_find("POS_EST_SIGMA");
	h->r 	=	param_find("POS_EST_R");
	h->useGPS_param_handle = param_find("POS_EST_useGPS");
	h->useBARO_param_handle = param_find("POS_EST_useBARO");
	h->accThreshold_param_handle = param_find("POS_EST_accThres");
	h->flyingThreshold_param_handle = param_find("POS_EST_flyingT");
	h->velDecay_param_handle = param_find("POS_EST_velDecay");
	return OK;
}

int parameters_update(const struct position_estimator1D_param_handles *h, struct position_estimator1D_params *p)
{
	param_get(h->q11, &(p->QQ[0]));
	param_get(h->q22, &(p->QQ[1]));
	param_get(h->q33, &(p->QQ[2]));
	param_get(h->sigma, &(p->sigma));
	param_get(h->r, &(p->R));
	param_get(h->useGPS_param_handle, &(p->useGPS));
	param_get(h->useBARO_param_handle, &(p->useBARO));
	param_get(h->accThreshold_param_handle, &(p->accThres));
	param_get(h->flyingThreshold_param_handle, &(p->flyingT));
	param_get(h->velDecay_param_handle, &(p->velDecay));
	return OK;
}
