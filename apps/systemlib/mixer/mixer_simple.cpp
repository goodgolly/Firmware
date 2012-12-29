/****************************************************************************
 *
 *   Copyright (C) 2012 PX4 Development Team. All rights reserved.
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
 * @file mixer_simple.cpp
 *
 * Simple summing mixer.
 */

#include <nuttx/config.h>

#include <sys/types.h>
#include <stdint.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>
#include <fcntl.h>
#include <poll.h>
#include <errno.h>
#include <stdio.h>
#include <math.h>
#include <unistd.h>
#include <ctype.h>

#include "mixer.h"

#define debug(fmt, args...)	do { } while(0)
//#define debug(fmt, args...)	do { printf("[mixer] " fmt "\n", ##args); } while(0)

SimpleMixer::SimpleMixer(ControlCallback control_cb,
			 uintptr_t cb_handle,
			 mixer_simple_s *mixinfo) :
	Mixer(control_cb, cb_handle),
	_info(mixinfo)
{
}

SimpleMixer::~SimpleMixer()
{
	if (_info != nullptr)
		free(_info);
}

static const char *
skipspace(const char *p, unsigned &len)
{
	while (isspace(*p)) {
		if (len == 0)
			return nullptr;
		len--;
		p++;
	}
	return p;
}

int
SimpleMixer::parse_output_scaler(const char *buf, unsigned &buflen, mixer_scaler_s &scaler)
{
	int ret;
	int s[5];
	int used;

	buf = skipspace(buf, buflen);
	if (buflen < 16)
		return -1;

	if ((ret = sscanf(buf, "O: %d %d %d %d %d%n",
		   &s[0], &s[1], &s[2], &s[3], &s[4], &used)) != 5) {
		debug("scaler parse failed on '%s' (got %d)", buf, ret);
		return -1;
	}
	buflen -= used;

	scaler.negative_scale	= s[0] / 10000.0f;
	scaler.positive_scale	= s[1] / 10000.0f;
	scaler.offset		= s[2] / 10000.0f;
	scaler.min_output	= s[3] / 10000.0f;
	scaler.max_output	= s[4] / 10000.0f;

	return 0;
}

int
SimpleMixer::parse_control_scaler(const char *buf, unsigned &buflen, mixer_scaler_s &scaler, uint8_t &control_group, uint8_t &control_index)
{
	unsigned u[2];
	int s[5];
	int used;

	buf = skipspace(buf, buflen);
	if (buflen < 16)
		return -1;

	if (sscanf(buf, "S: %u %u %d %d %d %d %d%n",
		   &u[0], &u[1], &s[0], &s[1], &s[2], &s[3], &s[4], &used) != 7) {
		debug("control parse failed on '%s'", buf);
		return -1;
	}
	buflen -= used;

	control_group		= u[0];
	control_index		= u[1];
	scaler.negative_scale	= s[0] / 10000.0f;
	scaler.positive_scale	= s[1] / 10000.0f;
	scaler.offset		= s[2] / 10000.0f;
	scaler.min_output	= s[3] / 10000.0f;
	scaler.max_output	= s[4] / 10000.0f;

	return 0;
}

SimpleMixer *
SimpleMixer::from_text(Mixer::ControlCallback control_cb, uintptr_t cb_handle, const char *buf, unsigned &buflen)
{
	SimpleMixer *sm = nullptr;
	mixer_simple_s *mixinfo = nullptr;
	unsigned inputs;
	int used;
	const char *end = buf + buflen;

	/* get the base info for the mixer */
	if (sscanf(buf, "M: %u%n", &inputs, &used) != 1) {
		debug("simple parse failed on '%s'", buf);
		goto out;
	}
	buflen -= used;

	mixinfo = (mixer_simple_s *)malloc(MIXER_SIMPLE_SIZE(inputs));
	if (mixinfo == nullptr) {
		debug("could not allocate memory for mixer info");
		goto out;
	}
	mixinfo->control_count = inputs;

	if (parse_output_scaler(end - buflen, buflen, mixinfo->output_scaler))
		goto out;

	for (unsigned i = 0; i < inputs; i++) {
		if (parse_control_scaler(end - buflen, buflen,
			mixinfo->controls[i].scaler,
			mixinfo->controls[i].control_group,
			mixinfo->controls[i].control_index))
			goto out;
	}

	sm = new SimpleMixer(control_cb, cb_handle, mixinfo);
	if (sm != nullptr) {
		mixinfo = nullptr;
		debug("loaded mixer with %d inputs", inputs);
	} else {
		debug("could not allocate memory for mixer");
	}

out:
	if (mixinfo != nullptr)
		free(mixinfo);

	return sm;
}

unsigned
SimpleMixer::mix(float *outputs, unsigned space)
{
	float		sum = 0.0f;

	if (_info == nullptr)
		return 0;

	if (space < 1)
		return 0;

	for (unsigned i = 0; i < _info->control_count; i++) {
		float input;

		_control_cb(_cb_handle,
			    _info->controls[i].control_group,
			    _info->controls[i].control_index,
			    input);

		sum += scale(_info->controls[i].scaler, input);
	}

	*outputs = scale(_info->output_scaler, sum);
	return 1;
}

void
SimpleMixer::groups_required(uint32_t &groups)
{
	for (unsigned i = 0; i < _info->control_count; i++)
		groups |= 1 << _info->controls[i].control_group;
}

int
SimpleMixer::check()
{
	int ret;
	float junk;

	/* sanity that presumes that a mixer includes a control no more than once */
	/* max of 32 groups due to groups_required API */
	if (_info->control_count > 32)
		return -2;

	/* validate the output scaler */
	ret = scale_check(_info->output_scaler);

	if (ret != 0)
		return ret;

	/* validate input scalers */
	for (unsigned i = 0; i < _info->control_count; i++) {

		/* verify that we can fetch the control */
		if (_control_cb(_cb_handle,
				_info->controls[i].control_group,
				_info->controls[i].control_index,
				junk) != 0) {
			return -3;
		}

		/* validate the scaler */
		ret = scale_check(_info->controls[i].scaler);

		if (ret != 0)
			return (10 * i + ret);
	}

	return 0;
}