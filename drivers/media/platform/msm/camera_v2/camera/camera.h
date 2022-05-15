/* Copyright (c) 2012-2014, The Linux Foundation. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#ifndef _CAMERA_H
#define _CAMERA_H

/* Jeff made changes - begining
enum stream_state {
	START_STREAM = 0,
	STOP_STREAM,
};
 * Jeff made changes - end */
enum stream_state {
	CAMERA_START_STREAM = 0,
	CAMERA_STOP_STREAM,
};

/* Jeff added - begin */
enum camera_init_v4l2_t {
	CAMERA_V4L2_SENSOR = 0,
	CAMERA_V4L2_PPROC,
	CAMERA_V4L2_DUMMY,
};
/* Jeff added - end */

int camera_init_v4l2(struct device *dev, unsigned int *session, 
	enum camera_init_v4l2_t type);

#endif /*_CAMERA_H */
