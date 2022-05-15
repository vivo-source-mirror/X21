/* Copyright (c) 2012-2015, The Linux Foundation. All rights reserved.
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#ifndef _VIVO_KTV_CONFIG_H_
#define _VIVO_KTV_CONFIG_H_

#include <sound/soc.h>


void vivo_ktv_add_controls(struct snd_soc_platform *platform);
void set_vivo_ktv_copp_idx(int topology, int copp_idx, int port_id);
void vivo_ktv_init(void);
void vivo_ktv_reset(void);
int ktv_basic_callback(uint32_t *payload, u32 payload_size);
void ktv_params_callback(uint32_t *payload, u32 payload_size);

#endif

