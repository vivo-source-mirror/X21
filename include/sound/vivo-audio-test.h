#ifndef _VIVO_AUDIO_TEST_H_
#define _VIVO_AUDIO_TEST_H_

#include <linux/wait.h>
#include <linux/sched.h>
#include <sound/apr_audio-v2.h>
#include <sound/adsp_err.h>
#include <sound/q6afe-v2.h>
#include <sound/q6adm-v2.h>

struct clock_config{
	int enable;
	int port_id;
	uint32_t sample_rate;
	uint32_t channels;
	uint16_t bit_width;

};

struct apr_ctl {
	atomic_t state;
	atomic_t status;
	wait_queue_head_t wait;
};

struct apr_test {
	int enable;;
	int cnt;
	int cmd;
};

struct clock_config* get_mi2s_config(void);
int adsp_subsystem_restart(void);
int vivo_afe_apr_send_pkt(void *data);
void vivo_apr_callback(uint32_t cmd, uint32_t status);
void audio_apr_test_set(int enable, int cmd, int cnt);
#endif
