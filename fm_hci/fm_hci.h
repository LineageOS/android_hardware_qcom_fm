/*
 * Copyright (c) 2015, The Linux Foundation. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are
 * met:
 *        * Redistributions of source code must retain the above copyright
 *            notice, this list of conditions and the following disclaimer.
 *        * Redistributions in binary form must reproduce the above
 *            copyright notice, this list of conditions and the following
 *            disclaimer in the documentation and/or other materials provided
 *            with the distribution.
 *        * Neither the name of The Linux Foundation nor the names of its
 *            contributors may be used to endorse or promote products derived
 *            from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED "AS IS" AND ANY EXPRESS OR IMPLIED
 * WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NON-INFRINGEMENT
 * ARE DISCLAIMED.  IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS
 * BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR
 * BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE
 * OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN
 * IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef __FM_HCI__
#define __FM_HCI__

#pragma pack(1)

#include <radio-helium.h>

/** Host/Controller Library Return Status */
typedef enum {
    FM_HC_STATUS_SUCCESS,
    FM_HC_STATUS_FAIL,
    FM_HC_STATUS_NOT_READY,
    FM_HC_STATUS_NOMEM,
    FM_HC_STATUS_BUSY,
    FM_HC_STATUS_CORRUPTED_BUFFER
} fm_hc_status_t;

typedef enum {
    FM_RADIO_DISABLE,
    FM_RADIO_ENABLE
}fm_power_state;

/* Host/Controller lib internal event ID */
#define HC_EVENT_PRELOAD               0x0001
#define HC_EVENT_POSTLOAD              0x0002
#define HC_EVENT_RX                    0x0004
#define HC_EVENT_TX                    0x0008
#define HC_EVENT_LPM_ENABLE            0x0010
#define HC_EVENT_LPM_DISABLE           0x0020
#define HC_EVENT_LPM_WAKE_DEVICE       0x0040
#define HC_EVENT_LPM_ALLOW_SLEEP       0x0080
#define HC_EVENT_LPM_IDLE_TIMEOUT      0x0100
#define HC_EVENT_EXIT                  0x0200
#define HC_EVENT_EPILOG                0x0400

#define MAX_FM_CMD_CNT                 100
#define FM_CMD                         0x11
#define FM_EVT                         0x14
#define MAX_FM_EVT_PARAMS              255

#define FM_CMD_COMPLETE 0x0f
#define FM_CMD_STATUS   0x10
#define FM_HW_ERR_EVENT 0x1A

static pthread_mutex_t utils_mutex;

static volatile uint8_t lib_running = 0;
static volatile uint16_t ready_events = 0;

FILE *fd_wcnss_filter;

typedef struct {
    uint8_t protocol_byte;
    uint16_t opcode;
    uint8_t plen;
    uint8_t cmd_params[];
} FM_HDR;

typedef struct {
    uint8_t protocol_byte;
    uint8_t evt_code;
    uint8_t evt_len;
    uint8_t cmd_params[];
} FM_EVT_HDR;

typedef struct hdr
{
    FM_HDR  *hdr;
    struct hdr *next;
} TX_Q;

int transmit(FM_HDR *pbuf);
int  fm_hci_init(fm_hal_cb *);
int fm_power(fm_power_state state);
int open_serial_port(void);
void fm_userial_close(void);

typedef struct {
    pthread_mutex_t tx_q_lock;
    pthread_mutex_t credit_lock;
    pthread_mutex_t event_lock;

    pthread_cond_t event_cond;
    pthread_cond_t cmd_credits_cond;

    pthread_t fmHALTaskThreadId;
    pthread_t fmHCITaskThreadId;
    pthread_t fmRxTaskThreadId;

    TX_Q *first;
    TX_Q *last;

} fmHCIControlStructure;

fmHCIControlStructure fmHCIControlBlock;

#endif
