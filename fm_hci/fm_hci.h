/*
 * Copyright (c) 2015-2016 The Linux Foundation. All rights reserved.
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

#include "bt_hci_bdroid.h"
#include "bt_vendor_lib.h"
#include "fm_hci_api.h"

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
#define HC_EVENT_EXIT_DONE             0x8000

#define MAX_FM_CMD_CNT                 100
#define FM_CMD                         0x11
#define FM_EVT                         0x14
#define MAX_FM_EVT_PARAMS              255

#define FM_CMD_COMPLETE 0x0f
#define FM_CMD_STATUS   0x10
#define FM_HW_ERR_EVENT 0x1A

/* TODO: move inside context */
static volatile uint8_t lib_running = 0;
static volatile uint16_t ready_events = 0;

// The set of events one can send to the userial read thread.
// Note that the values must be >= 0x8000000000000000 to guarantee delivery
// of the message (see eventfd(2) for details on blocking behaviour).
enum {
    USERIAL_RX_EXIT     = 0x8000000000000000ULL
};

struct transmit_queue_t {
    struct fm_command_header_t *hdr;
    struct transmit_queue_t *next;
};

struct fm_hci_t {
    int fd;
    pthread_mutex_t credit_lock;
    pthread_cond_t cmd_credits_cond;

    pthread_mutex_t event_lock;
    pthread_cond_t event_cond;

    pthread_t hal_thread;
    pthread_t tx_thread;
    pthread_t rx_thread;
    pthread_t mon_thread;

    pthread_mutex_t tx_q_lock;
    struct transmit_queue_t *first;
    struct transmit_queue_t *last;

    void *dlhandle;
    bt_vendor_interface_t *vendor;

    struct fm_hci_callbacks_t *cb;
    void *private_data;
    volatile uint16_t command_credits;
};

#endif

