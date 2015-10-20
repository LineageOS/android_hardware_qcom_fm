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

#define LOG_TAG "fm_hci_helium"

#include <assert.h>
#include <utils/Log.h>

#include "bt_hci_bdroid.h"
#include "bt_vendor_lib.h"
#include "hci.h"
#include "userial.h"
#include "utils.h"
#include "fm_hci.h"
#include "wcnss_hci.h"
#include <dlfcn.h>
#include <sys/eventfd.h>
#include <errno.h>

int fm_fd;
fm_hal_cb *hal_cb;

void event_notification(uint16_t event)
{
    pthread_mutex_lock(&fmHCIControlBlock.event_lock);
    ready_events |= event;
    pthread_cond_signal(&fmHCIControlBlock.event_cond);
    ALOGI("%s: Notifying worker thread with event: %d", __func__, event);
    pthread_mutex_unlock(&fmHCIControlBlock.event_lock);
}

bt_vendor_interface_t *fm_vnd_if = NULL;
void init_vnd_if()
{
    void *dlhandle;
    unsigned char bdaddr[] = {0xaa, 0xbb, 0xcc, 0x11, 0x22, 0x33};

    dlhandle = dlopen("libbt-vendor.so", RTLD_NOW);
    if (!dlhandle)
    {
        ALOGE("!!! Failed to load libbt-vendor.so !!!");
        return;
    }

    fm_vnd_if = (bt_vendor_interface_t *) dlsym(dlhandle, "BLUETOOTH_VENDOR_LIB_INTERFACE");
    if (!fm_vnd_if)
    {
        ALOGE("!!! Failed to get bt vendor interface !!!");
        return;
    }

    ALOGI("FM-HCI: Registering the WCNSS HAL library by passing CBs and BD addr.");
    fm_vnd_if->init(&fm_vendor_callbacks, bdaddr);
}

volatile uint16_t command_credits;

/* De-queues the FM CMD from the TX_Q */
void dequeue_fm_tx_cmd()
{
	TX_Q *new_first, *new_last;
	static int cmd_count = 0;
	static uint8_t credits = 0;
	uint8_t i;
	uint8_t temp_1 = 0x11;

	if (cmd_count >= MAX_FM_CMD_CNT) {
		ALOGI("\n\n\t\tReached Max. CMD COUNT!!\n\n");
		lib_running = 0;
		return;
	}

	/*
	 * Save the 'first' pointer and make it NULL.
	 * This is to allow the FM-HAL to enqueue more CMDs to the TX_Q
	 * without having to contend for the 'tx_q_lock' with the FM-HCI thread.
	 * Once the pointer to the 'first' element in the TX_Q is available,
	 * send all the commands in the queue to WCNSS FILTER based on the
	 * command credits provided by the Controller. If command credits are
	 * not available, then wait for the same.
	 */
	pthread_mutex_lock(&fmHCIControlBlock.tx_q_lock);
	if (!fmHCIControlBlock.first) {
		ALOGI("No FM CMD available in the Q\n");
		pthread_mutex_unlock(&fmHCIControlBlock.tx_q_lock);
		return;
	}
	else {
		new_first = fmHCIControlBlock.first;
		new_last  = fmHCIControlBlock.last;
		fmHCIControlBlock.first = fmHCIControlBlock.last = NULL;
	}
	pthread_mutex_unlock(&fmHCIControlBlock.tx_q_lock);

	//credits = command_credits;

	TX_Q *temp = new_first;
	while(temp != NULL) {

wait_for_cmd_credits:
		pthread_mutex_lock(&fmHCIControlBlock.credit_lock);
		while (command_credits == 0) {
			ALOGI("\n\n\t\tWaiting for COMMAND CREDITS from CONTROLLER\n\n");
			pthread_cond_wait(&fmHCIControlBlock.cmd_credits_cond, &fmHCIControlBlock.credit_lock);
		}
		pthread_mutex_unlock(&fmHCIControlBlock.credit_lock);

		/* Check if we really got the command credits */
		//REVISIT this area
		//if (credits) {
		if (command_credits) {
			ALOGI("%s: Sending the FM-CMD(prot_byte: 0x%x): 0x%x dequeued from TX_Q\n", __func__, temp->hdr->protocol_byte, temp->hdr->opcode);

			if (temp->hdr->plen) {
				ALOGI("%s: CMD-PARAMS:", __func__);
				for (i = 0; i < temp->hdr->plen; i++)
					ALOGI(" <0x%x> ", temp->hdr->cmd_params[i]);
			} else
				ALOGE("%s: NO CMD-PARAMS available for this command", __func__);

			ALOGE("%s: Sizeof FM_HDR: %d", __func__, (int)sizeof(temp->hdr));
			/* Use socket 'fd' to send the command down to WCNSS Filter */
			write(fm_fd, (uint8_t *)temp->hdr, (sizeof(FM_HDR) + temp->hdr->plen));
			//write(fd, &temp_1, 1);

			/* Decrement cmd credits by '1' after sending the cmd*/
			command_credits--;

			/* TODO:
			 * Initialize 'cmd_cnt' to MAX_FM_CMD(?). Should we have any limit on the
			 * number of outstanding commands in the TX-Q ??
			 */
			cmd_count--;

			/* Fetch the next cmd to be sent */
			temp = temp->next;
		} else {
			if (!lib_running)
				break;

			ALOGI("\n\n\t\tFalse wakeup: Yet to get COMMAND CREDITS from CONTROLLER\n\n");
			goto wait_for_cmd_credits;
		}
	}
}


static int event_fd = -1;

static inline int add_event_fd(fd_set *set) {
    if (event_fd == -1) {
      event_fd = eventfd(0, 0);
      if (event_fd == -1) {
          ALOGE("%s unable to create event fd: %s", __func__, strerror(errno));
          return -1;
      }
    }

    FD_SET(event_fd, set);
    return event_fd;
}

static inline uint64_t read_event() {
    assert(event_fd != -1);

    uint64_t value = 0;
    eventfd_read(event_fd, &value);
    return value;
}
static inline void fm_send_event(uint64_t event_id) {
    assert(event_fd != -1);
    eventfd_write(event_fd, event_id);
}

static int read_fm_event(int fd, FM_EVT_HDR *pbuf, int len)
{
    fd_set readFds;
    int n = 0, ret = -1, evt_type = -1, evt_len = -1;

    while (lib_running)
    {
        FD_ZERO(&readFds);
        FD_SET(fd, &readFds);

        if (event_fd == -1) {
            event_fd = eventfd(0, 0);
            if (event_fd == -1) {
                ALOGE("%s: unable to create event fd: %s", __func__, strerror(errno));
                return -1;
            }
        }
        FD_SET(event_fd, &readFds);
        int fd_max = (event_fd > fd ? event_fd : fd);

        ALOGE("%s: Waiting for events from WCNSS FILTER...\n", __func__);

        /* Wait for event/data from WCNSS Filter */
        n = select(fd_max+1, &readFds, NULL, NULL, NULL);
        if (n > 0)
        {
            /* Check if event is available or not */
#if 1
            if (FD_ISSET(fd, &readFds)) {
                ret = read(fd, (uint8_t *)pbuf, (size_t)(sizeof(FM_EVT_HDR) + MAX_FM_EVT_PARAMS));
                if (0 == ret) {
                    ALOGE("%s: read() returned '0' bytes\n", __func__);
                }
                else {
                    ALOGE("%s: read() returned %d bytes of FM event/data\n", __func__, ret);
                    while (ret > 0) {
                        if (pbuf->evt_code == FM_CMD_COMPLETE) {
                            ALOGE("\n\t%s: Received %d bytes of CC event data from WCNSS FILTER!!!\n\t"
                                "Evt type\t: 0x%x \n\tEvt Code\t: 0x%x \n\tEvt len\t\t: 0x%x \n\topcode\t\t: 0x%x%x \n\tCmd Credits\t: 0x%x \n\tStatus\t\t: 0x%x\n",
                                __func__, ret, pbuf->protocol_byte, pbuf->evt_code, pbuf->evt_len, pbuf->cmd_params[2], pbuf->cmd_params[1],
                            pbuf->cmd_params[0], pbuf->cmd_params[3]);
                            evt_type = FM_CMD_COMPLETE;
                        } else if (pbuf->evt_code == FM_CMD_STATUS) {
                            ALOGE("\n\t%s: Received %d bytes of CS event data from WCNSS FILTER!!!\n\t"
                                "Evt type\t: 0x%x \n\tEvt Code\t: 0x%x \n\tEvt len\t\t: 0x%x \n\topcode\t\t: 0x%x%x \n\tCmd Credits\t: 0x%x \n\tStatus\t\t: 0x%x\n",
                                __func__, ret, pbuf->protocol_byte, pbuf->evt_code, pbuf->evt_len, pbuf->cmd_params[3], pbuf->cmd_params[2],
                            pbuf->cmd_params[1], pbuf->cmd_params[0]);
                            evt_type = FM_CMD_STATUS;
                        } else if (pbuf->evt_code == FM_HW_ERR_EVENT) {
                              ALOGI("%s: FM H/w Err Event Recvd. Event Code: 0x%2x", __func__, pbuf->evt_code);
                              lib_running =0;
                              // commented till bt vendor include added
                             // fm_vnd_if->ssr_cleanup();

                        } else {
                            ALOGI("%s: Not CS/CC Event: Recvd. Event Code: 0x%2x", __func__, pbuf->evt_code);
                            evt_type = -1;
                        }

                        evt_len = pbuf->evt_len;

                        /* Notify 'fmHCITask' about availability of event or data */
                        ALOGE("%s: \nNotifying 'fmHCITask' availability of FM event or data...\n", __func__);
                        event_notification(HC_EVENT_RX);

                        if (hal_cb && hal_cb->fm_evt_notify != NULL)
                            hal_cb->fm_evt_notify((uint8_t *)pbuf);
                        else
                            ALOGE("%s: ASSERT $$$$$$ Callback function NULL $$$$$", __func__);

                        if((evt_type == FM_CMD_STATUS) || (evt_type == FM_CMD_COMPLETE)) {
                            /* Provide command credits to allow fmHCITask to send cmds */
                            pthread_mutex_lock(&fmHCIControlBlock.credit_lock);
                            if (evt_type == FM_CMD_COMPLETE) {
                                ALOGE("\n%s: Command Credit(s): '%d' received as part of CC Event for FM-CMD: 0x%x%x \n", __func__, pbuf->cmd_params[0],
                                     pbuf->cmd_params[2], pbuf->cmd_params[1]);
                                command_credits = pbuf->cmd_params[0];
                            } else if (evt_type == FM_CMD_STATUS) {
                                ALOGE("\n%s: Command Credit(s): '%d' received as part of CS Event for FM-CMD: 0x%x%x \n", __func__, pbuf->cmd_params[1],
                                    pbuf->cmd_params[3], pbuf->cmd_params[2]);
                                command_credits = pbuf->cmd_params[1];
                            }
                            pthread_cond_signal(&fmHCIControlBlock.cmd_credits_cond);
                            pthread_mutex_unlock(&fmHCIControlBlock.credit_lock);
                        }

                        ret = ret - (evt_len + 3);
                        ALOGD("%s: Length of available bytes @ HCI Layer: %d", __func__, ret);

                        if (ret > 0) {
                            ALOGD("%s: Remaining bytes of event/data: %d", __func__, ret);
                            pbuf = (FM_EVT_HDR *)&pbuf->cmd_params[evt_len];
                            ALOGD("%s: Protocol byte of next packet: 0x%2x", __func__, pbuf[0]);
                        }

                    }
                } //end of processing the event

            } else
                ALOGE("%s: No data available, though select returned!!!\n", __func__);
#endif
        }
        else if (n < 0) {
           ALOGE("%s: select() failed with return value: %d", __func__, ret);
           lib_running =0;
        }
        else if (n == 0)
            ALOGE("%s: select() timeout!!!", __func__);
    }

    return ret;
}

static void *userial_read_thread(void *arg)
{
	int length;

       FM_EVT_HDR *evt_buf = (FM_EVT_HDR *) malloc(sizeof(FM_EVT_HDR) + MAX_FM_EVT_PARAMS);

       ALOGE("%s: Wait for events from the WCNSS Filter", __func__);
       length = read_fm_event(fm_fd, evt_buf, sizeof(FM_EVT_HDR) + MAX_FM_EVT_PARAMS);
       ALOGE("length=%d\n",length);
       if(length <=0){
         lib_running =0;
       }
       ALOGE("%s: Leaving userial_read_thread()", __func__);
       pthread_exit(NULL);
       return arg;
}

/*
 * Reads the FM-CMDs from the TX_Q and sends it down to WCNSS Filter
 * Reads events sent by the WCNSS Filter and copies onto RX_Q
 */
static void* fmHCITask(void *arg)
{
	static uint16_t events;

	while (lib_running) {
		pthread_mutex_lock(&fmHCIControlBlock.event_lock);
		while (ready_events == 0) {
			pthread_cond_wait(&fmHCIControlBlock.event_cond, &fmHCIControlBlock.event_lock);
		}

		events = ready_events;
		ready_events = 0;
		pthread_mutex_unlock(&fmHCIControlBlock.event_lock);

		if ((events & 0xFFF8) == HC_EVENT_TX) {
			ALOGI("\n@@@@@ FM-HCI Task : EVENT_TX available @@@@@\n");
			dequeue_fm_tx_cmd();
		}
		if ((events & 0xFFF4) == HC_EVENT_RX) {
			ALOGI("\n##### FM-HCI Task : EVENT_RX available #####\n");
			//TODO: Notify FM-HAL about event/data availablity
		}
	}
        ALOGE("%s: ##### Exiting fmHCITask Worker thread!!! #####", __func__);
	return arg;
}

int fm_hci_init(fm_hal_cb *p_cb)
{
    pthread_attr_t thread_attr;
    struct sched_param param;
    int policy, result, hci_type;

    ALOGI("FM-HCI: init");

    /* Save the FM-HAL event notofication callback func. */
    hal_cb = p_cb;

    ALOGI("FM-HCI: Loading the WCNSS HAL library...");
    init_vnd_if();

    ALOGI("%s: Initializing FM-HCI layer...", __func__);
    lib_running = 1;
    ready_events = 0;
    command_credits = 1;

    pthread_mutex_init(&fmHCIControlBlock.tx_q_lock, NULL);
    pthread_mutex_init(&fmHCIControlBlock.credit_lock, NULL);
    pthread_mutex_init(&fmHCIControlBlock.event_lock, NULL);

    pthread_cond_init(&fmHCIControlBlock.event_cond, NULL);
    pthread_cond_init(&fmHCIControlBlock.cmd_credits_cond, NULL);

    pthread_attr_init(&thread_attr);

    ALOGI("FM-HCI: Creating the FM-HCI TASK...");
    if (pthread_create(&fmHCIControlBlock.fmHCITaskThreadId, &thread_attr, \
                       fmHCITask, NULL) != 0)
    {
        ALOGE("pthread_create failed!");
        lib_running = 0;
        return FM_HC_STATUS_FAIL;
    }

    ALOGI("FM-HCI: Configuring the scheduling policy and priority of the FM HCI thread");
    if(pthread_getschedparam(fmHCIControlBlock.fmHCITaskThreadId, &policy, &param)==0)
    {
        policy = SCHED_NORMAL;
#if (BTHC_LINUX_BASE_POLICY!=SCHED_NORMAL)
        param.sched_priority = BTHC_MAIN_THREAD_PRIORITY;
#endif
        result = pthread_setschedparam(fmHCIControlBlock.fmHCITaskThreadId, policy, &param);
        if (result != 0)
        {
            ALOGW("libbt-hci init: pthread_setschedparam failed (%s)", \
                  strerror(result));
        }
    } else
        ALOGI("FM-HCI: Failed to get the Scheduling parameters!!!");

    return FM_HC_STATUS_SUCCESS;
}


void fm_power(fm_power_state state)
{
    int val;

    if (state) {
        ALOGI("FM-HCI: %s: Turning FM ON", __func__);
        val = state;
        fm_vnd_if->op(FM_VND_OP_POWER_CTRL, &val);
    } else {
        ALOGI("FM-HCI: %s: Turning FM OFF", __func__);
        val = state;
        fm_vnd_if->op(FM_VND_OP_POWER_CTRL, &val);
    }
}

#define CH_MAX 3
int open_serial_port()
{
    int i, ret;
    int fd_array[CH_MAX];

    for (int i = 0; i < CH_MAX; i++)
        fd_array[i] = -1;

    ALOGI("%s: Opening the TTy Serial port...", __func__);
    ret = fm_vnd_if->op(BT_VND_OP_FM_USERIAL_OPEN, &fd_array);

    fm_fd = fd_array[0];
    if (fm_fd == -1) {
        ALOGE("%s unable to open TTY serial port", __func__);
        goto err;
    }

    //TODO: Start the userial read thread here
    ALOGE("%s: Starting the userial read thread....", __func__);
    if (pthread_create(&fmHCIControlBlock.fmRxTaskThreadId, NULL, \
                       userial_read_thread, NULL) != 0)
    {
        ALOGE("pthread_create failed!");
        lib_running = 0;
        return FM_HC_STATUS_FAIL;
    }

    return 0;

err:
    ALOGI("%s: Closing the TTy Serial port due to error!!!", __func__);
    ret = fm_vnd_if->op(BT_VND_OP_FM_USERIAL_CLOSE, NULL);
    return -1;
}

void enqueue_fm_tx_cmd(FM_HDR *pbuf)
{

	pthread_mutex_lock(&fmHCIControlBlock.tx_q_lock);

	if (!fmHCIControlBlock.first) {
		fmHCIControlBlock.first = (TX_Q *) malloc(sizeof(TX_Q));
		if (!fmHCIControlBlock.first) {
			printf("Failed to allocate memory for first!!\n");
			pthread_mutex_unlock(&fmHCIControlBlock.tx_q_lock);
			return;
		}
		fmHCIControlBlock.first->hdr = pbuf;
		fmHCIControlBlock.first->next = NULL;
		fmHCIControlBlock.last = fmHCIControlBlock.first;
                ALOGI("%s: FM-CMD ENQUEUED SUCCESSFULLY", __func__);
	} else {
		TX_Q *element =  (TX_Q *) malloc(sizeof(TX_Q));
		if (!element) {
			printf("Failed to allocate memory for element!!\n");
			pthread_mutex_unlock(&fmHCIControlBlock.tx_q_lock);
			return;
		}
		fmHCIControlBlock.last->next = element;
		element->hdr = pbuf;
		element->next = NULL;
		fmHCIControlBlock.last = element;
                ALOGI("%s: fm-cmd enqueued successfully", __func__);
	}

	pthread_mutex_unlock(&fmHCIControlBlock.tx_q_lock);
}

/** Transmit frame */
void transmit(FM_HDR *pbuf)
{
    enqueue_fm_tx_cmd(pbuf);
    event_notification(HC_EVENT_TX);
}

void userial_close_reader(void) {
    // Join the reader thread if it is still running.
    if (lib_running) {
   //     send_event(USERIAL_RX_EXIT);
        int result = pthread_join(&fmHCIControlBlock.fmRxTaskThreadId, NULL);
        if (result)
            ALOGE("%s failed to join reader thread: %d", __func__, result);
        return;
    }
    ALOGW("%s Already closed userial reader thread", __func__);
}

void fm_userial_close(void) {
   if (lib_running) {
       int result = pthread_join(&fmHCIControlBlock.fmRxTaskThreadId, NULL);
       if (result)
           ALOGE("%s failed to join reader thread: %d", __func__, result);
   }
   fm_vnd_if->op(BT_VND_OP_FM_USERIAL_CLOSE, NULL);
   // Free all buffers still waiting in the RX queue.
   //  TODO: use list data structure and clean this up.
   fm_fd = -1;
}
