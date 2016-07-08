/*
 * Copyright (c) 2016, The Linux Foundation. All rights reserved.
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

/*
 * FMhal service used to access RFKILL

**/

#include <cutils/log.h>
#include <sys/socket.h>
#include <cutils/sockets.h>
#include <pthread.h>
#include <sys/select.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <stdlib.h>
#include <termios.h>
#include <fcntl.h>
#include <sys/un.h>
#include <sys/eventfd.h>
#include <errno.h>
#include <string.h>

#include <cutils/properties.h>
#include "private/android_filesystem_config.h"


//#include "bt_hci_bdroid.h"
#include "bt_vendor_lib.h"
#include "fm_hci.h"
#include "wcnss_fmhci.h"
#include <dlfcn.h>


#define LOG_TAG "FMHalService"

#define FM_HAL_SOCK "fmhal_sock"


#define BT_SSR_TRIGGERED 0xee

#define FM_POWER_OFF      0x01
#define FM_POWER_ON       0x02
#define FM_USERIAL_OPEN   0x03
#define FM_USERIAL_CLOSE  0x04

#define FM_CMD_PACKET_TYPE     0x11
#define FM_EVT_PACKET_TYPE     0x14

#ifndef BLUETOOTH_UID
#define BLUETOOTH_UID 1002
#endif
#ifndef SYSTEM_UID
#define SYSTEM_UID 1000
#endif

#ifndef ROOT_UID
#define ROOT_UID 0
#endif

pthread_mutex_t signal_mutex;

bt_vendor_interface_t *fm_if = NULL;
int remote_fm_hal_fd;

int do_write(int fd, unsigned char *buf,int len);

unsigned char reset_cmpl[] = {0x04, 0x0e, 0x04, 0x01,0x03, 0x0c, 0x00};

static int extract_uid(int uuid)
{
    int userid;
    int appid;

    appid = userid =  uuid % AID_USER;
    if (userid > BLUETOOTH_UID)
    {
        appid = userid % AID_APP;
    }
    ALOGD("%s appid = %d",__func__,appid);
    return appid;
}
 void service_cleanup()
{
    char ref_count[PROPERTY_VALUE_MAX];
    char cleanup[PROPERTY_VALUE_MAX];
    int ref_val,clean;

    ALOGE("Service is stopped ");
    property_get("wc_transport.clean_up", cleanup, "0");
    property_set("wc_transport.fm_service_status", "0");
    property_set("wc_transport.start_fmhci", "0");
    property_set("wc_transport.fm_power_status", "0");
    clean = atoi(cleanup);
    ALOGE("clean Value =  %d",clean);
}

static int establish_fm_remote_socket(char *name)
{
    int fd = -1;
    struct sockaddr_un client_address;
    socklen_t clen;
    int sock_id, ret;
    struct ucred creds;
    int c_uid;
    ALOGI("%s(%s) Entry  ", __func__, name);

    sock_id = socket(AF_LOCAL, SOCK_STREAM, 0);
    if (sock_id < 0) {
        ALOGE("%s: server Socket creation failure", __func__);
        return fd;
    }

    ALOGI("convert name to android abstract name:%s %d", name, sock_id);
    if (socket_local_server_bind(sock_id,
        name, ANDROID_SOCKET_NAMESPACE_ABSTRACT) >= 0) {
        if (listen(sock_id, 5) == 0) {
            ALOGI("listen to local socket:%s, fd:%d", name, sock_id);
        } else {
            ALOGE("listen to local socket:failed");
            close(sock_id);
            return fd;
        }
    } else {
        close(sock_id);
        ALOGE("%s: server bind failed for socket : %s", __func__, name);
        return fd;
    }

    clen = sizeof(client_address);
    /*Indicate that, server is ready to accept*/
    property_set("wc_transport.fm_service_status", "1");
    ALOGI("%s: wc_transport.fm_service_status set to 1 ", __func__);
    ALOGI("%s: before accept_server_socket", name);
    fd = accept(sock_id, (struct sockaddr *)&client_address, &clen);
    if (fd > 0) {
        ALOGI("%s accepted fd:%d for server fd:%d", name, fd, sock_id);
        close(sock_id);

        memset(&creds, 0, sizeof(creds));
        socklen_t szCreds = sizeof(creds);
        ret = getsockopt(fd, SOL_SOCKET, SO_PEERCRED, &creds, &szCreds);
        if (ret < 0) {
            ALOGE("%s: error getting remote socket creds: %d\n", __func__, ret);
            close(fd);
            return -1;
        }
        c_uid = creds.uid;
        if (c_uid > BLUETOOTH_UID)
            c_uid = extract_uid(creds.uid);
        if (c_uid != BLUETOOTH_UID && c_uid != SYSTEM_UID
                && c_uid != ROOT_UID) {
            ALOGE("%s: client doesn't have required credentials", __func__);
            ALOGE("<%s req> client uid: %d", name, creds.uid);
            close(fd);
            return -1;
        }

        ALOGI("%s: Remote socket credentials: %d\n", __func__, creds.uid);
        return fd;
    } else {
        ALOGE("BTC accept failed fd:%d sock d:%d error %s", fd, sock_id, strerror(errno));
        close(sock_id);
        return fd;
    }

    close(sock_id);
    return fd;
}


int handle_fmcommand_writes(int fd) {
    ALOGI("%s: ", __func__);
    unsigned char first_byte;
    int retval,val;
	int fd_array[CH_MAX];

    ALOGE("%s: FMHAL: read 1st byte to determine the power on/off ", __func__);

    retval = read (fd, &first_byte, 1);
    if (retval < 0) {
        ALOGE("%s:read returns err: %d\n", __func__,retval);
        return -1;
    }
    if (retval == 0) {
        ALOGE("%s: This indicates the close of other end", __func__);
        return -99;
    }

    ALOGE("%s: FM command type: 0x%x", __func__, first_byte);
    switch(first_byte) {
        case FM_POWER_OFF:
             ALOGE("%s: Received power off command from FM stack: %d", __func__, first_byte);
             val = 0;
             retval = fm_if->op(BT_VND_OP_POWER_CTRL, &val);
             if (retval < 0)
             {
                ALOGE("Failed to turn off power from  bt vendor interface");
               //   return -1;
             }
             else {
                 property_set("wc_transport.fm_power_status", "0");
                 retval = -99;
             }
             break;

        case FM_POWER_ON:
             ALOGE("%s: Received power ON command from FM stack: %d", __func__, first_byte);
             val = 1;
             retval =fm_if->op(FM_VND_OP_POWER_CTRL, &val);
             if (retval < 0)
             {
               ALOGE("Failed to turn on power from  bt vendor interface");
             }
             else
                property_set("wc_transport.fm_power_status", "1");
             break;
        default:
            ALOGE("%s: Unexpected data format!!",__func__);
            retval = -1;
    }
    return retval;
}


int do_read(int fd, unsigned char* buf, size_t len) {
   int bytes_left, bytes_read = 0, read_offset;

   bytes_left = len;
   read_offset = 0;

   do {
       bytes_read = read(fd, buf+read_offset, bytes_left);
       if (bytes_read < 0) {
           ALOGE("%s: Read error: %d (%s)", __func__, bytes_left, strerror(errno));
           return -1;
       } else if (bytes_read == 0) {
            ALOGE("%s: read returned 0, err = %s, read bytes: %d, expected: %d",
                              __func__, strerror(errno), (len-bytes_left), len);
            return (len-bytes_left);
       }
       else {
           if (bytes_read < bytes_left) {
              ALOGV("Still there are %d bytes to read", bytes_left-bytes_read);
              bytes_left = bytes_left-bytes_read;
              read_offset = read_offset+bytes_read;
           } else {
              ALOGV("%s: done with read",__func__);
              break;
           }
       }
   }while(1);
   return len;
}

int do_write(int fd, unsigned char *buf,int len)
{
    int ret = 0;
    int write_offset = 0;
    int write_len = len;
    do {
        ret = write(fd, buf+write_offset, write_len);
        if (ret < 0)
        {
            ALOGE("%s: write failed ret = %d err = %s",__func__,ret,strerror(errno));
            return -1;

        } else if (ret == 0) {
            ALOGE("%s: Write returned 0, err = %s, Written bytes: %d, expected: %d",
                        __func__, strerror(errno), (len-write_len), len);
            return (len-write_len);

        } else {
            if (ret < write_len)
            {
               ALOGD("%s, Write pending,do write ret = %d err = %s",__func__,ret,
                       strerror(errno));
               write_len = write_len - ret;
               write_offset = ret;
            } else if (ret > write_len) {
               ALOGE("%s: FATAL wrote more than expected: written bytes: %d expected: %d",
                      __func__, write_len, ret);
               break;
            } else {
               ALOGV("Write successful");
               break;
            }
        }
    } while(1);
    return len;
}

void vnd_load_if()
{
    void *dlhandle;
    unsigned char bdaddr[] = {0xaa, 0xbb, 0xcc, 0x11, 0x22, 0x33};

    dlhandle = dlopen("libbt-vendor.so", RTLD_NOW);
    if (!dlhandle)
    {
        ALOGE("!!! Failed to load libbt-vendor.so !!!");
        return;
    }

    fm_if = (bt_vendor_interface_t *) dlsym(dlhandle, "BLUETOOTH_VENDOR_LIB_INTERFACE");
    if (!fm_if)
    {
        ALOGE("!!! Failed to get bt vendor interface !!!");
        return;
    }

    ALOGI("FM-HCI: Registering the WCNSS HAL library by passing CBs and BD addr.");
    fm_if->init(&fmhci_vendor_callbacks, bdaddr);
}


int main()  {
   fd_set client_fds;
    int retval = -1, n;

    ALOGI("%s: Entry ", __func__);
    ALOGI("FM HAL SERVICE: Loading the WCNSS HAL library...");
    vnd_load_if();
    ALOGI("create socket");
    remote_fm_hal_fd = establish_fm_remote_socket(FM_HAL_SOCK);
    if (remote_fm_hal_fd < 0) {
       ALOGE("%s: invalid remote socket", __func__);
       return -1;
    }

     FD_ZERO(&client_fds);
     FD_SET(remote_fm_hal_fd, &client_fds);

     do {
            ALOGI("%s: Step 1-FM-HAL SERVICE: Waiting for FM HAL cmd ", __func__);
            n = select(remote_fm_hal_fd+1, &client_fds, NULL, NULL, NULL);
            if(n < 0){
                ALOGE("Select: failed: %s", strerror(errno));
                break;
            }
            ALOGI("%s: Step 2-FM-HAL SERVICE: FM  POWER CMD available for processing...\n", __func__);
            if (FD_ISSET(remote_fm_hal_fd, &client_fds)) {
                retval = handle_fmcommand_writes(remote_fm_hal_fd);
                ALOGI("%s: handle_fmcommand_writes . %d", __func__, retval);
               if(retval < 0) {
                   if (retval == -99) {
                       ALOGI("%s:End of wait loop", __func__);
                       break;
                   }
                   ALOGI("%s: handle_fmcommand_writes returns: %d: ", __func__, retval);
                 //  break;
                }
            }
        } while(1);

        service_cleanup();
        ALOGI("%s: FM turned off or power off failed .service kill itself", __func__);
        close(remote_fm_hal_fd);
       remote_fm_hal_fd = 0;

    ALOGI("%s: Exit: %d", __func__, retval);
    return retval;
}

