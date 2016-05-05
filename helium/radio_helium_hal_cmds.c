/*
Copyright (c) 2015, The Linux Foundation. All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are
met:
    * Redistributions of source code must retain the above copyright
      notice, this list of conditions and the following disclaimer.
    * Redistributions in binary form must reproduce the above
      copyright notice, this list of conditions and the following
      disclaimer in the documentation and/or other materials provided
      with the distribution.
    * Neither the name of The Linux Foundation nor the names of its
      contributors may be used to endorse or promote products derived
      from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED "AS IS" AND ANY EXPRESS OR IMPLIED
WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NON-INFRINGEMENT
ARE DISCLAIMED.  IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS
BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR
BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE
OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN
IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

#include <stdio.h>
#include <utils/Log.h>
#include "radio-helium-commands.h"
#include "radio-helium.h"
#include "fm_hci.h"
#include <dlfcn.h>
#define LOG_TAG "radio_helium"

static int send_fm_cmd_pkt(uint16_t opcode,  uint32_t len, void *param)
{
    int p_len = 4 + len;
    int ret = 0;

//    pthread_mutex_lock(&fm_hal);
    FM_HDR *hdr = (FM_HDR *) malloc(p_len);
    if (!hdr) {
        ALOGE("%s:hdr allocation failed", LOG_TAG);
        return -1;
    }

    ALOGE("%s:%s: Sizeof FM_HDR: %d", LOG_TAG, __func__, sizeof(FM_HDR));
    ALOGE("%s:opcode: %x", LOG_TAG, opcode);

    hdr->protocol_byte = 0x11;
    hdr->opcode = opcode;
    hdr->plen = len;
    if (len)
        memcpy(hdr->cmd_params, (uint8_t *)param, len);
    ALOGE("%s:calling transmit", __func__);
    transmit(hdr);
    ALOGE("%s:transmit success",__func__);
    return 0;
}

int hci_fm_get_signal_threshold()
{

    FM_HDR *hdr = (FM_HDR *) malloc(sizeof(FM_HDR));
    hdr->protocol_byte = FM_CMD;
    hdr->opcode = hci_opcode_pack(HCI_OGF_FM_RECV_CTRL_CMD_REQ, HCI_OCF_FM_GET_SIGNAL_THRESHOLD);
    hdr->plen   = 0;
    transmit(hdr);
    return 0;
}

int hci_fm_enable_recv_req()
{
    uint16_t opcode = 0;

    opcode = hci_opcode_pack(HCI_OGF_FM_RECV_CTRL_CMD_REQ,
                                  HCI_OCF_FM_ENABLE_RECV_REQ);
    return send_fm_cmd_pkt(opcode, 0, NULL);
}

int hci_fm_disable_recv_req()
{
  uint16_t opcode = 0;

  opcode = hci_opcode_pack(HCI_OGF_FM_RECV_CTRL_CMD_REQ,
                                 HCI_OCF_FM_DISABLE_RECV_REQ);
  return send_fm_cmd_pkt(opcode, 0, NULL);
}

int  hci_fm_mute_mode_req(struct hci_fm_mute_mode_req *mute)
{
    uint16_t opcode = 0;
    int len = 0;
    opcode = hci_opcode_pack(HCI_OGF_FM_RECV_CTRL_CMD_REQ,
                               HCI_OCF_FM_SET_MUTE_MODE_REQ);
    len = sizeof(struct hci_fm_mute_mode_req);
    return send_fm_cmd_pkt(opcode, len, mute);
}

int helium_search_list(struct hci_fm_search_station_list_req *s_list)
{
   uint16_t opcode = 0;

   if (s_list == NULL) {
       ALOGE("%s:%s, search list param is null\n", LOG_TAG, __func__);
       return -1;
   }
   opcode = hci_opcode_pack(HCI_OGF_FM_RECV_CTRL_CMD_REQ,
                HCI_OCF_FM_SEARCH_STATIONS_LIST);
   return send_fm_cmd_pkt(opcode, sizeof((*s_list)), s_list);
}

int helium_search_rds_stations(struct hci_fm_search_rds_station_req *rds_srch)
{
   uint16_t opcode = 0;

   if (rds_srch == NULL) {
       ALOGE("%s:%s, rds stations param is null\n", LOG_TAG, __func__);
       return -1;
   }
   opcode = hci_opcode_pack(HCI_OGF_FM_RECV_CTRL_CMD_REQ,
                HCI_OCF_FM_SEARCH_RDS_STATIONS);
   return send_fm_cmd_pkt(opcode, sizeof((*rds_srch)), rds_srch);
}

int helium_search_stations(struct hci_fm_search_station_req *srch)
{
   uint16_t opcode = 0;

   if (srch == NULL) {
       ALOGE("%s:%s, search station param is null\n", LOG_TAG, __func__);
       return -1;
   }
   opcode = hci_opcode_pack(HCI_OGF_FM_RECV_CTRL_CMD_REQ,
                HCI_OCF_FM_SEARCH_STATIONS);
   return send_fm_cmd_pkt(opcode, sizeof((*srch)), srch);
}

int helium_cancel_search_req()
{
   uint16_t opcode = 0;

   opcode = hci_opcode_pack(HCI_OGF_FM_RECV_CTRL_CMD_REQ,
                HCI_OCF_FM_CANCEL_SEARCH);
   return send_fm_cmd_pkt(opcode, 0, NULL);
}

int hci_fm_set_recv_conf_req (struct hci_fm_recv_conf_req *conf)
{
    uint16_t opcode = 0;

    if (conf == NULL) {
        ALOGE("%s:%s, recv conf is null\n", LOG_TAG, __func__);
        return -1;
    }
    opcode = hci_opcode_pack(HCI_OGF_FM_RECV_CTRL_CMD_REQ,
                              HCI_OCF_FM_SET_RECV_CONF_REQ);
    return send_fm_cmd_pkt(opcode, sizeof((*conf)), conf);
}

int helium_set_sig_threshold_req(char th)
{
    uint16_t opcode = 0;

    if (th == NULL) {
        ALOGE("%s:Threshold value NULL", LOG_TAG);
        return -1;
    }
    opcode = hci_opcode_pack(HCI_OGF_FM_RECV_CTRL_CMD_REQ,
                               HCI_OCF_FM_SET_SIGNAL_THRESHOLD);
    return send_fm_cmd_pkt(opcode, sizeof(th), th);
}

int helium_rds_grp_mask_req(struct hci_fm_rds_grp_req *rds_grp_msk)
{
    uint16_t opcode = 0;

    if (rds_grp_msk == NULL) {
        ALOGE("%s:%s, grp mask param is null\n", LOG_TAG, __func__);
        return -EINVAL;
    }
    opcode = hci_opcode_pack(HCI_OGF_FM_RECV_CTRL_CMD_REQ,
                                    HCI_OCF_FM_RDS_GRP);
    return send_fm_cmd_pkt(opcode, sizeof(*rds_grp_msk), rds_grp_msk);
}

int helium_rds_grp_process_req(int rds_grp)
{
    uint16_t opcode = 0;

    opcode = hci_opcode_pack(HCI_OGF_FM_RECV_CTRL_CMD_REQ,
                              HCI_OCF_FM_RDS_GRP_PROCESS);
    return send_fm_cmd_pkt(opcode, sizeof(rds_grp), &rds_grp);
}

int helium_set_event_mask_req(char e_mask)
{
    uint16_t opcode = 0;

    opcode = hci_opcode_pack(HCI_OGF_FM_RECV_CTRL_CMD_REQ,
                          HCI_OCF_FM_SET_EVENT_MASK);
    return send_fm_cmd_pkt(opcode, sizeof(e_mask), &e_mask);
}

int helium_set_antenna_req(char ant)
{
    uint16_t opcode = 0;

    opcode = hci_opcode_pack(HCI_OGF_FM_RECV_CTRL_CMD_REQ,
                                   HCI_OCF_FM_SET_ANTENNA);
    return send_fm_cmd_pkt(opcode, sizeof(ant), &ant);
}

int helium_set_fm_mute_mode_req(struct hci_fm_mute_mode_req *mute)
{
    uint16_t opcode = 0;

    if (mute == NULL) {
        ALOGE("%s:%s, mute mode is null\n", LOG_TAG, __func__);
        return -1;
    }
    opcode = hci_opcode_pack(HCI_OGF_FM_RECV_CTRL_CMD_REQ,
                               HCI_OCF_FM_SET_MUTE_MODE_REQ);
    return send_fm_cmd_pkt(opcode, sizeof((*mute)), mute);
}

int hci_fm_tune_station_req(int param)
{
    uint16_t opcode = 0;
    int tune_freq = param;

    ALOGE("%s:tune_freq: %d", LOG_TAG, tune_freq);
    opcode = hci_opcode_pack(HCI_OGF_FM_COMMON_CTRL_CMD_REQ,
                                  HCI_OCF_FM_TUNE_STATION_REQ);
    return send_fm_cmd_pkt(opcode, sizeof(tune_freq), &tune_freq);
}

int hci_set_fm_stereo_mode_req(struct hci_fm_stereo_mode_req *param)
{
    uint16_t opcode = 0;
    struct hci_fm_stereo_mode_req *stereo_mode_req =
                           (struct hci_fm_stereo_mode_req *) param;

    if (stereo_mode_req == NULL) {
        ALOGE("%s:%s, stere mode req is null\n", LOG_TAG, __func__);
        return -1;
    }
    opcode = hci_opcode_pack(HCI_OGF_FM_RECV_CTRL_CMD_REQ,
                             HCI_OCF_FM_SET_STEREO_MODE_REQ);
    return send_fm_cmd_pkt(opcode, sizeof((*stereo_mode_req)),
                                              stereo_mode_req);
}

