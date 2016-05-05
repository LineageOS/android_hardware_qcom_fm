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
#include <unistd.h>
#include <utils/Log.h>
#include "radio-helium-commands.h"
#include "radio-helium.h"
#include "fm_hci.h"
#include <dlfcn.h>

fm_vendor_callbacks_t *jni_cb;
int hci_fm_get_signal_threshold();
int hci_fm_enable_recv_req();
struct helium_device *radio;
static int oda_agt;
static int grp_mask;
static int rt_plus_carrier = -1;
static int ert_carrier = -1;
static unsigned char ert_buf[256];
static unsigned char ert_len;
static unsigned char c_byt_pair_index;
static char utf_8_flag;
static char rt_ert_flag;
static char formatting_dir;

#define LOG_TAG "radio_helium"
static void radio_hci_req_complete(char result)
{
  ALOGD("%s:enetred %s", LOG_TAG, __func__);
}

static void radio_hci_status_complete(int result)
{
   ALOGD("%s:enetred %s", LOG_TAG, __func__);
}

static void hci_cc_fm_enable_rsp(char *ev_rsp)
{
    struct hci_fm_conf_rsp  *rsp;

    if (ev_rsp == NULL) {
        ALOGE("%s:%s, buffer is null\n", LOG_TAG, __func__);
        return;
    }
    rsp = (struct hci_fm_conf_rsp *)ev_rsp;
    jni_cb->thread_evt_cb(0);
    radio_hci_req_complete(rsp->status);
    jni_cb->enabled_cb();
}

static void hci_cc_conf_rsp(char *ev_rsp)
{
    struct hci_fm_conf_rsp  *rsp;

    if (ev_rsp == NULL) {
        ALOGE("%s:%s, buffer is null\n", LOG_TAG, __func__);
        return;
    }
    rsp = (struct hci_fm_conf_rsp *)ev_rsp;
    radio_hci_req_complete(rsp->status);
    if (!rsp->status) {
        radio->recv_conf = rsp->recv_conf_rsp;
    }
}

static void hci_cc_fm_disable_rsp(char *ev_buff)
{
    char status;

    if (ev_buff == NULL) {
        ALOGE("%s:%s, buffer is null\n", LOG_TAG, __func__);
        return;
    }
    status = (char) *ev_buff;
    radio_hci_req_complete(status);
    if (radio->mode == FM_TURNING_OFF) {
        jni_cb->disabled_cb();
        radio->mode = FM_OFF;
    }
}

static void hci_cc_rsp(char *ev_buff)
{
    char status;

    if (ev_buff == NULL) {
        ALOGE("%s:%s, socket buffer is null\n", LOG_TAG, __func__);
        return;
    }
    status = (char)*ev_buff;

    radio_hci_req_complete(status);
}

static inline void hci_cmd_complete_event(char *buff)
{
    uint16_t opcode;
    uint8_t *pbuf;

    if (buff == NULL) {
        ALOGE("%s:%s, buffer is null\n", LOG_TAG, __func__);
        return;
    }
    ALOGE("%s:buff[1] = 0x%x buff[2] = 0x%x", LOG_TAG, buff[1], buff[2]);
    opcode = ((buff[2] << 8) | buff[1]);
    ALOGE("%s: Received HCI CMD COMPLETE EVENT for opcode: 0x%x", __func__, opcode);
    pbuf = &buff[3];

    switch (opcode) {
    case hci_recv_ctrl_cmd_op_pack(HCI_OCF_FM_ENABLE_RECV_REQ):
            ALOGE("%s: Recvd. CC event for FM_ENABLE_RECV_REQ", __func__);
            hci_cc_fm_enable_rsp(pbuf);
            break;
    case hci_recv_ctrl_cmd_op_pack(HCI_OCF_FM_GET_RECV_CONF_REQ):
            hci_cc_conf_rsp(pbuf);
            break;
    case hci_recv_ctrl_cmd_op_pack(HCI_OCF_FM_DISABLE_RECV_REQ):
            hci_cc_fm_disable_rsp(pbuf);
            break;

    case hci_recv_ctrl_cmd_op_pack(HCI_OCF_FM_SET_RECV_CONF_REQ):
    case hci_recv_ctrl_cmd_op_pack(HCI_OCF_FM_SET_MUTE_MODE_REQ):
    case hci_recv_ctrl_cmd_op_pack(HCI_OCF_FM_SET_STEREO_MODE_REQ):
    case hci_recv_ctrl_cmd_op_pack(HCI_OCF_FM_SET_ANTENNA):
    case hci_recv_ctrl_cmd_op_pack(HCI_OCF_FM_SET_SIGNAL_THRESHOLD):
    case hci_recv_ctrl_cmd_op_pack(HCI_OCF_FM_CANCEL_SEARCH):
    case hci_recv_ctrl_cmd_op_pack(HCI_OCF_FM_RDS_GRP):
    case hci_recv_ctrl_cmd_op_pack(HCI_OCF_FM_RDS_GRP_PROCESS):
    case hci_recv_ctrl_cmd_op_pack(HCI_OCF_FM_EN_WAN_AVD_CTRL):
    case hci_recv_ctrl_cmd_op_pack(HCI_OCF_FM_EN_NOTCH_CTRL):
    case hci_recv_ctrl_cmd_op_pack(HCI_OCF_FM_SET_CH_DET_THRESHOLD):
    case hci_recv_ctrl_cmd_op_pack(HCI_OCF_FM_SET_BLND_TBL):
    case hci_common_cmd_op_pack(HCI_OCF_FM_DEFAULT_DATA_WRITE):
            hci_cc_rsp(pbuf);
            break;
    case hci_common_cmd_op_pack(HCI_OCF_FM_RESET):
    case hci_diagnostic_cmd_op_pack(HCI_OCF_FM_SSBI_POKE_REG):
    case hci_diagnostic_cmd_op_pack(HCI_OCF_FM_POKE_DATA):
    case hci_diagnostic_cmd_op_pack(HCI_FM_SET_INTERNAL_TONE_GENRATOR):
    case hci_common_cmd_op_pack(HCI_OCF_FM_SET_CALIBRATION):
    case hci_recv_ctrl_cmd_op_pack(HCI_OCF_FM_SET_EVENT_MASK):
    case hci_common_cmd_op_pack(HCI_OCF_FM_SET_SPUR_TABLE):
            hci_cc_rsp(pbuf);
            break;
/*    case hci_common_cmd_op_pack(HCI_OCF_FM_GET_SPUR_TABLE):
            hci_cc_get_spur_tbl(buff);
            break;
    case hci_diagnostic_cmd_op_pack(HCI_OCF_FM_SSBI_PEEK_REG):
            hci_cc_ssbi_peek_rsp(buff);
            break;
    case hci_recv_ctrl_cmd_op_pack(HCI_OCF_FM_GET_SIGNAL_THRESHOLD):
            hci_cc_sig_threshold_rsp(buff);
            break;

    case hci_recv_ctrl_cmd_op_pack(HCI_OCF_FM_GET_STATION_PARAM_REQ):
            hci_cc_station_rsp(buff);
            break;

    case hci_recv_ctrl_cmd_op_pack(HCI_OCF_FM_GET_PROGRAM_SERVICE_REQ):
            hci_cc_prg_srv_rsp(buff);
            break;

    case hci_recv_ctrl_cmd_op_pack(HCI_OCF_FM_GET_RADIO_TEXT_REQ):
            hci_cc_rd_txt_rsp(buff);
            break;

    case hci_recv_ctrl_cmd_op_pack(HCI_OCF_FM_GET_AF_LIST_REQ):
            hci_cc_af_list_rsp(buff);
            break;

    case hci_common_cmd_op_pack(HCI_OCF_FM_DEFAULT_DATA_READ):
            hci_cc_riva_read_default_rsp(buff);
            break;

    case hci_diagnostic_cmd_op_pack(HCI_OCF_FM_PEEK_DATA):
            hci_cc_riva_peek_rsp(buff);
            break;

    case hci_common_cmd_op_pack(HCI_OCF_FM_GET_FEATURE_LIST):
            hci_cc_feature_list_rsp(buff);
            break;

    case hci_diagnostic_cmd_op_pack(HCI_OCF_FM_STATION_DBG_PARAM):
            hci_cc_dbg_param_rsp(buff);
            break;
    case hci_status_param_op_pack(HCI_OCF_FM_READ_GRP_COUNTERS):
            hci_cc_rds_grp_cntrs_rsp(buff);
            break;
    case hci_common_cmd_op_pack(HCI_OCF_FM_DO_CALIBRATION):
            hci_cc_do_calibration_rsp(buff);
            break;

    case hci_recv_ctrl_cmd_op_pack(HCI_OCF_FM_GET_CH_DET_THRESHOLD):
            hci_cc_get_ch_det_threshold_rsp(buff);
            break;
    case hci_recv_ctrl_cmd_op_pack(HCI_OCF_FM_GET_BLND_TBL):
            hci_cc_get_blend_tbl_rsp(buff);
            break;
    default:
            ALOGE("opcode 0x%x", opcode);
            break; */
    }
}

static inline void hci_cmd_status_event(char *st_rsp)
{
    struct hci_ev_cmd_status *ev = (void *) st_rsp;
    uint16_t opcode;

    if (st_rsp == NULL) {
        ALOGE("%s:%s, buffer is null\n", LOG_TAG, __func__);
        return;
    }
    ALOGE("%s:st_rsp[2] = 0x%x st_rsp[3] = 0x%x", LOG_TAG, st_rsp[2], st_rsp[3]);
    opcode = ((st_rsp[3] << 8) | st_rsp[2]);
    ALOGE("%s: Received HCI CMD STATUS EVENT for opcode: 0x%x", __func__, opcode);

    radio_hci_status_complete(ev->status);
}

static inline void hci_ev_tune_status(char *buff)
{

    memcpy(&radio->fm_st_rsp.station_rsp, &buff[0],
                               sizeof(struct hci_ev_tune_status));
    jni_cb->tune_cb(radio->fm_st_rsp.station_rsp.station_freq);

    //    if (radio->fm_st_rsp.station_rsp.serv_avble)
          // todo callback for threshould

    if (radio->fm_st_rsp.station_rsp.stereo_prg)
        jni_cb->stereo_status_cb(true);
    else if (radio->fm_st_rsp.station_rsp.stereo_prg == 0)
        jni_cb->stereo_status_cb(false);

    if (radio->fm_st_rsp.station_rsp.rds_sync_status)
        jni_cb->rds_avail_status_cb(true);
    else
        jni_cb->rds_avail_status_cb(false);
}

static inline void hci_ev_search_next(char *buff)
{
    jni_cb->scan_next_cb();
}

static inline void hci_ev_stereo_status(char *buff)
{
    char st_status;

    if (buff == NULL) {
        ALOGE("%s:%s, socket buffer is null\n", LOG_TAG,__func__);
        return;
    }
    st_status =  buff[0];
    if (st_status)
        jni_cb->stereo_status_cb(true);
    else
        jni_cb->stereo_status_cb(false);
}

static void hci_ev_rds_lock_status(char *buff)
{
    char rds_status;

    if (buff == NULL) {
        ALOGE("%s:%s, socket buffer is null\n", LOG_TAG, __func__);
        return;
    }

    rds_status = buff[0];

    if (rds_status)
        jni_cb->rds_avail_status_cb(true);
    else
        jni_cb->rds_avail_status_cb(false);
}

static inline void hci_ev_program_service(char *buff)
{
    int len;
    char *data;

    len = (buff[RDS_PS_LENGTH_OFFSET] * RDS_STRING) + RDS_OFFSET;
    data = malloc(len);
    if (!data) {
        ALOGE("%s:Failed to allocate memory", LOG_TAG);
        return;
    }

    data[0] = buff[RDS_PS_LENGTH_OFFSET];
    data[1] = buff[RDS_PTYPE];
    data[2] = buff[RDS_PID_LOWER];
    data[3] = buff[RDS_PID_HIGHER];
    data[4] = 0;

    memcpy(data+RDS_OFFSET, &buff[RDS_PS_DATA_OFFSET], len-RDS_OFFSET);

    ALOGE("SSK call ps-callback");
    jni_cb->ps_update_cb(data);

    free(data);
}

static inline void hci_ev_radio_text(char *buff)
{
    int len = 0;
    char *data;

    if (buff == NULL) {
        ALOGE("%s:%s, buffer is null\n", LOG_TAG,__func__);
        return;
    }

    while ((buff[len+RDS_OFFSET] != 0x0d) && (len < MAX_RT_LENGTH))
           len++;
    data = malloc(len+RDS_OFFSET);
    if (!data) {
        ALOGE("%s:Failed to allocate memory", LOG_TAG);
        return;
    }

    data[0] = len;
    data[1] = buff[RDS_PTYPE];
    data[2] = buff[RDS_PID_LOWER];
    data[3] = buff[RDS_PID_HIGHER];
    data[4] = buff[RT_A_B_FLAG_OFFSET];

    memcpy(data+RDS_OFFSET, &buff[RDS_OFFSET], len);
    data[len+RDS_OFFSET] = 0x00;

    jni_cb->rt_update_cb(data);
    free(data);
}

static void hci_ev_af_list(char *buff)
{
    struct hci_ev_af_list ev;

    if (buff == NULL) {
        ALOGE("%s:%s, buffer is null\n", LOG_TAG, __func__);
        return;
    }
    ev.tune_freq = *((int *) &buff[0]);
    ev.pi_code = *((__le16 *) &buff[PI_CODE_OFFSET]);
    ev.af_size = buff[AF_SIZE_OFFSET];
    if (ev.af_size > AF_LIST_MAX) {
        ALOGE("%s:AF list size received more than available size", LOG_TAG);
        return;
    }
    memcpy(&ev.af_list[0], &buff[AF_LIST_OFFSET],
                                        ev.af_size * sizeof(int));
    jni_cb->af_list_update_cb(&ev);
}

static inline void hci_ev_search_compl(char *buff)
{
    if (buff == NULL) {
        ALOGE("%s:%s,buffer is null\n", LOG_TAG, __func__);
        return;
    }
    radio->search_on = 0;
    jni_cb->seek_cmpl_cb(radio->fm_st_rsp.station_rsp.station_freq);
}

static inline void hci_ev_srch_st_list_compl(char *buff)
{
    struct hci_ev_srch_list_compl *ev ;
    int cnt;
    int stn_num;
    int rel_freq;
    int abs_freq;
    int len;

    if (buff == NULL) {
        ALOGE("%s:%s, buffer is null\n", LOG_TAG,__func__);
        return;
    }
    ev = malloc(sizeof(*ev));
    if (!ev) {
        ALOGE("%s:Memory allocation failed", LOG_TAG);
        return ;
    }

    ev->num_stations_found = buff[STN_NUM_OFFSET];
    len = ev->num_stations_found * PARAMS_PER_STATION + STN_FREQ_OFFSET;

    for(cnt = STN_FREQ_OFFSET, stn_num = 0;
           (cnt < len) && (stn_num < ev->num_stations_found)
                      && (stn_num < SIZE_ARRAY(ev->rel_freq));
            cnt += PARAMS_PER_STATION, stn_num++) {

        abs_freq = *((int *)&buff[cnt]);
        rel_freq = abs_freq - radio->recv_conf.band_low_limit;
        rel_freq = (rel_freq * 20) / KHZ_TO_MHZ;

        ev->rel_freq[stn_num].rel_freq_lsb = GET_LSB(rel_freq);
        ev->rel_freq[stn_num].rel_freq_msb = GET_MSB(rel_freq);
    }

    len = ev->num_stations_found * 2 + sizeof(ev->num_stations_found);
    jni_cb->srch_list_cb((char*)ev);
    free(ev);
}

static void hci_ev_rt_plus(struct rds_grp_data rds_buf)
{
    char tag_type1, tag_type2;
    char *data = NULL;
    int len = 0;
    unsigned short int agt;

    agt = AGT(rds_buf.rdsBlk[1].rdsLsb);
    /*right most 3 bits of Lsb of block 2
     * and left most 3 bits of Msb of block 3
     */
    tag_type1 = (((agt & TAG1_MSB_MASK) << TAG1_MSB_OFFSET) |
                 (rds_buf.rdsBlk[2].rdsMsb >> TAG1_LSB_OFFSET));

    /*right most 1 bit of lsb of 3rd block
     * and left most 5 bits of Msb of 4th block
     */
    tag_type2 = (((rds_buf.rdsBlk[2].rdsLsb & TAG2_MSB_MASK) << TAG2_MSB_OFFSET) |
                 (rds_buf.rdsBlk[3].rdsMsb >> TAG2_LSB_OFFSET));

    if (tag_type1 != DUMMY_CLASS)
        len += RT_PLUS_LEN_1_TAG;
    if (tag_type2 != DUMMY_CLASS)
        len += RT_PLUS_LEN_1_TAG;

    if (len != 0) {
        len += 2;
        data = malloc(len);
    } else {
        ALOGE("%s:Len is zero\n", LOG_TAG);
        return ;
    }
    if (data != NULL) {
        data[0] = len;
        len = 1;
        data[len++] = rt_ert_flag;
        if (tag_type1 != DUMMY_CLASS) {
            data[len++] = tag_type1;
            /*start position of tag1
             *right most 5 bits of msb of 3rd block
             *and left most bit of lsb of 3rd block
             */
            data[len++] = (((rds_buf.rdsBlk[2].rdsMsb & TAG1_POS_MSB_MASK)
                                            << TAG1_POS_MSB_OFFSET)|
                            (rds_buf.rdsBlk[2].rdsLsb >> TAG1_POS_LSB_OFFSET));
            /*length of tag1
             *left most 6 bits of lsb of 3rd block
             */
            data[len++] = ((rds_buf.rdsBlk[2].rdsLsb >> TAG1_LEN_OFFSET) &
                                                         TAG1_LEN_MASK) + 1;
        }
        if (tag_type2 != DUMMY_CLASS) {
            data[len++] = tag_type2;
            /*start position of tag2
             *right most 3 bit of msb of 4th block
             *and left most 3 bits of lsb of 4th block
             */
            data[len++] = (((rds_buf.rdsBlk[3].rdsMsb & TAG2_POS_MSB_MASK)
                                                << TAG2_POS_MSB_OFFSET) |
                           (rds_buf.rdsBlk[3].rdsLsb >> TAG2_POS_LSB_OFFSET));
            /*length of tag2
             *right most 5 bits of lsb of 4th block
             */
            data[len++] = (rds_buf.rdsBlk[3].rdsLsb & TAG2_LEN_MASK) + 1;
        }
       jni_cb->rt_plus_update_cb(data);
        free(data);
    } else {
        ALOGE("%s:memory allocation failed\n", LOG_TAG);
    }
}

static void hci_ev_ert()
{
    char *data = NULL;

    if (ert_len <= 0)
        return;
    data = malloc(ert_len + 3);
    if (data != NULL) {
        data[0] = ert_len;
        data[1] = utf_8_flag;
        data[2] = formatting_dir;
        memcpy((data + 3), ert_buf, ert_len);
        jni_cb->ert_update_cb(data);
        free(data);
    }
}

static void hci_buff_ert(struct rds_grp_data *rds_buf)
{
    int i;
    unsigned short int info_byte = 0;
    unsigned short int byte_pair_index;

    if (rds_buf == NULL) {
        ALOGE("%s:%s, rds buffer is null\n", LOG_TAG, __func__);
        return;
    }
    byte_pair_index = AGT(rds_buf->rdsBlk[1].rdsLsb);
    if (byte_pair_index == 0) {
        c_byt_pair_index = 0;
        ert_len = 0;
    }
    if (c_byt_pair_index == byte_pair_index) {
        c_byt_pair_index++;
        for (i = 2; i <= 3; i++) {
             info_byte = rds_buf->rdsBlk[i].rdsLsb;
             info_byte |= (rds_buf->rdsBlk[i].rdsMsb << 8);
             ert_buf[ert_len++] = rds_buf->rdsBlk[i].rdsMsb;
             ert_buf[ert_len++] = rds_buf->rdsBlk[i].rdsLsb;
             if ((utf_8_flag == 0) && (info_byte == CARRIAGE_RETURN)) {
                 ert_len -= 2;
                 break;
             } else if ((utf_8_flag == 1) &&
                        (rds_buf->rdsBlk[i].rdsMsb == CARRIAGE_RETURN)) {
                 info_byte = CARRIAGE_RETURN;
                 ert_len -= 2;
                 break;
             } else if ((utf_8_flag == 1) &&
                        (rds_buf->rdsBlk[i].rdsLsb == CARRIAGE_RETURN)) {
                 info_byte = CARRIAGE_RETURN;
                 ert_len--;
                 break;
             }
        }
        if ((byte_pair_index == MAX_ERT_SEGMENT) ||
            (info_byte == CARRIAGE_RETURN)) {
            hci_ev_ert();
            c_byt_pair_index = 0;
            ert_len = 0;
        }
    } else {
        ert_len = 0;
        c_byt_pair_index = 0;
    }
}

static void hci_ev_raw_rds_group_data(char *buff)
{
    unsigned char blocknum, index;
    struct rds_grp_data temp;
    unsigned int mask_bit;
    unsigned short int aid, agt, gtc;
    unsigned short int carrier;

    index = RDSGRP_DATA_OFFSET;

    if (buff == NULL) {
        ALOGE("%s:%s, socket buffer is null\n", LOG_TAG, __func__);
        return;
    }

    for (blocknum = 0; blocknum < RDS_BLOCKS_NUM; blocknum++) {
         temp.rdsBlk[blocknum].rdsLsb = buff[index];
         temp.rdsBlk[blocknum].rdsMsb = buff[index+1];
         index = index + 2;
    }

    aid = AID(temp.rdsBlk[3].rdsLsb, temp.rdsBlk[3].rdsMsb);
    gtc = GTC(temp.rdsBlk[1].rdsMsb);
    agt = AGT(temp.rdsBlk[1].rdsLsb);

    if (gtc == GRP_3A) {
        switch (aid) {
        case ERT_AID:
            /* calculate the grp mask for RDS grp
             * which will contain actual eRT text
             *
             * Bit Pos  0  1  2  3  4   5  6   7
             * Grp Type 0A 0B 1A 1B 2A  2B 3A  3B
             *
             * similary for rest grps
             */
             mask_bit = (((agt >> 1) << 1) + (agt & 1));
             oda_agt = (1 << mask_bit);
             utf_8_flag = (temp.rdsBlk[2].rdsLsb & 1);
             formatting_dir = EXTRACT_BIT(temp.rdsBlk[2].rdsLsb,
                                               ERT_FORMAT_DIR_BIT);
             if (ert_carrier != agt)
                 jni_cb->oda_update_cb();
             ert_carrier = agt;
             break;
        case RT_PLUS_AID:
            /* calculate the grp mask for RDS grp
             * which will contain actual eRT text
             *
             * Bit Pos  0  1  2  3  4   5  6   7
             * Grp Type 0A 0B 1A 1B 2A  2B 3A  3B
             *
             * similary for rest grps
             */
             mask_bit = (((agt >> 1) << 1) + (agt & 1));
             oda_agt =  (1 << mask_bit);
             /*Extract 5th bit of MSB (b7b6b5b4b3b2b1b0)*/
             rt_ert_flag = EXTRACT_BIT(temp.rdsBlk[2].rdsMsb,
                                              RT_ERT_FLAG_BIT);
             if (rt_plus_carrier != agt)
                 jni_cb->oda_update_cb();
             rt_plus_carrier = agt;
             break;
        default:
             oda_agt = 0;
             break;
        }
    } else {
        carrier = gtc;
        if ((carrier == rt_plus_carrier))
             hci_ev_rt_plus(temp);
        else if (carrier == ert_carrier)
             hci_buff_ert(&temp);
    }
}

void radio_hci_event_packet(char *evt_buf)
{
    char evt;

    ALOGE("%s:%s: Received %d bytes of HCI EVENT PKT from Controller", LOG_TAG,
                                      __func__, ((FM_EVT_HDR *)evt_buf)->evt_len);
    evt = ((FM_EVT_HDR *)evt_buf)->evt_code;
    ALOGE("%s:evt: %d", LOG_TAG, evt);

    switch(evt) {
    case HCI_EV_TUNE_STATUS:
        hci_ev_tune_status(((FM_EVT_HDR *)evt_buf)->cmd_params);
        break;
    case HCI_EV_SEARCH_PROGRESS:
    case HCI_EV_SEARCH_RDS_PROGRESS:
    case HCI_EV_SEARCH_LIST_PROGRESS:
        hci_ev_search_next(((FM_EVT_HDR *)evt_buf)->cmd_params);
        break;
    case HCI_EV_STEREO_STATUS:
        hci_ev_stereo_status(((FM_EVT_HDR *)evt_buf)->cmd_params);
        break;
    case HCI_EV_RDS_LOCK_STATUS:
        hci_ev_rds_lock_status(((FM_EVT_HDR *)evt_buf)->cmd_params);
        break;
/*    case HCI_EV_SERVICE_AVAILABLE:
        hci_ev_service_available(hdev, skb);
        break; */
    case HCI_EV_RDS_RX_DATA:
        hci_ev_raw_rds_group_data(((FM_EVT_HDR *)evt_buf)->cmd_params);
        break;
    case HCI_EV_PROGRAM_SERVICE:
        hci_ev_program_service(((FM_EVT_HDR *)evt_buf)->cmd_params);
        break;
    case HCI_EV_RADIO_TEXT:
        hci_ev_radio_text(((FM_EVT_HDR *)evt_buf)->cmd_params);
        break;
    case HCI_EV_FM_AF_LIST:
        hci_ev_af_list(((FM_EVT_HDR *)evt_buf)->cmd_params);
        break;
    case HCI_EV_CMD_COMPLETE:
        ALOGE("%s:%s: Received HCI_EV_CMD_COMPLETE", LOG_TAG, __func__);
        hci_cmd_complete_event(((FM_EVT_HDR *)evt_buf)->cmd_params);
        break;
    case HCI_EV_CMD_STATUS:
        hci_cmd_status_event(((FM_EVT_HDR *)evt_buf)->cmd_params);
        break;
    case HCI_EV_SEARCH_COMPLETE:
    case HCI_EV_SEARCH_RDS_COMPLETE:
        hci_ev_search_compl(((FM_EVT_HDR *)evt_buf)->cmd_params);
        break;
    case HCI_EV_SEARCH_LIST_COMPLETE:
        hci_ev_srch_st_list_compl(((FM_EVT_HDR *)evt_buf)->cmd_params);
        break;
    default:
        break;
    }
}

/* 'evt_buf' contains the event received from Controller */
int fm_evt_notify(char *evt_buf)
{
    ALOGI("%s: %s: Received event notification from FM-HCI thread. EVT CODE: %d ",
                            LOG_TAG,  __func__, ((FM_EVT_HDR *)evt_buf)->evt_code);
    radio_hci_event_packet(evt_buf);
    return 0;
}

int helium_search_req(int on, int direct)
{
    int retval = 0;
    enum search_t srch;
    int saved_val;
    int dir;

    srch = radio->g_search_mode & SRCH_MODE;
    saved_val = radio->search_on;
    radio->search_on = on;
    if (direct)
        dir = SRCH_DIR_UP;
    else
        dir = SRCH_DIR_DOWN;

    if (on) {
        switch (srch) {
        case SCAN_FOR_STRONG:
        case SCAN_FOR_WEAK:
            radio->srch_st_list.srch_list_dir = dir;
            radio->srch_st_list.srch_list_mode = srch;
            retval = helium_search_list(&radio->srch_st_list);
            break;
        case RDS_SEEK_PTY:
        case RDS_SCAN_PTY:
        case RDS_SEEK_PI:
            srch = srch - SEARCH_RDS_STNS_MODE_OFFSET;
            radio->srch_rds.srch_station.srch_mode = srch;
            radio->srch_rds.srch_station.srch_dir = dir;
            radio->srch_rds.srch_station.scan_time = radio->g_scan_time;
            retval = helium_search_rds_stations(&radio->srch_rds);
            break;
        default:
            radio->srch_st.srch_mode = srch;
            radio->srch_st.scan_time = radio->g_scan_time;
            radio->srch_st.srch_dir = dir;
            retval = helium_search_stations(&radio->srch_st);
            break;
        }
    } else {
        retval = helium_cancel_search_req();
    }

    if (retval < 0)
        radio->search_on = saved_val;
    return retval;
}

int helium_recv_set_region(int req_region)
{
    int retval;
    int saved_val;

    saved_val = radio->region;
    radio->region = req_region;

    retval = hci_fm_set_recv_conf_req(&radio->recv_conf);
    if (retval < 0)
        radio->region = saved_val;
    return retval;
}

int set_low_power_mode(int lp_mode)
{
    int rds_grps_proc = 0x00;
    int retval = 0;

    if (radio->power_mode != lp_mode) {
       if (lp_mode) {
           radio->event_mask = 0x00;
           if (radio->af_jump_bit)
               rds_grps_proc = 0x00 | AF_JUMP_ENABLE;
           else
               rds_grps_proc = 0x00;
           retval = helium_rds_grp_process_req(rds_grps_proc);
           if (retval < 0) {
               ALOGE("%s:Disable RDS failed", LOG_TAG);
               return retval;
           }
           retval = helium_set_event_mask_req(&radio->event_mask);
       } else {
           radio->event_mask = SIG_LEVEL_INTR | RDS_SYNC_INTR | AUDIO_CTRL_INTR;
           retval = helium_set_event_mask_req(&radio->event_mask);
           if (retval < 0) {
               ALOGE("%s:Enable Async events failed", LOG_TAG);
               return retval;
           }
           retval = helium_rds_grp_process_req(&radio->g_rds_grp_proc_ps);
       }
       radio->power_mode = lp_mode;
    }
    return retval;
}


/* Callback function to be registered with FM-HCI for event notification */
static fm_hal_cb hal_cb = {
    fm_evt_notify
};

int hal_init( fm_vendor_callbacks_t *p_cb)
{
    int ret = -1;

     radio = malloc(sizeof(struct helium_device));
     if (!radio) {
         ALOGE("%s:Failed to allocate memory for device", LOG_TAG);
         return ret;
     }
    /* Save the JNI callback functions */
    jni_cb = p_cb;

    /* Initialize the FM-HCI */
    ALOGE("%s:%s: Initializing the event notification func with FM-HCI", LOG_TAG, __func__);
    ret = fm_hci_init(&hal_cb);

    ALOGE("%s:%s: Turning FM ON...", LOG_TAG, __func__);
    fm_power(FM_RADIO_ENABLE);

    ALOGE("%s:%s: Firmware download and HCI Initialization in-progress...", LOG_TAG, __func__);
    /* TODO : Start the preload timer */
    open_serial_port();
    pthread_mutex_init(&fm_hal, NULL);
    return 0;
}

/* Called by the JNI for performing the FM operations */
static int set_fm_ctrl(int cmd, int val)
{
    int ret = 0;
    int saved_val;
    char temp_val = 0;
    unsigned int rds_grps_proc = 0;
    char *data;

    ALOGE("%s:cmd: %x, val: %d",LOG_TAG, cmd, val);
    switch (cmd) {
    case HCI_FM_HELIUM_AUDIO_MUTE:
        saved_val = radio->mute_mode.hard_mute;
        radio->mute_mode.hard_mute = val;
        ret = hci_fm_mute_mode_req(radio->mute_mode);
        if (ret < 0) {
            ALOGE("%s:Error while set FM hard mute %d", LOG_TAG, ret);
            radio->mute_mode.hard_mute = saved_val;
        }
        break;
    case HCI_FM_HELIUM_SRCHMODE:
            radio->g_search_mode = val;
        break;
    case HCI_FM_HELIUM_SCANDWELL:
            radio->g_scan_time = val;
        break;
    case HCI_FM_HELIUM_SRCHON:
        helium_search_req(val, SRCH_DIR_UP);
        break;
    case HCI_FM_HELIUM_STATE:
        switch (val) {
        case FM_RECV:
            ret = hci_fm_enable_recv_req();
            break;
        case FM_OFF:
            radio->mode = FM_TURNING_OFF;
            hci_fm_disable_recv_req();
            break;
        default:
            break;
        }
        break;
    case HCI_FM_HELIUM_REGION:
        ret = helium_recv_set_region(val);
        break;
    case HCI_FM_HELIUM_SIGNAL_TH:
        temp_val = val;
        ret = helium_set_sig_threshold_req(temp_val);
        if (ret < 0) {
            ALOGE("%s:Error while setting signal threshold\n", LOG_TAG);
            goto END;
        }
        break;
    case HCI_FM_HELIUM_SRCH_PTY:
         radio->srch_rds.srch_pty = val;
         radio->srch_st_list.srch_pty = val;
         break;
    case HCI_FM_HELIUM_SRCH_PI:
         radio->srch_rds.srch_pi = val;
         break;
    case HCI_FM_HELIUM_SRCH_CNT:
         radio->srch_st_list.srch_list_max = val;
         break;
    case HCI_FM_HELIUM_SPACING:
         saved_val = radio->recv_conf.ch_spacing;
         radio->recv_conf.ch_spacing = val;
         ret = hci_fm_set_recv_conf_req(&radio->recv_conf);
         if (ret < 0) {
             ALOGE("%s:Error in setting channel spacing", LOG_TAG);
             radio->recv_conf.ch_spacing = saved_val;
             goto END;
        }
        break;
    case HCI_FM_HELIUM_EMPHASIS:
         saved_val = radio->recv_conf.emphasis;
         radio->recv_conf.emphasis = val;
         ret = hci_fm_set_recv_conf_req(&radio->recv_conf);
         if (ret < 0) {
             ALOGE("%s:Error in setting emphasis", LOG_TAG);
             radio->recv_conf.emphasis = saved_val;
             goto END;
         }
         break;
    case HCI_FM_HELIUM_RDS_STD:
         saved_val = radio->recv_conf.rds_std;
         radio->recv_conf.rds_std = val;
         ret = hci_fm_set_recv_conf_req(&radio->recv_conf);
         if (ret < 0) {
             ALOGE("%s:Error in rds_std", LOG_TAG);
             radio->recv_conf.rds_std = saved_val;
             goto END;
         }
         break;
    case HCI_FM_HELIUM_RDSON:
         saved_val = radio->recv_conf.rds_std;
         radio->recv_conf.rds_std = val;
         ret = hci_fm_set_recv_conf_req(&radio->recv_conf);
         if (ret < 0) {
             ALOGE("%s:Error in rds_std", LOG_TAG);
             radio->recv_conf.rds_std = saved_val;
             goto END;
         }
         break;
    case HCI_FM_HELIUM_RDSGROUP_MASK:
         saved_val = radio->rds_grp.rds_grp_enable_mask;
         grp_mask = (grp_mask | oda_agt | val);
         radio->rds_grp.rds_grp_enable_mask = grp_mask;
         radio->rds_grp.rds_buf_size = 1;
         radio->rds_grp.en_rds_change_filter = 0;
         ret = helium_rds_grp_mask_req(&radio->rds_grp);
         if (ret < 0) {
             ALOGE("%s:error in setting group mask\n", LOG_TAG);
             radio->rds_grp.rds_grp_enable_mask = saved_val;
             goto END;
        }
        break;
    case HCI_FM_HELIUM_RDSGROUP_PROC:
         saved_val = radio->g_rds_grp_proc_ps;
         rds_grps_proc = radio->g_rds_grp_proc_ps | val;
         radio->g_rds_grp_proc_ps = (rds_grps_proc >> RDS_CONFIG_OFFSET);
         ret = helium_rds_grp_process_req(radio->g_rds_grp_proc_ps);
         if (ret < 0) {
             radio->g_rds_grp_proc_ps = saved_val;
             goto END;
         }
         break;
    case HCI_FM_HELIUM_RDSD_BUF:
         radio->rds_grp.rds_buf_size = val;
         break;
    case HCI_FM_HELIUM_PSALL:
         saved_val = radio->g_rds_grp_proc_ps;
         rds_grps_proc = (val << RDS_CONFIG_OFFSET);
         radio->g_rds_grp_proc_ps |= rds_grps_proc;
         ret = helium_rds_grp_process_req(radio->g_rds_grp_proc_ps);
         if (ret < 0) {
             radio->g_rds_grp_proc_ps = saved_val;
             goto END;
        }
        break;
    case HCI_FM_HELIUM_AF_JUMP:
        saved_val = radio->g_rds_grp_proc_ps;
        radio->g_rds_grp_proc_ps &= ~(1 << RDS_AF_JUMP_OFFSET);
        radio->af_jump_bit = val;
        rds_grps_proc = 0x00;
        rds_grps_proc = (val << RDS_AF_JUMP_OFFSET);
        radio->g_rds_grp_proc_ps |= rds_grps_proc;
        ret = helium_rds_grp_process_req(radio->g_rds_grp_proc_ps);
        if (ret < 0) {
            radio->g_rds_grp_proc_ps = saved_val;
            goto END;
        }
        break;
    case HCI_FM_HELIUM_LP_MODE:
         set_low_power_mode(val);
         break;
    case HCI_FM_HELIUM_ANTENNA:
        temp_val = val;
        ret = helium_set_antenna_req(temp_val);
        if (ret < 0) {
            ALOGE("%s:Set Antenna failed retval = %x", LOG_TAG, ret);
            goto END;
        }
        radio->g_antenna =  val;
        break;
    case HCI_FM_HELIUM_SOFT_MUTE:
         saved_val = radio->mute_mode.soft_mute;
         radio->mute_mode.soft_mute = val;
         ret = helium_set_fm_mute_mode_req(&radio->mute_mode);
         if (ret < 0) {
             ALOGE("%s:Error while setting FM soft mute %d", LOG_TAG, ret);
             radio->mute_mode.soft_mute = saved_val;
             goto END;
         }
         break;
    case HCI_FM_HELIUM_FREQ:
        hci_fm_tune_station_req(val);
        break;
    case HCI_FM_HELIUM_SEEK:
        helium_search_req(1, val);
        break;
    case HCI_FM_HELIUM_UPPER_BAND:
        radio->recv_conf.band_high_limit = val;
        break;
    case HCI_FM_HELIUM_LOWER_BAND:
        radio->recv_conf.band_low_limit = val;
        break;
    case HCI_FM_HELIUM_AUDIO_MODE:
        radio->stereo_mode.stereo_mode = ~val;
        hci_set_fm_stereo_mode_req(&radio->stereo_mode);
        break;
    default:
        ALOGE("%s:%s: Not a valid FM CMD!!", LOG_TAG, __func__);
        ret = 0;
        break;
    }
END:
    if (ret < 0)
        ALOGE("%s:%s: %d cmd failed", LOG_TAG, __func__, cmd);
    return ret;
}

static void get_fm_ctrl(int cmd, int val)
{
    int ret = 0;

    switch(cmd) {
    case HCI_FM_HELIUM_FREQ:
        val = radio->fm_st_rsp.station_rsp.station_freq;
        break;
    case HCI_FM_HELIUM_UPPER_BAND:
        val = radio->recv_conf.band_high_limit;
        break;
    case HCI_FM_HELIUM_LOWER_BAND:
        val = radio->recv_conf.band_low_limit;
        break;
    default:
        break;
    }
    if (ret < 0)
        ALOGE("%s:%s: %d cmd failed", LOG_TAG, __func__, cmd);
    return ret;
}

const fm_interface_t FM_HELIUM_LIB_INTERFACE = {
    hal_init,
    set_fm_ctrl,
    get_fm_ctrl
};
