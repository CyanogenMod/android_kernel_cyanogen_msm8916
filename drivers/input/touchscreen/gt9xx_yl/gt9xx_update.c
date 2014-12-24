/* drivers/input/touchscreen/gt9xx_update.c
 * 
 * 2010 - 2012 Goodix Technology.
 * 
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 * 
 * This program is distributed in the hope that it will be a reference 
 * to you, when you are integrating the GOODiX's CTP IC into your system, 
 * but WITHOUT ANY WARRANTY; without even the implied warranty of 
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU 
 * General Public License for more details.
 * 
 * Latest Version:1.9
 * Author: andrew@goodix.com
 * Revision Record: 
 *      V1.0:
 *          first release. By Andrew, 2012/08/31
 *      V1.2:
 *          add force update,GT9110P pid map. By Andrew, 2012/10/15
 *      V1.4:
 *          1. add config auto update function;
 *          2. modify enter_update_mode;
 *          3. add update file cal checksum.
 *                          By Andrew, 2012/12/12
 *      V1.6: 
 *          1. replace guitar_client with i2c_connect_client;
 *          2. support firmware header array update.
 *                          By Meta, 2013/03/11
 *      V1.9:
 *          1. fixed with pid judge bug about GT915S IC was upgraded to GT915 fw
 *          2. ESD init write 0x8041 with 0xAA only
 *          3. fixed with slide wakeup bug
 */
#include <linux/kthread.h>

#include "gt9xx.h"

#define GOODIX_FIRMWARE_INTER_UPDATE

#ifdef GOODIX_FIRMWARE_INTER_UPDATE
#include "goodix_firmware.h"
#endif

#define GUP_REG_HW_INFO             0x4220
#define GUP_REG_FW_MSG              0x41E4
#define GUP_REG_PID_VID             0x8140

#define GUP_SEARCH_FILE_TIMES       5
#define UPDATE_FILE_PATH_2          "/sdcard/goodix_update.bin"
#define UPDATE_FILE_PATH_1          "/data/goodix_update.bin"

#define FW_HEAD_LENGTH               14
#define FW_DOWNLOAD_LENGTH           0x4000
#define FW_SECTION_LENGTH            0x2000
#define FW_DSP_ISP_LENGTH            0x1000
#define FW_DSP_LENGTH                0x1000
#define FW_BOOT_LENGTH               0x800
#define FW_SS51_LENGTH               (4 * FW_SECTION_LENGTH)
#define FW_BOOT_ISP_LENGTH           0x800                     // 2k
#define FW_LINK_LENGTH               0x3000                    // 12k
#define FW_APP_CODE_LENGTH           (4 * FW_SECTION_LENGTH)   // 32k

#define FIRMWARE_LENGTH             (FW_SS51_LENGTH + FW_DSP_LENGTH + FW_BOOT_LENGTH + FW_DSP_ISP_LENGTH + FW_BOOT_ISP_LENGTH + FW_APP_CODE_LENGTH)

#define PACK_SIZE                    252
#define MAX_FRAME_CHECK_TIME         5

#define _bRW_MISCTL__SRAM_BANK       0x4048
#define _bRW_MISCTL__MEM_CD_EN       0x4049
#define _bRW_MISCTL__CACHE_EN        0x404B
#define _bRW_MISCTL__TMR0_EN         0x40B0
#define _rRW_MISCTL__SWRST_B0_       0x4180
#define _bWO_MISCTL__CPU_SWRST_PULSE 0x4184
#define _rRW_MISCTL__BOOTCTL_B0_     0x4190
#define _rRW_MISCTL__BOOT_OPT_B0_    0x4218
#define _rRW_MISCTL__BOOT_CTL_       0x5094

#define FAIL    0
#define SUCCESS 1

#define _CLOSE_FILE(p_file) if (p_file && !IS_ERR(p_file)) \
                            { \
                                filp_close(p_file, NULL); \
                            }

#pragma pack(1)
typedef struct 
{
    u8  hw_info[4];          //hardware info//
    u8  pid[8];              //product id   //
    u16 vid;                 //version id   //
}st_fw_head;
#pragma pack()

typedef struct
{
    u8 force_update;
    u8 fw_flag;
    struct file *file; 
    struct file *cfg_file;
    st_fw_head  ic_fw_msg;
    mm_segment_t old_fs;
    u32 fw_total_len;
    u32 fw_burned_len;
}st_update_msg;

st_update_msg update_msg;
//struct i2c_client *guitar_client = NULL;
u16 show_len;
u16 total_len;
u8 file_ready = 0;
extern s32  gtp_i2c_read(struct i2c_client *client, uint8_t *buf, s32 len);
extern s32  gtp_i2c_write(struct i2c_client *client,uint8_t *data,s32 len);
extern s32 gtp_init_panel(struct goodix_ts_data *ts);

extern void gtp_reset_guitar(struct i2c_client *client, s32 ms);
extern s32  gtp_send_cfg(struct i2c_client *client);
extern struct i2c_client * i2c_connect_client;
extern void gtp_irq_enable(struct goodix_ts_data *ts);
extern void gtp_irq_disable(struct goodix_ts_data *ts);
extern s32 gtp_i2c_read_dbl_check(struct i2c_client *, u16, u8 *, int);
static u8 gup_burn_fw_app_section(struct i2c_client *client, u8 *fw_section, u16 start_addr, u32 len, u8 bank_cmd );

static u8 gup_get_ic_msg(struct i2c_client *client, u16 addr, u8* msg, s32 len)
{
    s32 i = 0;

    msg[0] = (addr >> 8) & 0xff;
    msg[1] = addr & 0xff;

    for (i = 0; i < 5; i++)
    {
        if (gtp_i2c_read(client, msg, GTP_ADDR_LENGTH + len) > 0)
        {
            break;
        }
    }

    if (i >= 5)
    {
        GTP_ERROR("Read data from 0x%02x%02x failed!", msg[0], msg[1]);
        return FAIL;
    }

    return SUCCESS;
}

static u8 gup_set_ic_msg(struct i2c_client *client, u16 addr, u8 val)
{
    s32 i = 0;
    u8 msg[3];

    msg[0] = (addr >> 8) & 0xff;
    msg[1] = addr & 0xff;
    msg[2] = val;

    for (i = 0; i < 5; i++)
    {
        if (gtp_i2c_write(client, msg, GTP_ADDR_LENGTH + 1) > 0)
        {
            break;
        }
    }

    if (i >= 5)
    {
        GTP_ERROR("Set data to 0x%02x%02x failed!", msg[0], msg[1]);
        return FAIL;
    }

    return SUCCESS;
}

u8 gup_get_ic_fw_msg(struct i2c_client *client)
{
    s32 ret = -1;
    u8  retry = 0;
    u8  buf[16];
    u8  i;
    
    // step1:get hardware info
    ret = gtp_i2c_read_dbl_check(client, GUP_REG_HW_INFO, &buf[GTP_ADDR_LENGTH], 4);
    if (FAIL == ret)
    {
        GTP_ERROR("[get_ic_fw_msg]get hw_info failed,exit");
        return FAIL;
    }
     
    // buf[2~5]: 00 06 90 00
    // hw_info: 00 90 06 00
    for(i=0; i<4; i++)
    {
        update_msg.ic_fw_msg.hw_info[i] = buf[GTP_ADDR_LENGTH + 3 - i];
    } 
    GTP_DEBUG("IC Hardware info:%02x%02x%02x%02x", update_msg.ic_fw_msg.hw_info[0], update_msg.ic_fw_msg.hw_info[1],
                                                   update_msg.ic_fw_msg.hw_info[2], update_msg.ic_fw_msg.hw_info[3]);
    // step2:get firmware message
    for(retry=0; retry<2; retry++)
    {
        ret = gup_get_ic_msg(client, GUP_REG_FW_MSG, buf, 1);
        if(FAIL == ret)
        {
            GTP_ERROR("Read firmware message fail.");
            return ret;
        }
        
        update_msg.force_update = buf[GTP_ADDR_LENGTH];
        if((0xBE != update_msg.force_update)&&(!retry))
        {
            GTP_INFO("The check sum in ic is error.");
            GTP_INFO("The IC will be updated by force.");
            continue;
        }
        break;
    }

    GTP_INFO("IC force update flag:0x%x", update_msg.force_update);
    
    // step3:get pid & vid
    ret = gtp_i2c_read_dbl_check(client, GUP_REG_PID_VID, &buf[GTP_ADDR_LENGTH], 6);
    if (FAIL == ret)
    {
        GTP_ERROR("[get_ic_fw_msg]get pid & vid failed,exit");
        return FAIL;
    }
    
    memset(update_msg.ic_fw_msg.pid, 0, sizeof(update_msg.ic_fw_msg.pid));
    memcpy(update_msg.ic_fw_msg.pid, &buf[GTP_ADDR_LENGTH], 4);
    GTP_DEBUG("IC Product id:%s", update_msg.ic_fw_msg.pid);
    
    //GT9XX PID MAPPING
    /*|-----FLASH-----RAM-----|
      |------918------918-----|
      |------968------968-----|
      |------913------913-----|
      |------913P-----913P----|
      |------927------927-----|
      |------927P-----927P----|
      |------9110-----9110----|
      |------9110P----9111----|*/
    if(update_msg.ic_fw_msg.pid[0] != 0)
    {
        if(!memcmp(update_msg.ic_fw_msg.pid, "9111", 4))
        {
            GTP_DEBUG("IC Mapping Product id:%s", update_msg.ic_fw_msg.pid);
            memcpy(update_msg.ic_fw_msg.pid, "9110P", 5);
        }
    }
    
    update_msg.ic_fw_msg.vid = buf[GTP_ADDR_LENGTH+4] + (buf[GTP_ADDR_LENGTH+5]<<8);
    GTP_DEBUG("IC version id:%04x", update_msg.ic_fw_msg.vid);
    
    return SUCCESS;
}

s32 gup_enter_update_mode(struct i2c_client *client)
{
    s32 ret = -1;
    s32 retry = 0;
    u8 rd_buf[3];
	
    gtp_reset_guitar(client, 5);

    while(retry++ < 200)
    {
        //step4:Hold ss51 & dsp
        ret = gup_set_ic_msg(client, _rRW_MISCTL__SWRST_B0_, 0x0C);
        if(ret <= 0)
        {
            GTP_DEBUG("Hold ss51 & dsp I2C error,retry:%d", retry);
            continue;
        }
        
        //step5:Confirm hold
        ret = gup_get_ic_msg(client, _rRW_MISCTL__SWRST_B0_, rd_buf, 1);
        if(ret <= 0)
        {
            GTP_DEBUG("Hold ss51 & dsp I2C error,retry:%d", retry);
            continue;
        }
        if(0x0C == rd_buf[GTP_ADDR_LENGTH])
        {
            GTP_DEBUG("Hold ss51 & dsp confirm SUCCESS");
            break;
        }
        GTP_DEBUG("Hold ss51 & dsp confirm 0x4180 failed,value:%d", rd_buf[GTP_ADDR_LENGTH]);
    }
    if(retry >= 200)
    {
        GTP_ERROR("Enter update Hold ss51 failed.");
        return FAIL;
    }
    
    //step6:DSP_CK and DSP_ALU_CK PowerOn
    ret = gup_set_ic_msg(client, 0x4010, 0x00);
    
    //20121211 modify end
    return ret;
}

void gup_leave_update_mode(void)
{
    GTP_DEBUG("[leave_update_mode]reset chip.");
    gtp_reset_guitar(i2c_connect_client, 20);
}

// Get the correct nvram data
// The correct conditions: 
//  1. the hardware info is the same
//  2. the product id is the same
//  3. the firmware version in update file is greater than the firmware version in ic 
//      or the check sum in ic is wrong
/* Update Conditions: 
    1. Same hardware info
    2. Same PID
    3. File PID > IC PID
   Force Update Conditions:
    1. Wrong ic firmware checksum
    2. INVALID IC PID or VID
    3. IC PID == 91XX || File PID == 91XX
*/

static u8 gup_enter_update_judge(st_fw_head *fw_head)
{
    u16 u16_tmp;
    s32 i = 0;
    u32 fw_len = 0;
    s32 pid_cmp_len = 0;
    
    u16_tmp = fw_head->vid;
    fw_head->vid = (u16)(u16_tmp>>8) + (u16)(u16_tmp<<8);

    GTP_DEBUG("FILE HARDWARE INFO:%02x%02x%02x%02x", fw_head->hw_info[0], fw_head->hw_info[1], fw_head->hw_info[2], fw_head->hw_info[3]);
    GTP_DEBUG("FILE PID:%s", fw_head->pid);
    GTP_DEBUG("FILE VID:%04x", fw_head->vid);

    GTP_DEBUG("IC HARDWARE INFO:%02x%02x%02x%02x", update_msg.ic_fw_msg.hw_info[0], update_msg.ic_fw_msg.hw_info[1],
                                                   update_msg.ic_fw_msg.hw_info[2], update_msg.ic_fw_msg.hw_info[3]);
    GTP_DEBUG("IC PID:%s", update_msg.ic_fw_msg.pid);
    GTP_DEBUG("IC VID:%04x", update_msg.ic_fw_msg.vid);

    if (!memcmp(fw_head->pid, "9158", 4) && !memcmp(update_msg.ic_fw_msg.pid, "915S", 4))
    {
        GTP_INFO("Update GT915S to GT9158 directly!");
        return SUCCESS;
    }

    if (!memcmp(fw_head->hw_info, update_msg.ic_fw_msg.hw_info, sizeof(update_msg.ic_fw_msg.hw_info)))
    {
        fw_len = 42 * 1024;
        //TODO flashless equals another length
    }
    else
    {
        fw_len = fw_head->hw_info[3];
        fw_len += (((u32)fw_head->hw_info[2]) << 8);
        fw_len += (((u32)fw_head->hw_info[1]) << 16);
        fw_len += (((u32)fw_head->hw_info[0]) << 24);
    }
    if (update_msg.fw_total_len != fw_len)
    {
        GTP_ERROR("Inconsistent firmware size, Update aborted! Firmware header len: %d, actual size: %d", fw_len, update_msg.fw_total_len);
        return FAIL;
    }
    if ((update_msg.fw_total_len < 36*1024) || (update_msg.fw_total_len > 128*1024))
    {
        GTP_ERROR("Invalid firmware length(%d), update aborted!", update_msg.fw_total_len);
        return FAIL;
    }
    GTP_INFO("Firmware length:%d", update_msg.fw_total_len);
    
    if (update_msg.force_update != 0xBE)
    {
        GTP_INFO("FW chksum error,need enter update.");
        return SUCCESS;
    }
    
    // 20130523 start
    if (strlen(update_msg.ic_fw_msg.pid) < 3)
    {
        GTP_INFO("Illegal IC pid, need enter update");
        return SUCCESS;
    }
    else
    {
        for (i = 0; i < 3; i++)
        {
            if ((update_msg.ic_fw_msg.pid[i] < 0x30) || (update_msg.ic_fw_msg.pid[i] > 0x39))
            {
                GTP_INFO("Illegal IC pid, out of bound, need enter update");
                return SUCCESS;
            }
        }
    }
    // 20130523 end
    
    pid_cmp_len = strlen(fw_head->pid);
    if (pid_cmp_len < strlen(update_msg.ic_fw_msg.pid))
    {
        pid_cmp_len = strlen(update_msg.ic_fw_msg.pid);
    }
    
    if ((!memcmp(fw_head->pid, update_msg.ic_fw_msg.pid, pid_cmp_len)) ||
            (!memcmp(update_msg.ic_fw_msg.pid, "91XX", 4))||
            (!memcmp(fw_head->pid, "91XX", 4)))
    {
        if(!memcmp(fw_head->pid, "91XX", 4))
        {
            GTP_DEBUG("Force none same pid update mode.");
        }
        else
        {
            GTP_DEBUG("Get the same pid.");
        }

        //The third condition
        if (fw_head->vid > update_msg.ic_fw_msg.vid)
        {

            GTP_INFO("Need enter update.");
            return SUCCESS;
        }

        GTP_INFO("Don't meet the third condition.");
        GTP_ERROR("File VID <= IC VID, update aborted!");
    }
    else
    {
        GTP_ERROR("File PID != IC PID, update aborted!");
    }

    return FAIL;
}

#ifndef GOODIX_FIRMWARE_INTER_UPDATE
static u8 ascii2hex(u8 a)
{
    s8 value = 0;

    if(a >= '0' && a <= '9')
    {
        value = a - '0';
    }
    else if(a >= 'A' && a <= 'F')
    {
        value = a - 'A' + 0x0A;
    }
    else if(a >= 'a' && a <= 'f')
    {
        value = a - 'a' + 0x0A;
    }
    else
    {
        value = 0xff;
    }
    
    return value;
}

static s8 gup_update_config(struct i2c_client *client)
{
    s32 file_len = 0;
    s32 ret = 0;
    s32 i = 0;
    s32 file_cfg_len = 0;
    s32 chip_cfg_len = 0;
    s32 count = 0;
    u8 *buf;
    u8 *pre_buf;
    u8 *file_config;
    //u8 checksum = 0;
    u8 pid[8];
    
    if(NULL == update_msg.cfg_file)
    {
        GTP_ERROR("[update_cfg]No need to upgrade config!");
        return FAIL;
    }
    file_len = update_msg.cfg_file->f_op->llseek(update_msg.cfg_file, 0, SEEK_END);
    
    ret = gup_get_ic_msg(client, GUP_REG_PID_VID, pid, 6);
    if(FAIL == ret)
    {
        GTP_ERROR("[update_cfg]Read product id & version id fail.");
        return FAIL;
    }
    pid[5] = '\0';
    GTP_DEBUG("update cfg get pid:%s", &pid[GTP_ADDR_LENGTH]);
    
    chip_cfg_len = 186;
    if(!memcmp(&pid[GTP_ADDR_LENGTH], "968", 3) || 
       !memcmp(&pid[GTP_ADDR_LENGTH], "910", 3) ||
       !memcmp(&pid[GTP_ADDR_LENGTH], "960", 3))
    {
        chip_cfg_len = 228;
    }
    GTP_DEBUG("[update_cfg]config file len:%d", file_len);
    GTP_DEBUG("[update_cfg]need config len:%d",chip_cfg_len);
    if((file_len+5) < chip_cfg_len*5)
    {
        GTP_ERROR("Config length error");
        return -1;
    }
    
    buf = (u8*)kzalloc(file_len, GFP_KERNEL);
    pre_buf = (u8*)kzalloc(file_len, GFP_KERNEL);
    file_config = (u8*)kzalloc(chip_cfg_len + GTP_ADDR_LENGTH, GFP_KERNEL);
    update_msg.cfg_file->f_op->llseek(update_msg.cfg_file, 0, SEEK_SET);
    
    GTP_DEBUG("[update_cfg]Read config from file.");
    ret = update_msg.cfg_file->f_op->read(update_msg.cfg_file, (char*)pre_buf, file_len, &update_msg.cfg_file->f_pos);
    if(ret<0)
    {
        GTP_ERROR("[update_cfg]Read config file failed.");
        goto update_cfg_file_failed;
    }
    
    GTP_DEBUG("[update_cfg]Delete illgal charactor.");
    for(i=0,count=0; i<file_len; i++)
    {
        if (pre_buf[i] == ' ' || pre_buf[i] == '\r' || pre_buf[i] == '\n')
        {
            continue;
        }
        buf[count++] = pre_buf[i];
    }
    
    GTP_DEBUG("[update_cfg]Ascii to hex.");
    file_config[0] = GTP_REG_CONFIG_DATA >> 8;
    file_config[1] = GTP_REG_CONFIG_DATA & 0xff;
    for(i=0,file_cfg_len=GTP_ADDR_LENGTH; i<count; i+=5)
    {
        if((buf[i]=='0') && ((buf[i+1]=='x') || (buf[i+1]=='X')))
        {
            u8 high,low;
            high = ascii2hex(buf[i+2]);
            low = ascii2hex(buf[i+3]);
            
            if((high == 0xFF) || (low == 0xFF))
            {
                ret = 0;
                GTP_ERROR("[update_cfg]Illegal config file.");
                goto update_cfg_file_failed;
            }
            file_config[file_cfg_len++] = (high<<4) + low;
        }
        else
        {
            ret = 0;
            GTP_ERROR("[update_cfg]Illegal config file.");
            goto update_cfg_file_failed;
        }
    }
    
//    //cal checksum
//    for(i=GTP_ADDR_LENGTH; i<chip_cfg_len; i++)
//    {
//        checksum += file_config[i];
//    }
//    file_config[chip_cfg_len] = (~checksum) + 1;
//    file_config[chip_cfg_len+1] = 0x01;
    
    GTP_DEBUG("config:");
    GTP_DEBUG_ARRAY(file_config+2, file_cfg_len);
    
    i = 0;
    while(i++ < 5)
    {
        ret = gtp_i2c_write(client, file_config, file_cfg_len);
        if(ret > 0)
        {
            GTP_INFO("[update_cfg]Send config SUCCESS.");
            break;
        }
        GTP_ERROR("[update_cfg]Send config i2c error.");
    }
    
update_cfg_file_failed:
    kfree(pre_buf);
    kfree(buf);
    kfree(file_config);
    return ret;
}
#endif

u8 gup_check_update_file(struct i2c_client *client, st_fw_head* fw_head, u8* path)
{
    s32 ret = 0;
    s32 i = 0;
    s32 fw_checksum = 0;
    u8 buf[FW_HEAD_LENGTH];
    
    if (path)
    {
        GTP_DEBUG("Update File path:%s, %zu", path, strlen(path));
        update_msg.file = filp_open(path, O_RDONLY, 0);

        if (IS_ERR(update_msg.file))
        {
            GTP_ERROR("Open update file(%s) error!", path);
            return FAIL;
        }
        file_ready = 1;
    }
    else
    {
#ifdef GOODIX_FIRMWARE_INTER_UPDATE
        update_msg.fw_total_len = sizeof(goodix_firmware) - FW_HEAD_LENGTH;
        
        if (sizeof(goodix_firmware) < (FW_HEAD_LENGTH+FW_SECTION_LENGTH*4+FW_DSP_ISP_LENGTH+FW_DSP_LENGTH+FW_BOOT_LENGTH))
        {
            GTP_ERROR("INVALID goodix_firmware, check your gt9xx_firmware.h file!");
            return FAIL;           
        }
        
        GTP_DEBUG("Firmware actual size: %dBytes", update_msg.fw_total_len);
        
        memcpy(fw_head, goodix_firmware, FW_HEAD_LENGTH);
        
        fw_checksum = 0;
        for (i = 0; i < update_msg.fw_total_len; i += 2)
        {
            fw_checksum += (goodix_firmware[FW_HEAD_LENGTH + i] << 8) + goodix_firmware[FW_HEAD_LENGTH + i + 1];
        }
        if (fw_checksum & 0xFFFF)
        {
            GTP_ERROR("Illegal firmware with checksum(0x%04X)", fw_checksum);
            return FAIL;
        }
        file_ready = 0;
		
        ret = gup_enter_update_judge(fw_head);
        if(SUCCESS == ret)
        {
            GTP_INFO("Check firmware file success.");
            return SUCCESS;
        }
        return FAIL;
#else
        u8 fp_len = max(sizeof(UPDATE_FILE_PATH_1), sizeof(UPDATE_FILE_PATH_2));
        u8 cfp_len = max(sizeof(CONFIG_FILE_PATH_1), sizeof(CONFIG_FILE_PATH_2));
        u8 *search_update_path = (u8*)kzalloc(fp_len, GFP_KERNEL);
        u8 *search_cfg_path = (u8*)kzalloc(cfp_len, GFP_KERNEL);
        //Begin to search update file,the config file & firmware file must be in the same path,single or double.
        searching_file = 1;
        for (i = 0; i < GUP_SEARCH_FILE_TIMES; i++)
        {
            if (searching_file == 0)
            {
                kfree(search_update_path);
                kfree(search_cfg_path);
                GTP_INFO(".bin/.cfg update file search forcely terminated!");
                return FAIL;
            }
            if(i%2)
            {
                memcpy(search_update_path, UPDATE_FILE_PATH_1, sizeof(UPDATE_FILE_PATH_1));
                memcpy(search_cfg_path, CONFIG_FILE_PATH_1, sizeof(CONFIG_FILE_PATH_1));
            }
            else
            {
                memcpy(search_update_path, UPDATE_FILE_PATH_2, sizeof(UPDATE_FILE_PATH_2));
                memcpy(search_cfg_path, CONFIG_FILE_PATH_2, sizeof(CONFIG_FILE_PATH_2));
            }
            
            if(!(got_file_flag&0x0F))
            {
                update_msg.file = filp_open(search_update_path, O_RDONLY, 0);
                if(!IS_ERR(update_msg.file))
                {
                    GTP_DEBUG("Find the bin file");
                    got_file_flag |= 0x0F;
                }
            }
            if(!(got_file_flag&0xF0))
            {
                update_msg.cfg_file = filp_open(search_cfg_path, O_RDONLY, 0);
                if(!IS_ERR(update_msg.cfg_file))
                {
                    GTP_DEBUG("Find the cfg file");
                    got_file_flag |= 0xF0;
                }
            }
            
            if(got_file_flag)
            {
                if(got_file_flag == 0xFF)
                {
                    break;
                }
                else
                {
                    i += 4;
                }
            }
            GTP_DEBUG("%3d:Searching %s %s file...", i, (got_file_flag&0x0F)?"":"bin", (got_file_flag&0xF0)?"":"cfg");
            msleep(3000);
        }
        searching_file = 0;
        kfree(search_update_path);
        kfree(search_cfg_path);
        
        if(!got_file_flag)
        {
            GTP_ERROR("Can't find update file.");
            goto load_failed;
        }
        
        if(got_file_flag&0xF0)
        {
            GTP_DEBUG("Got the update config file.");
            ret = gup_update_config(client);
            if(ret <= 0)
            {
                GTP_ERROR("Update config failed.");
            }
            _CLOSE_FILE(update_msg.cfg_file);
            msleep(500);                //waiting config to be stored in FLASH.
        }
        if(got_file_flag&0x0F)
        {
            GTP_DEBUG("Got the update firmware file.");
        }
        else
        {
            GTP_ERROR("No need to upgrade firmware.");
            goto load_failed;
        }
#endif
    }
    
    file_ready = 1;
    update_msg.old_fs = get_fs();
    set_fs(KERNEL_DS);

    update_msg.file->f_op->llseek(update_msg.file, 0, SEEK_SET);
    update_msg.fw_total_len = update_msg.file->f_op->llseek(update_msg.file, 0, SEEK_END);
    
    update_msg.fw_total_len -= FW_HEAD_LENGTH;
    
    GTP_DEBUG("Bin firmware actual size: %dBytes.", update_msg.fw_total_len);
    
    update_msg.file->f_op->llseek(update_msg.file, 0, SEEK_SET);
    ret = update_msg.file->f_op->read(update_msg.file, (char*)buf, FW_HEAD_LENGTH, &update_msg.file->f_pos);
    if (ret < 0)
    {
        GTP_ERROR("Read firmware head in update file error.");
        goto load_failed;
    }
    memcpy(fw_head, buf, FW_HEAD_LENGTH);
    
    //check firmware legality
    fw_checksum = 0;
    for(i=0; i<update_msg.fw_total_len; i+=2)
    {
        u16 temp;
        ret = update_msg.file->f_op->read(update_msg.file, (char*)buf, 2, &update_msg.file->f_pos);
        if (ret < 0)
        {
            GTP_ERROR("Read firmware file error.");
            goto load_failed;
        }
        //GTP_DEBUG("BUF[0]:%x", buf[0]);
        temp = (buf[0]<<8) + buf[1];
        fw_checksum += temp;
    }
    
    GTP_DEBUG("firmware checksum:%x", fw_checksum&0xFFFF);
    if(fw_checksum&0xFFFF)
    {
        GTP_ERROR("Illegal firmware file.");
        goto load_failed;    
    }

    ret = gup_enter_update_judge(fw_head);
    if(SUCCESS == ret)
    {
        GTP_INFO("Check *.bin file success.");
        return SUCCESS;
    }

load_failed:
    set_fs(update_msg.old_fs);
    return FAIL;
}

#if 0
static u8 gup_check_update_header(struct i2c_client *client, st_fw_head* fw_head)
{
    const u8* pos;
    int i = 0;
    u8 mask_num = 0;
    s32 ret = 0;

    pos = HEADER_UPDATE_DATA;
      
    memcpy(fw_head, pos, FW_HEAD_LENGTH);
    pos += FW_HEAD_LENGTH;

    ret = gup_enter_update_judge(fw_head);
    if(SUCCESS == ret)
    {
        return SUCCESS;
    }
    return FAIL;
}
#endif

static u8 gup_burn_proc(struct i2c_client *client, u8 *burn_buf, u16 start_addr, u16 total_length)
{
    s32 ret = 0;
    u16 burn_addr = start_addr;
    u16 frame_length = 0;
    u16 burn_length = 0;
    u8  wr_buf[PACK_SIZE + GTP_ADDR_LENGTH];
    u8  rd_buf[PACK_SIZE + GTP_ADDR_LENGTH];
    u8  retry = 0;
    
    GTP_DEBUG("Begin burn %dk data to addr 0x%x", (total_length/1024), start_addr);
    while(burn_length < total_length)
    {
        GTP_DEBUG("B/T:%04d/%04d", burn_length, total_length);
        frame_length = ((total_length - burn_length) > PACK_SIZE) ? PACK_SIZE : (total_length - burn_length);
        wr_buf[0] = (u8)(burn_addr>>8);
        rd_buf[0] = wr_buf[0];
        wr_buf[1] = (u8)burn_addr;
        rd_buf[1] = wr_buf[1];
        memcpy(&wr_buf[GTP_ADDR_LENGTH], &burn_buf[burn_length], frame_length);
        
        for(retry = 0; retry < MAX_FRAME_CHECK_TIME; retry++)
        {
            ret = gtp_i2c_write(client, wr_buf, GTP_ADDR_LENGTH + frame_length);
            if(ret <= 0)
            {
                GTP_ERROR("Write frame data i2c error.");
                continue;
            }
            ret = gtp_i2c_read(client, rd_buf, GTP_ADDR_LENGTH + frame_length);
            if(ret <= 0)
            {
                GTP_ERROR("Read back frame data i2c error.");
                continue;
            }
            
            if(memcmp(&wr_buf[GTP_ADDR_LENGTH], &rd_buf[GTP_ADDR_LENGTH], frame_length))
            {
                GTP_ERROR("Check frame data fail,not equal.");
                GTP_DEBUG("write array:");
                GTP_DEBUG_ARRAY(&wr_buf[GTP_ADDR_LENGTH], frame_length);
                GTP_DEBUG("read array:");
                GTP_DEBUG_ARRAY(&rd_buf[GTP_ADDR_LENGTH], frame_length);
                continue;
            }
            else
            {
                //GTP_DEBUG("Check frame data success.");
                break;
            }
        }
        if(retry >= MAX_FRAME_CHECK_TIME)
        {
            GTP_ERROR("Burn frame data time out,exit.");
            return FAIL;
        }
        burn_length += frame_length;
        burn_addr += frame_length;
    }
    return SUCCESS;
}

static u8 gup_load_section_file(u8 *buf, u32 offset, u16 length, u8 set_or_end)
{
#ifdef GOODIX_FIRMWARE_INTER_UPDATE
    if (file_ready == 0)
    {
        if(SEEK_SET == set_or_end)
        {
            memcpy(buf, &goodix_firmware[FW_HEAD_LENGTH + offset], length);
        }
        else    //seek end
        {
            memcpy(buf, &goodix_firmware[update_msg.fw_total_len + FW_HEAD_LENGTH - offset], length);
        }
        return SUCCESS;
    }
#endif
    {
        s32 ret = 0;
    
        if (update_msg.file == NULL)
        {
            GTP_ERROR("cannot find update file,load section file fail.");
            return FAIL;
        }
        
        if(SEEK_SET == set_or_end)
        {
            update_msg.file->f_pos = FW_HEAD_LENGTH + offset;
        }
        else    //seek end
        {
            update_msg.file->f_pos = update_msg.fw_total_len + FW_HEAD_LENGTH - offset;
        }
    
        ret = update_msg.file->f_op->read(update_msg.file, (char *)buf, length, &update_msg.file->f_pos);
    
        if (ret < 0)
        {
            GTP_ERROR("Read update file fail.");
            return FAIL;
        }
    
        return SUCCESS;
    }
}

static u8 gup_recall_check(struct i2c_client *client, u8* chk_src, u16 start_rd_addr, u16 chk_length)
{
    u8  rd_buf[PACK_SIZE + GTP_ADDR_LENGTH];
    s32 ret = 0;
    u16 recall_addr = start_rd_addr;
    u16 recall_length = 0;
    u16 frame_length = 0;

    while(recall_length < chk_length)
    {
        frame_length = ((chk_length - recall_length) > PACK_SIZE) ? PACK_SIZE : (chk_length - recall_length);
        ret = gup_get_ic_msg(client, recall_addr, rd_buf, frame_length);
        if(ret <= 0)
        {
            GTP_ERROR("recall i2c error,exit");
            return FAIL;
        }
        
        if(memcmp(&rd_buf[GTP_ADDR_LENGTH], &chk_src[recall_length], frame_length))
        {
            GTP_ERROR("Recall frame data fail,not equal.");
            GTP_DEBUG("chk_src array:");
            GTP_DEBUG_ARRAY(&chk_src[recall_length], frame_length);
            GTP_DEBUG("recall array:");
            GTP_DEBUG_ARRAY(&rd_buf[GTP_ADDR_LENGTH], frame_length);
            return FAIL;
        }
        
        recall_length += frame_length;
        recall_addr += frame_length;
    }
    GTP_DEBUG("Recall check %dk firmware success.", (chk_length/1024));
    
    return SUCCESS;
}

static u8 gup_burn_fw_section(struct i2c_client *client, u8 *fw_section, u16 start_addr, u8 bank_cmd )
{
    s32 ret = 0;
    u8  rd_buf[5];
  
    //step1:hold ss51 & dsp
    ret = gup_set_ic_msg(client, _rRW_MISCTL__SWRST_B0_, 0x0C);
    if(ret <= 0)
    {
        GTP_ERROR("[burn_fw_section]hold ss51 & dsp fail.");
        return FAIL;
    }
    
    //step2:set scramble
    ret = gup_set_ic_msg(client, _rRW_MISCTL__BOOT_OPT_B0_, 0x00);
    if(ret <= 0)
    {
        GTP_ERROR("[burn_fw_section]set scramble fail.");
        return FAIL;
    }
    
    //step3:select bank
    ret = gup_set_ic_msg(client, _bRW_MISCTL__SRAM_BANK, (bank_cmd >> 4)&0x0F);
    if(ret <= 0)
    {
        GTP_ERROR("[burn_fw_section]select bank %d fail.", (bank_cmd >> 4)&0x0F);
        return FAIL;
    }
    
    //step4:enable accessing code
    ret = gup_set_ic_msg(client, _bRW_MISCTL__MEM_CD_EN, 0x01);
    if(ret <= 0)
    {
        GTP_ERROR("[burn_fw_section]enable accessing code fail.");
        return FAIL;
    }
    
    //step5:burn 8k fw section
    ret = gup_burn_proc(client, fw_section, start_addr, FW_SECTION_LENGTH);
    if(FAIL == ret)
    {
        GTP_ERROR("[burn_fw_section]burn fw_section fail.");
        return FAIL;
    }
    
    //step6:hold ss51 & release dsp
    ret = gup_set_ic_msg(client, _rRW_MISCTL__SWRST_B0_, 0x04);
    if(ret <= 0)
    {
        GTP_ERROR("[burn_fw_section]hold ss51 & release dsp fail.");
        return FAIL;
    }
    //must delay
    msleep(1);
    
    //step7:send burn cmd to move data to flash from sram
    ret = gup_set_ic_msg(client, _rRW_MISCTL__BOOT_CTL_, bank_cmd&0x0f);
    if(ret <= 0)
    {
        GTP_ERROR("[burn_fw_section]send burn cmd fail.");
        return FAIL;
    }
    GTP_DEBUG("[burn_fw_section]Wait for the burn is complete......");
    do{
        ret = gup_get_ic_msg(client, _rRW_MISCTL__BOOT_CTL_, rd_buf, 1);
        if(ret <= 0)
        {
            GTP_ERROR("[burn_fw_section]Get burn state fail");
            return FAIL;
        }
        msleep(10);
        //GTP_DEBUG("[burn_fw_section]Get burn state:%d.", rd_buf[GTP_ADDR_LENGTH]);
    }while(rd_buf[GTP_ADDR_LENGTH]);

    //step8:select bank
    ret = gup_set_ic_msg(client, _bRW_MISCTL__SRAM_BANK, (bank_cmd >> 4)&0x0F);
    if(ret <= 0)
    {
        GTP_ERROR("[burn_fw_section]select bank %d fail.", (bank_cmd >> 4)&0x0F);
        return FAIL;
    }
    
    //step9:enable accessing code
    ret = gup_set_ic_msg(client, _bRW_MISCTL__MEM_CD_EN, 0x01);
    if(ret <= 0)
    {
        GTP_ERROR("[burn_fw_section]enable accessing code fail.");
        return FAIL;
    }
    
    //step10:recall 8k fw section
    ret = gup_recall_check(client, fw_section, start_addr, FW_SECTION_LENGTH);
    if(FAIL == ret)
    {
        GTP_ERROR("[burn_fw_section]recall check %dk firmware fail.", FW_SECTION_LENGTH/1024);
        return FAIL;
    }
    
    //step11:disable accessing code
    ret = gup_set_ic_msg(client, _bRW_MISCTL__MEM_CD_EN, 0x00);
    if(ret <= 0)
    {
        GTP_ERROR("[burn_fw_section]disable accessing code fail.");
        return FAIL;
    }
    
    return SUCCESS;
}

static u8 gup_burn_dsp_isp(struct i2c_client *client)
{
    s32 ret = 0;
    u8* fw_dsp_isp = NULL;
    u8  retry = 0;
    
    GTP_DEBUG("[burn_dsp_isp]Begin burn dsp isp---->>");
    
    //step1:alloc memory
    GTP_DEBUG("[burn_dsp_isp]step1:alloc memory");
    while(retry++ < 5)
    {
        fw_dsp_isp = (u8*)kzalloc(FW_DSP_ISP_LENGTH, GFP_KERNEL);
        if(fw_dsp_isp == NULL)
        {
            continue;
        }
        else
        {
            GTP_INFO("[burn_dsp_isp]Alloc %dk byte memory success.", (FW_DSP_ISP_LENGTH/1024));
            break;
        }
    }
    if(retry >= 5)
    {
        GTP_ERROR("[burn_dsp_isp]Alloc memory fail,exit.");
        return FAIL;
    }
    
    //step2:load dsp isp file data
    GTP_DEBUG("[burn_dsp_isp]step2:load dsp isp file data");
    //offset = FW_HEAD_LENGTH + FW_SS51_LENGTH + FW_DSP_LENGTH + FW_BOOT_LENGTH + FW_BOOT_ISP_LENGTH + FW_LINK_LENGTH;
    //ret = gup_load_section_file(fw_dsp_isp, offset, FW_DSP_ISP_LENGTH, SEEK_SET);
    ret = gup_load_section_file(fw_dsp_isp, FW_DSP_ISP_LENGTH, FW_DSP_ISP_LENGTH, SEEK_END);
    //gup_show_elements(update_msg.fw_total_len + FW_HEAD_LENGTH - FW_DSP_ISP_LENGTH, fw_dsp_isp, 10); // for test

    if(FAIL == ret)
    {
        GTP_ERROR("[burn_dsp_isp]load firmware dsp_isp fail.");
        goto exit_burn_dsp_isp;
    }
    
    //step3:disable wdt,clear cache enable
    GTP_DEBUG("[burn_dsp_isp]step3:disable wdt,clear cache enable");
    ret = gup_set_ic_msg(client, _bRW_MISCTL__TMR0_EN, 0x00);
    if(ret <= 0)
    {
        GTP_ERROR("[burn_dsp_isp]disable wdt fail.");
        ret = FAIL;
        goto exit_burn_dsp_isp;
    }
    ret = gup_set_ic_msg(client, _bRW_MISCTL__CACHE_EN, 0x00);
    if(ret <= 0)
    {
        GTP_ERROR("[burn_dsp_isp]clear cache enable fail.");
        ret = FAIL;
        goto exit_burn_dsp_isp;
    }
    
    //step4:hold ss51 & dsp
    GTP_DEBUG("[burn_dsp_isp]step4:hold ss51 & dsp");
    ret = gup_set_ic_msg(client, _rRW_MISCTL__SWRST_B0_, 0x0C);
    if(ret <= 0)
    {
        GTP_ERROR("[burn_dsp_isp]hold ss51 & dsp fail.");
        ret = FAIL;
        goto exit_burn_dsp_isp;
    }
    
    //step5:set boot from sram
    GTP_DEBUG("[burn_dsp_isp]step5:set boot from sram");
    ret = gup_set_ic_msg(client, _rRW_MISCTL__BOOTCTL_B0_, 0x02);
    if(ret <= 0)
    {
        GTP_ERROR("[burn_dsp_isp]set boot from sram fail.");
        ret = FAIL;
        goto exit_burn_dsp_isp;
    }
    
    //step6:software reboot
    GTP_DEBUG("[burn_dsp_isp]step6:software reboot");
    ret = gup_set_ic_msg(client, _bWO_MISCTL__CPU_SWRST_PULSE, 0x01);
    if(ret <= 0)
    {
        GTP_ERROR("[burn_dsp_isp]software reboot fail.");
        ret = FAIL;
        goto exit_burn_dsp_isp;
    }
    
    //step7:select bank2
    GTP_DEBUG("[burn_dsp_isp]step7:select bank2");
    ret = gup_set_ic_msg(client, _bRW_MISCTL__SRAM_BANK, 0x02);
    if(ret <= 0)
    {
        GTP_ERROR("[burn_dsp_isp]select bank2 fail.");
        ret = FAIL;
        goto exit_burn_dsp_isp;
    }
    
    //step8:enable accessing code
    GTP_DEBUG("[burn_dsp_isp]step8:enable accessing code");
    ret = gup_set_ic_msg(client, _bRW_MISCTL__MEM_CD_EN, 0x01);
    if(ret <= 0)
    {
        GTP_ERROR("[burn_dsp_isp]enable accessing code fail.");
        ret = FAIL;
        goto exit_burn_dsp_isp;
    }
    
    //step9:burn 4k dsp_isp
    GTP_DEBUG("[burn_dsp_isp]step9:burn 4k dsp_isp");
    ret = gup_burn_proc(client, fw_dsp_isp, 0xC000, FW_DSP_ISP_LENGTH);
    if(FAIL == ret)
    {
        GTP_ERROR("[burn_dsp_isp]burn dsp_isp fail.");
        goto exit_burn_dsp_isp;
    }
    
    //step10:set scramble
    GTP_DEBUG("[burn_dsp_isp]step10:set scramble");
    ret = gup_set_ic_msg(client, _rRW_MISCTL__BOOT_OPT_B0_, 0x00);
    if(ret <= 0)
    {
        GTP_ERROR("[burn_dsp_isp]set scramble fail.");
        ret = FAIL;
        goto exit_burn_dsp_isp;
    }

    update_msg.fw_burned_len += FW_DSP_ISP_LENGTH;
    GTP_DEBUG("[burn_dsp_isp]Burned length:%d", update_msg.fw_burned_len);
    ret = SUCCESS;

exit_burn_dsp_isp:
    kfree(fw_dsp_isp);
    return ret;
}

static u8 gup_burn_fw_ss51(struct i2c_client *client)
{
    u8* fw_ss51 = NULL;
    u8  retry = 0;
    s32 ret = 0;
    
    GTP_DEBUG("[burn_fw_ss51]Begin burn ss51 firmware---->>");
    
    //step1:alloc memory
    GTP_DEBUG("[burn_fw_ss51]step1:alloc memory");
    while(retry++ < 5)
    {
        fw_ss51 = (u8*)kzalloc(FW_SECTION_LENGTH, GFP_KERNEL);
        if(fw_ss51 == NULL)
        {
            continue;
        }
        else
        {
            GTP_INFO("[burn_fw_ss51]Alloc %dk byte memory success.", (FW_SECTION_LENGTH/1024));
            break;
        }
    }
    if(retry >= 5)
    {
        GTP_ERROR("[burn_fw_ss51]Alloc memory fail,exit.");
        return FAIL;
    }
    
    //step2:load ss51 firmware section 1 file data
   
//    GTP_DEBUG("[burn_fw_ss51]step2:load ss51 firmware section 1 file data");
//    ret = gup_load_section_file(fw_ss51, 0, FW_SECTION_LENGTH, SEEK_SET);
//    if(FAIL == ret)
//    {
//        GTP_ERROR("[burn_fw_ss51]load ss51 firmware section 1 fail.");
//        goto exit_burn_fw_ss51;
//    }
    
    GTP_INFO("[burn_fw_ss51]Reset first 8K of ss51 to 0xFF.");
    GTP_DEBUG("[burn_fw_ss51]step2: reset bank0 0xC000~0xD000");
    memset(fw_ss51, 0xFF, FW_SECTION_LENGTH);
    
    //step3:clear control flag
    GTP_DEBUG("[burn_fw_ss51]step3:clear control flag");
    ret = gup_set_ic_msg(client, _rRW_MISCTL__BOOT_CTL_, 0x00);
    if(ret <= 0)
    {
        GTP_ERROR("[burn_fw_ss51]clear control flag fail.");
        ret = FAIL;
        goto exit_burn_fw_ss51;
    }
    
    //step4:burn ss51 firmware section 1
    GTP_DEBUG("[burn_fw_ss51]step4:burn ss51 firmware section 1");
    ret = gup_burn_fw_section(client, fw_ss51, 0xC000, 0x01);
    if(FAIL == ret)
    {
        GTP_ERROR("[burn_fw_ss51]burn ss51 firmware section 1 fail.");
        goto exit_burn_fw_ss51;
    }
    
    //step5:load ss51 firmware section 2 file data
    GTP_DEBUG("[burn_fw_ss51]step5:load ss51 firmware section 2 file data");
    ret = gup_load_section_file(fw_ss51, FW_SECTION_LENGTH, FW_SECTION_LENGTH, SEEK_SET);
    if(FAIL == ret)
    {
        GTP_ERROR("[burn_fw_ss51]load ss51 firmware section 2 fail.");
        goto exit_burn_fw_ss51;
    }
    
    //step6:burn ss51 firmware section 2
    GTP_DEBUG("[burn_fw_ss51]step6:burn ss51 firmware section 2");
    ret = gup_burn_fw_section(client, fw_ss51, 0xE000, 0x02);
    if(FAIL == ret)
    {
        GTP_ERROR("[burn_fw_ss51]burn ss51 firmware section 2 fail.");
        goto exit_burn_fw_ss51;
    }
    
    //step7:load ss51 firmware section 3 file data
    GTP_DEBUG("[burn_fw_ss51]step7:load ss51 firmware section 3 file data");
    ret = gup_load_section_file(fw_ss51, 2 * FW_SECTION_LENGTH, FW_SECTION_LENGTH, SEEK_SET);
    if(FAIL == ret)
    {
        GTP_ERROR("[burn_fw_ss51]load ss51 firmware section 3 fail.");
        goto exit_burn_fw_ss51;
    }
    
    //step8:burn ss51 firmware section 3
    GTP_DEBUG("[burn_fw_ss51]step8:burn ss51 firmware section 3");
    ret = gup_burn_fw_section(client, fw_ss51, 0xC000, 0x13);
    if(FAIL == ret)
    {
        GTP_ERROR("[burn_fw_ss51]burn ss51 firmware section 3 fail.");
        goto exit_burn_fw_ss51;
    }
    
    //step9:load ss51 firmware section 4 file data
    GTP_DEBUG("[burn_fw_ss51]step9:load ss51 firmware section 4 file data");
    ret = gup_load_section_file(fw_ss51, 3 * FW_SECTION_LENGTH, FW_SECTION_LENGTH, SEEK_SET);
    if(FAIL == ret)
    {
        GTP_ERROR("[burn_fw_ss51]load ss51 firmware section 4 fail.");
        goto exit_burn_fw_ss51;
    }
    
    //step10:burn ss51 firmware section 4
    GTP_DEBUG("[burn_fw_ss51]step10:burn ss51 firmware section 4");
    ret = gup_burn_fw_section(client, fw_ss51, 0xE000, 0x14);
    if(FAIL == ret)
    {
        GTP_ERROR("[burn_fw_ss51]burn ss51 firmware section 4 fail.");
        goto exit_burn_fw_ss51;
    }
    
    update_msg.fw_burned_len += (FW_SECTION_LENGTH*4);
    GTP_DEBUG("[burn_fw_ss51]Burned length:%d", update_msg.fw_burned_len);
    ret = SUCCESS;
    
exit_burn_fw_ss51:
    kfree(fw_ss51);
    return ret;
}

static u8 gup_burn_fw_dsp(struct i2c_client *client)
{
    s32 ret = 0;
    u8* fw_dsp = NULL;
    u8  retry = 0;
    u8  rd_buf[5];
    
    GTP_DEBUG("[burn_fw_dsp]Begin burn dsp firmware---->>");
    //step1:alloc memory
    GTP_DEBUG("[burn_fw_dsp]step1:alloc memory");
    while(retry++ < 5)
    {
        fw_dsp = (u8*)kzalloc(FW_DSP_LENGTH, GFP_KERNEL);
        if(fw_dsp == NULL)
        {
            continue;
        }
        else
        {
            GTP_INFO("[burn_fw_dsp]Alloc %dk byte memory success.", (FW_SECTION_LENGTH/1024));
            break;
        }
    }
    if(retry >= 5)
    {
        GTP_ERROR("[burn_fw_dsp]Alloc memory fail,exit.");
        return FAIL;
    }
    
    //step2:load firmware dsp
    GTP_DEBUG("[burn_fw_dsp]step2:load firmware dsp");
    ret = gup_load_section_file(fw_dsp, 4 * FW_SECTION_LENGTH, FW_DSP_LENGTH, SEEK_SET);
    if(FAIL == ret)
    {
        GTP_ERROR("[burn_fw_dsp]load firmware dsp fail.");
        goto exit_burn_fw_dsp;
    }
    
    //step3:select bank3
    GTP_DEBUG("[burn_fw_dsp]step3:select bank3");
    ret = gup_set_ic_msg(client, _bRW_MISCTL__SRAM_BANK, 0x03);
    if(ret <= 0)
    {
        GTP_ERROR("[burn_fw_dsp]select bank3 fail.");
        ret = FAIL;
        goto exit_burn_fw_dsp;
    }
    
    //step4:hold ss51 & dsp
    GTP_DEBUG("[burn_fw_dsp]step4:hold ss51 & dsp");
    ret = gup_set_ic_msg(client, _rRW_MISCTL__SWRST_B0_, 0x0C);
    if(ret <= 0)
    {
        GTP_ERROR("[burn_fw_dsp]hold ss51 & dsp fail.");
        ret = FAIL;
        goto exit_burn_fw_dsp;
    }
    
    //step5:set scramble
    GTP_DEBUG("[burn_fw_dsp]step5:set scramble");
    ret = gup_set_ic_msg(client, _rRW_MISCTL__BOOT_OPT_B0_, 0x00);
    if(ret <= 0)
    {
        GTP_ERROR("[burn_fw_dsp]set scramble fail.");
        ret = FAIL;
        goto exit_burn_fw_dsp;
    }
    
    //step6:release ss51 & dsp
    GTP_DEBUG("[burn_fw_dsp]step6:release ss51 & dsp");
    ret = gup_set_ic_msg(client, _rRW_MISCTL__SWRST_B0_, 0x04);                 //20121211
    if(ret <= 0)
    {
        GTP_ERROR("[burn_fw_dsp]release ss51 & dsp fail.");
        ret = FAIL;
        goto exit_burn_fw_dsp;
    }
    //must delay
    msleep(1);
    
    //step7:burn 4k dsp firmware
    GTP_DEBUG("[burn_fw_dsp]step7:burn 4k dsp firmware");
    ret = gup_burn_proc(client, fw_dsp, 0x9000, FW_DSP_LENGTH);
    if(FAIL == ret)
    {
        GTP_ERROR("[burn_fw_dsp]burn fw_section fail.");
        goto exit_burn_fw_dsp;
    }
    
    //step8:send burn cmd to move data to flash from sram
    GTP_DEBUG("[burn_fw_dsp]step8:send burn cmd to move data to flash from sram");
    ret = gup_set_ic_msg(client, _rRW_MISCTL__BOOT_CTL_, 0x05);
    if(ret <= 0)
    {
        GTP_ERROR("[burn_fw_dsp]send burn cmd fail.");
        goto exit_burn_fw_dsp;
    }
    GTP_DEBUG("[burn_fw_dsp]Wait for the burn is complete......");
    do{
        ret = gup_get_ic_msg(client, _rRW_MISCTL__BOOT_CTL_, rd_buf, 1);
        if(ret <= 0)
        {
            GTP_ERROR("[burn_fw_dsp]Get burn state fail");
            goto exit_burn_fw_dsp;
        }
        msleep(10);
        //GTP_DEBUG("[burn_fw_dsp]Get burn state:%d.", rd_buf[GTP_ADDR_LENGTH]);
    }while(rd_buf[GTP_ADDR_LENGTH]);
    
    //step9:recall check 4k dsp firmware
    GTP_DEBUG("[burn_fw_dsp]step9:recall check 4k dsp firmware");
    ret = gup_recall_check(client, fw_dsp, 0x9000, FW_DSP_LENGTH);
    if(FAIL == ret)
    {
        GTP_ERROR("[burn_fw_dsp]recall check 4k dsp firmware fail.");
        goto exit_burn_fw_dsp;
    }
    
    update_msg.fw_burned_len += FW_DSP_LENGTH;
    GTP_DEBUG("[burn_fw_dsp]Burned length:%d", update_msg.fw_burned_len);
    ret = SUCCESS;
    
exit_burn_fw_dsp:
    kfree(fw_dsp);
    return ret;
}

static u8 gup_burn_fw_boot(struct i2c_client *client)
{
    s32 ret = 0;
    u8* fw_boot = NULL;
    u8  retry = 0;
    u8  rd_buf[5];
    
    GTP_DEBUG("[burn_fw_boot]Begin burn bootloader firmware---->>");
    
    //step1:Alloc memory
    GTP_DEBUG("[burn_fw_boot]step1:Alloc memory");
    while(retry++ < 5)
    {
        fw_boot = (u8*)kzalloc(FW_BOOT_LENGTH, GFP_KERNEL);
        if(fw_boot == NULL)
        {
            continue;
        }
        else
        {
            GTP_INFO("[burn_fw_boot]Alloc %dk byte memory success.", (FW_BOOT_LENGTH/1024));
            break;
        }
    }

    if (retry >= 5)
    {
        GTP_ERROR("[burn_fw_boot]Alloc memory fail,exit.");
        return FAIL;
    }

    //step2:load firmware bootloader
    GTP_DEBUG("[burn_fw_boot]step2:load firmware bootloader");
    ret = gup_load_section_file(fw_boot, (4 * FW_SECTION_LENGTH + FW_DSP_LENGTH), FW_BOOT_LENGTH, SEEK_SET);

    if (FAIL == ret)
    {
        GTP_ERROR("[burn_fw_boot]load firmware bootcode fail.");
        goto exit_burn_fw_boot;
    }

    //step3:hold ss51 & dsp
    GTP_DEBUG("[burn_fw_boot]step3:hold ss51 & dsp");
    ret = gup_set_ic_msg(client, _rRW_MISCTL__SWRST_B0_, 0x0C);

    if (ret <= 0)
    {
        GTP_ERROR("[burn_fw_boot]hold ss51 & dsp fail.");
        ret = FAIL;
        goto exit_burn_fw_boot;
    }
    
    //step4:set scramble
    GTP_DEBUG("[burn_fw_boot]step4:set scramble");
    ret = gup_set_ic_msg(client, _rRW_MISCTL__BOOT_OPT_B0_, 0x00);
    if(ret <= 0)
    {
        GTP_ERROR("[burn_fw_boot]set scramble fail.");
        ret = FAIL;
        goto exit_burn_fw_boot;
    }
    
    //step5:release ss51 & dsp
    GTP_DEBUG("[burn_fw_boot]step5:release ss51 & dsp");
    ret = gup_set_ic_msg(client, _rRW_MISCTL__SWRST_B0_, 0x04);                 //20121211
    if(ret <= 0)
    {
        GTP_ERROR("[burn_fw_boot]release ss51 & dsp fail.");
        ret = FAIL;
        goto exit_burn_fw_boot;
    }
    //must delay
    msleep(1);
    
    //step6:select bank3
    GTP_DEBUG("[burn_fw_boot]step6:select bank3");
    ret = gup_set_ic_msg(client, _bRW_MISCTL__SRAM_BANK, 0x03);
    if(ret <= 0)
    {
        GTP_ERROR("[burn_fw_boot]select bank3 fail.");
        ret = FAIL;
        goto exit_burn_fw_boot;
    }
    
    //step7:burn 2k bootloader firmware
    GTP_DEBUG("[burn_fw_boot]step6:burn 2k bootloader firmware");
    ret = gup_burn_proc(client, fw_boot, 0x9000, FW_BOOT_LENGTH);
    if(FAIL == ret)
    {
        GTP_ERROR("[burn_fw_boot]burn fw_boot fail.");
        goto exit_burn_fw_boot;
    }
    
    //step7:send burn cmd to move data to flash from sram
    GTP_DEBUG("[burn_fw_boot]step7:send burn cmd to move data to flash from sram");
    ret = gup_set_ic_msg(client, _rRW_MISCTL__BOOT_CTL_, 0x06);
    if(ret <= 0)
    {
        GTP_ERROR("[burn_fw_boot]send burn cmd fail.");
        goto exit_burn_fw_boot;
    }
    GTP_DEBUG("[burn_fw_boot]Wait for the burn is complete......");
    do{
        ret = gup_get_ic_msg(client, _rRW_MISCTL__BOOT_CTL_, rd_buf, 1);
        if(ret <= 0)
        {
            GTP_ERROR("[burn_fw_boot]Get burn state fail");
            goto exit_burn_fw_boot;
        }
        msleep(10);
        //GTP_DEBUG("[burn_fw_boot]Get burn state:%d.", rd_buf[GTP_ADDR_LENGTH]);
    }while(rd_buf[GTP_ADDR_LENGTH]);
    
    //step8:recall check 2k bootloader firmware
    GTP_DEBUG("[burn_fw_boot]step8:recall check 2k bootloader firmware");
    ret = gup_recall_check(client, fw_boot, 0x9000, FW_BOOT_LENGTH);
    if(FAIL == ret)
    {
        GTP_ERROR("[burn_fw_boot]recall check 2k bootcode firmware fail.");
        goto exit_burn_fw_boot;
    }
    
    update_msg.fw_burned_len += FW_BOOT_LENGTH;
    GTP_DEBUG("[burn_fw_boot]Burned length:%d", update_msg.fw_burned_len);
    ret = SUCCESS;
    
exit_burn_fw_boot:
    kfree(fw_boot);
    return ret;
}

/*
static void gup_show_elements(int start_index, u8 *buf, int len)
{
    
    int i = 0;
    printk("<GTP>0x%04X~0x%04X: ", start_index, start_index+len-1);
    for (i = 0; i < len; ++i)
    {
        printk("0x%02X,", buf[i]);
    }
    printk("\n");
}*/

static u8 gup_burn_fw_boot_isp(struct i2c_client *client)
{
    s32 ret = 0;
    u8* fw_boot_isp = NULL;
    u8  retry = 0;
    u8  rd_buf[5];
    
    if(update_msg.fw_burned_len >= update_msg.fw_total_len)
    {
        GTP_INFO("No need to upgrade the boot_isp code!");
        return SUCCESS;
    }
    GTP_DEBUG("[burn_fw_boot_isp]Begin burn bootloader firmware---->>");
    
    //step1:Alloc memory
    GTP_DEBUG("[burn_fw_boot_isp]step1:Alloc memory");
    while(retry++ < 5)
    {
        fw_boot_isp = (u8*)kzalloc(FW_BOOT_ISP_LENGTH, GFP_KERNEL);
        if(fw_boot_isp == NULL)
        {
            continue;
        }
        else
        {
            GTP_INFO("[burn_fw_boot]Alloc %dk byte memory success.", (FW_BOOT_ISP_LENGTH/1024));
            break;
        }
    }
    if(retry >= 5)
    {
        GTP_ERROR("[burn_fw_boot]Alloc memory fail,exit.");
        return FAIL;
    }
    
    //step2:load firmware bootloader
    GTP_DEBUG("[burn_fw_boot_isp]step2:load firmware bootloader isp");
    //ret = gup_load_section_file(fw_boot_isp, (4*FW_SECTION_LENGTH+FW_DSP_LENGTH+FW_BOOT_LENGTH+FW_DSP_ISP_LENGTH), FW_BOOT_ISP_LENGTH, SEEK_SET);
    ret = gup_load_section_file(fw_boot_isp, (update_msg.fw_burned_len - FW_DSP_ISP_LENGTH), FW_BOOT_ISP_LENGTH, SEEK_SET);
    if(FAIL == ret)
    {
        GTP_ERROR("[burn_fw_boot_isp]load firmware boot_isp fail.");
        goto exit_burn_fw_boot_isp;
    }
    
    //gup_show_elements((4*FW_SECTION_LENGTH+FW_DSP_LENGTH+FW_BOOT_LENGTH+FW_DSP_ISP_LENGTH), fw_boot_isp, 10); // for test
    //step3:hold ss51 & dsp
    GTP_DEBUG("[burn_fw_boot_isp]step3:hold ss51 & dsp");
    ret = gup_set_ic_msg(client, _rRW_MISCTL__SWRST_B0_, 0x0C);
    if(ret <= 0)
    {
        GTP_ERROR("[burn_fw_boot_isp]hold ss51 & dsp fail.");
        ret = FAIL;
        goto exit_burn_fw_boot_isp;
    }
    
    //step4:set scramble
    GTP_DEBUG("[burn_fw_boot_isp]step4:set scramble");
    ret = gup_set_ic_msg(client, _rRW_MISCTL__BOOT_OPT_B0_, 0x00);
    if(ret <= 0)
    {
        GTP_ERROR("[burn_fw_boot]set scramble fail.");
        ret = FAIL;
        goto exit_burn_fw_boot_isp;
    }
    
    
    //step5:hold ss51 & release dsp
    GTP_DEBUG("[burn_fw_boot_isp]step5:hold ss51 & release dsp");
    ret = gup_set_ic_msg(client, _rRW_MISCTL__SWRST_B0_, 0x04);                 //20121211
    if(ret <= 0)
    {
        GTP_ERROR("[burn_fw_boot]release ss51 & dsp fail.");
        ret = FAIL;
        goto exit_burn_fw_boot_isp;
    }
    //must delay
    msleep(1);
    
    //step6:select bank3
    GTP_DEBUG("[burn_fw_boot_isp]step6:select bank3");
    ret = gup_set_ic_msg(client, _bRW_MISCTL__SRAM_BANK, 0x03);
    if(ret <= 0)
    {
        GTP_ERROR("[burn_fw_boot_isp]select bank3 fail.");
        ret = FAIL;
        goto exit_burn_fw_boot_isp;
    }
    
    //step7:burn 2k bootload_isp firmware
    GTP_DEBUG("[burn_fw_boot]step7:burn 2k bootloader firmware");
    ret = gup_burn_proc(client, fw_boot_isp, 0x9000, FW_BOOT_ISP_LENGTH);
    if(FAIL == ret)
    {
        GTP_ERROR("[burn_fw_boot_isp]burn fw_section fail.");
        goto exit_burn_fw_boot_isp;
    }
    
    //step7:send burn cmd to move data to flash from sram
    GTP_DEBUG("[burn_fw_boot_isp]step7:send burn cmd to move data to flash from sram");
    ret = gup_set_ic_msg(client, _rRW_MISCTL__BOOT_CTL_, 0x07);
    if(ret <= 0)
    {
        GTP_ERROR("[burn_fw_boot]send burn cmd fail.");
        goto exit_burn_fw_boot_isp;
    }
    GTP_DEBUG("[burn_fw_boot_isp]Wait for the burn is complete......");
    do{
        ret = gup_get_ic_msg(client, _rRW_MISCTL__BOOT_CTL_, rd_buf, 1);
        if(ret <= 0)
        {
            GTP_ERROR("[burn_fw_boot]Get burn state fail");
            goto exit_burn_fw_boot_isp;
        }
        msleep(10);
        //GTP_DEBUG("[burn_fw_boot]Get burn state:%d.", rd_buf[GTP_ADDR_LENGTH]);
    }while(rd_buf[GTP_ADDR_LENGTH]);
    
    //step8:recall check 2k bootload_isp firmware
    GTP_DEBUG("[burn_fw_boot_isp]step8:recall check 2k bootloader firmware");
    ret = gup_recall_check(client, fw_boot_isp, 0x9000, FW_BOOT_ISP_LENGTH);
    if(FAIL == ret)
    {
        GTP_ERROR("[burn_fw_boot_isp]recall check 2k bootcode_isp firmware fail.");
        goto exit_burn_fw_boot_isp;
    }
    
    update_msg.fw_burned_len += FW_BOOT_ISP_LENGTH;
    GTP_DEBUG("[burn_fw_boot_isp]Burned length:%d", update_msg.fw_burned_len);
    ret = SUCCESS;
    
exit_burn_fw_boot_isp:
    kfree(fw_boot_isp);
    return ret;
}

static u8 gup_burn_fw_link(struct i2c_client *client)
{
    s32 ret = 0;
    u8* fw_link = NULL;
    u8  retry = 0;
    u32 offset;
    
    if(update_msg.fw_burned_len >= update_msg.fw_total_len)
    {
        GTP_INFO("No need to upgrade the link code!");
        return SUCCESS;
    }
    GTP_DEBUG("[burn_fw_link]Begin burn link firmware---->>");
    
    //step1:Alloc memory
    GTP_DEBUG("[burn_fw_link]step1:Alloc memory");
    while(retry++ < 5)
    {
        fw_link = (u8*)kzalloc(FW_SECTION_LENGTH, GFP_KERNEL);
        if(fw_link == NULL)
        {
            continue;
        }
        else
        {
            GTP_INFO("[burn_fw_link]Alloc %dk byte memory success.", (FW_SECTION_LENGTH/1024));
            break;
        }
    }
    if(retry >= 5)
    {
        GTP_ERROR("[burn_fw_link]Alloc memory fail,exit.");
        return FAIL;
    }
    
    //step2:load firmware link section 1
    GTP_DEBUG("[burn_fw_link]step2:load firmware link section 1");
    offset = update_msg.fw_burned_len - FW_DSP_ISP_LENGTH;
    ret = gup_load_section_file(fw_link, offset, FW_SECTION_LENGTH, SEEK_SET);
    if(FAIL == ret)
    {
        GTP_ERROR("[burn_fw_link]load firmware link section 1 fail.");
        goto exit_burn_fw_link;
    }
    
    //gup_show_elements((update_msg.fw_burned_len - FW_DSP_ISP_LENGTH), fw_link, 10); // for test
    
    //step3:burn link firmware section 1
    GTP_DEBUG("[burn_fw_link]step3:burn link firmware section 1");
    ret = gup_burn_fw_app_section(client, fw_link, 0x9000, FW_SECTION_LENGTH, 0x38);

    if (FAIL == ret)
    {
        GTP_ERROR("[burn_fw_link]burn link firmware section 1 fail.");
        goto exit_burn_fw_link;
    }
    
    //step4:load link firmware section 2 file data
    GTP_DEBUG("[burn_fw_link]step4:load link firmware section 2 file data");
    offset += FW_SECTION_LENGTH;
    ret = gup_load_section_file(fw_link, offset, FW_LINK_LENGTH - FW_SECTION_LENGTH, SEEK_SET);

    if (FAIL == ret)
    {
        GTP_ERROR("[burn_fw_link]load link firmware section 2 fail.");
        goto exit_burn_fw_link;
    }
    
    //step5:burn link firmware section 2
    GTP_DEBUG("[burn_fw_link]step4:burn link firmware section 2");
    ret = gup_burn_fw_app_section(client, fw_link, 0x9000, FW_LINK_LENGTH - FW_SECTION_LENGTH, 0x39);

    if (FAIL == ret)
    {
        GTP_ERROR("[burn_fw_link]burn link firmware section 2 fail.");
        goto exit_burn_fw_link;
    }
    
    update_msg.fw_burned_len += FW_LINK_LENGTH;
    GTP_DEBUG("[burn_fw_link]Burned length:%d", update_msg.fw_burned_len);
    ret = SUCCESS;
    
exit_burn_fw_link:
    kfree(fw_link);
    return ret;
}

static u8 gup_burn_fw_app_section(struct i2c_client *client, u8 *fw_section, u16 start_addr, u32 len, u8 bank_cmd )
{
    s32 ret = 0;
    u8  rd_buf[5];
  
    //step1:hold ss51 & dsp
    ret = gup_set_ic_msg(client, _rRW_MISCTL__SWRST_B0_, 0x0C);
    if(ret <= 0)
    {
        GTP_ERROR("[burn_fw_app_section]hold ss51 & dsp fail.");
        return FAIL;
    }
    
    //step2:set scramble
    ret = gup_set_ic_msg(client, _rRW_MISCTL__BOOT_OPT_B0_, 0x00);
    if(ret <= 0)
    {
        GTP_ERROR("[burn_fw_app_section]set scramble fail.");
        return FAIL;
    }
        
    //step3:hold ss51 & release dsp
    ret = gup_set_ic_msg(client, _rRW_MISCTL__SWRST_B0_, 0x04);
    if(ret <= 0)
    {
        GTP_ERROR("[burn_fw_app_section]hold ss51 & release dsp fail.");
        return FAIL;
    }
    //must delay
    msleep(1);
    
    //step4:select bank
    ret = gup_set_ic_msg(client, _bRW_MISCTL__SRAM_BANK, (bank_cmd >> 4)&0x0F);
    if(ret <= 0)
    {
        GTP_ERROR("[burn_fw_section]select bank %d fail.", (bank_cmd >> 4)&0x0F);
        return FAIL;
    }
    
    //step5:burn fw section
    ret = gup_burn_proc(client, fw_section, start_addr, len);
    if(FAIL == ret)
    {
        GTP_ERROR("[burn_fw_app_section]burn fw_section fail.");
        return FAIL;
    }
    
    //step6:send burn cmd to move data to flash from sram
    ret = gup_set_ic_msg(client, _rRW_MISCTL__BOOT_CTL_, bank_cmd&0x0F);
    if(ret <= 0)
    {
        GTP_ERROR("[burn_fw_app_section]send burn cmd fail.");
        return FAIL;
    }
    GTP_DEBUG("[burn_fw_section]Wait for the burn is complete......");
    do{
        ret = gup_get_ic_msg(client, _rRW_MISCTL__BOOT_CTL_, rd_buf, 1);
        if(ret <= 0)
        {
            GTP_ERROR("[burn_fw_app_section]Get burn state fail");
            return FAIL;
        }
        msleep(10);
        //GTP_DEBUG("[burn_fw_app_section]Get burn state:%d.", rd_buf[GTP_ADDR_LENGTH]);
    }while(rd_buf[GTP_ADDR_LENGTH]);
    
    //step7:recall fw section
    ret = gup_recall_check(client, fw_section, start_addr, len);
    if(FAIL == ret)
    {
        GTP_ERROR("[burn_fw_app_section]recall check %dk firmware fail.", len/1024);
        return FAIL;
    }
    
    return SUCCESS;
}

static u8 gup_burn_fw_app_code(struct i2c_client *client)
{
    u8* fw_app_code = NULL;
    u8  retry = 0;
    s32 ret = 0;
    //u16 start_index = 4*FW_SECTION_LENGTH+FW_DSP_LENGTH+FW_BOOT_LENGTH + FW_DSP_ISP_LENGTH + FW_BOOT_ISP_LENGTH; // 32 + 4 + 2 + 4 = 42K
    u16 start_index;
    
    if(update_msg.fw_burned_len >= update_msg.fw_total_len)
    {
        GTP_INFO("No need to upgrade the app code!");
        return SUCCESS;
    }
    start_index = update_msg.fw_burned_len - FW_DSP_ISP_LENGTH;
    GTP_DEBUG("[burn_fw_app_code]Begin burn app_code firmware---->>");
    
    //step1:alloc memory
    GTP_DEBUG("[burn_fw_app_code]step1:alloc memory");
    while(retry++ < 5)
    {
        fw_app_code = (u8*)kzalloc(FW_SECTION_LENGTH, GFP_KERNEL);
        if(fw_app_code == NULL)
        {
            continue;
        }
        else
        {
            GTP_INFO("[burn_fw_app_code]Alloc %dk byte memory success.", (FW_SECTION_LENGTH/1024));
            break;
        }
    }
    if(retry >= 5)
    {
        GTP_ERROR("[burn_fw_app_code]Alloc memory fail,exit.");
        return FAIL;
    }
    
    //step2:load app_code firmware section 1 file data
    GTP_DEBUG("[burn_fw_app_code]step2:load app_code firmware section 1 file data");
    ret = gup_load_section_file(fw_app_code, start_index, FW_SECTION_LENGTH, SEEK_SET);
    if(FAIL == ret)
    {
        GTP_ERROR("[burn_fw_app_code]load app_code firmware section 1 fail.");
        goto exit_burn_fw_app_code;
    }
  
    //gup_show_elements(start_index, fw_app_code, 10);   // for test
    //step3:burn app_code firmware section 1
    GTP_DEBUG("[burn_fw_app_code]step3:burn app_code firmware section 1");
    ret = gup_burn_fw_app_section(client, fw_app_code, 0x9000, FW_SECTION_LENGTH, 0x3A);
    if(FAIL == ret)
    {
        GTP_ERROR("[burn_fw_app_code]burn app_code firmware section 1 fail.");
        goto exit_burn_fw_app_code;
    }
    
    //step5:load app_code firmware section 2 file data
    GTP_DEBUG("[burn_fw_app_code]step5:load app_code firmware section 2 file data");
    ret = gup_load_section_file(fw_app_code, start_index+FW_SECTION_LENGTH, FW_SECTION_LENGTH, SEEK_SET);
    if(FAIL == ret)
    {
        GTP_ERROR("[burn_fw_app_code]load app_code firmware section 2 fail.");
        goto exit_burn_fw_app_code;
    }
    
    //gup_show_elements(start_index+FW_SECTION_LENGTH, fw_app_code, 10);   // for test
    //step6:burn app_code firmware section 2
    GTP_DEBUG("[burn_fw_app_code]step6:burn app_code firmware section 2");
    ret = gup_burn_fw_app_section(client, fw_app_code, 0x9000, FW_SECTION_LENGTH, 0x3B);
    if(FAIL == ret)
    {
        GTP_ERROR("[burn_fw_app_code]burn app_code firmware section 2 fail.");
        goto exit_burn_fw_app_code;
    }
    
    //step7:load app_code firmware section 3 file data
    GTP_DEBUG("[burn_fw_app_code]step7:load app_code firmware section 3 file data");
    ret = gup_load_section_file(fw_app_code, start_index+2*FW_SECTION_LENGTH, FW_SECTION_LENGTH, SEEK_SET);
    if(FAIL == ret)
    {
        GTP_ERROR("[burn_fw_app_code]load app_code firmware section 3 fail.");
        goto exit_burn_fw_app_code;
    }
    
    //gup_show_elements(start_index+2*FW_SECTION_LENGTH, fw_app_code, 10);   // for test
    //step8:burn app_code firmware section 3
    GTP_DEBUG("[burn_fw_app_code]step8:burn app_code firmware section 3");
    ret = gup_burn_fw_app_section(client, fw_app_code, 0x9000, FW_SECTION_LENGTH, 0x3C);
    if(FAIL == ret)
    {
        GTP_ERROR("[burn_fw_app_code]burn app_code firmware section 3 fail.");
        goto exit_burn_fw_app_code;
    }
    
    //step9:load app_code firmware section 4 file data
    GTP_DEBUG("[burn_fw_app_code]step9:load app_code firmware section 4 file data");
    ret = gup_load_section_file(fw_app_code, start_index + 3*FW_SECTION_LENGTH, FW_SECTION_LENGTH, SEEK_SET);
    if(FAIL == ret)
    {
        GTP_ERROR("[burn_fw_app_code]load app_code firmware section 4 fail.");
        goto exit_burn_fw_app_code;
    }
    
    //gup_show_elements(start_index + 3*FW_SECTION_LENGTH, fw_app_code, 10 );   // for test
    //step10:burn app_code firmware section 4
    GTP_DEBUG("[burn_fw_app_code]step10:burn app_code firmware section 4");
    ret = gup_burn_fw_app_section(client, fw_app_code, 0x9000, FW_SECTION_LENGTH, 0x3D);
    if(FAIL == ret)
    {
        GTP_ERROR("[burn_fw_app_code]burn app_code firmware section 4 fail.");
        goto exit_burn_fw_app_code;
    }
    
    update_msg.fw_burned_len += FW_APP_CODE_LENGTH;
    GTP_DEBUG("[burn_app_code]Burned length:%d", update_msg.fw_burned_len);
    ret = SUCCESS;
    
exit_burn_fw_app_code:
    kfree(fw_app_code);
    return ret;
}

static u8 gup_burn_fw_finish(struct i2c_client *client)
{
    u8* fw_ss51 = NULL;
    s32 ret = 0;
    
    GTP_INFO("[burn_fw_finish]burn first 8K of ss51 and finish update.");
    //step1:alloc memory
    GTP_DEBUG("[burn_fw_finish]step1:alloc memory");

    fw_ss51 = (u8*)kzalloc(FW_SECTION_LENGTH, GFP_KERNEL);
    if(fw_ss51 == NULL)
    {
        GTP_ERROR("[burn_fw_finish]Alloc memory fail,exit.");
        return FAIL;
    }
    
    GTP_DEBUG("[burn_fw_finish]step2: burn ss51 first 8K.");
    ret = gup_load_section_file(fw_ss51, 0, FW_SECTION_LENGTH, SEEK_SET);
    if(FAIL == ret)
    {
        GTP_ERROR("[burn_fw_finish]load ss51 firmware section 1 fail.");
        goto exit_burn_fw_finish;
    }

    GTP_DEBUG("[burn_fw_finish]step3:clear control flag");
    ret = gup_set_ic_msg(client, _rRW_MISCTL__BOOT_CTL_, 0x00);
    if(ret <= 0)
    {
        GTP_ERROR("[burn_fw_finish]clear control flag fail.");
        goto exit_burn_fw_finish;
    }
    
    GTP_DEBUG("[burn_fw_finish]step4:burn ss51 firmware section 1");
    ret = gup_burn_fw_section(client, fw_ss51, 0xC000, 0x01);
    if(FAIL == ret)
    {
        GTP_ERROR("[burn_fw_finish]burn ss51 firmware section 1 fail.");
        goto exit_burn_fw_finish;
    }
    
    //step11:enable download DSP code 
    GTP_DEBUG("[burn_app_code]step11:enable download DSP code ");
    ret = gup_set_ic_msg(client, _rRW_MISCTL__BOOT_CTL_, 0x99);
    if(ret <= 0)
    {
        GTP_ERROR("[burn_app_code]enable download DSP code fail.");
        goto exit_burn_fw_finish;
    }
    
    //step12:release ss51 & hold dsp
    GTP_DEBUG("[burn_app_code]step12:release ss51 & hold dsp");
    ret = gup_set_ic_msg(client, _rRW_MISCTL__SWRST_B0_, 0x08);
    if(ret <= 0)
    {
        GTP_ERROR("[burn_app_code]release ss51 & hold dsp fail.");
        goto exit_burn_fw_finish;
    }

    kfree(fw_ss51);
    return SUCCESS;

exit_burn_fw_finish:
    kfree(fw_ss51);
    return FAIL;
	
}

s32 gup_update_proc(void *dir)
{
    s32 ret = 0;
    u8  retry = 0;
    st_fw_head fw_head;
    struct goodix_ts_data *ts = NULL;
    
    GTP_DEBUG("[update_proc]Begin update ......");
    
    show_len = 1;
    total_len = 100;
    ts = i2c_get_clientdata(i2c_connect_client);

//    gtp_reset_guitar(i2c_connect_client, 20);
    ret = gup_get_ic_fw_msg(i2c_connect_client);
    if(FAIL == ret)
    {
        GTP_ERROR("[update_proc]get ic message fail.");
        goto update_fail;
    }    

    ret = gup_check_update_file(i2c_connect_client, &fw_head, (u8*)dir);
    if(FAIL == ret)
    {
        GTP_ERROR("[update_proc]Check *.bin file fail.");
        goto file_fail;
    }

    
    ts->enter_update = 1; 
    if (ts->use_irq)
    {       
    	gtp_irq_disable(ts);
  	}

/* begin to transplant TW GTP_ESD_PROTECT code from 8675_C00, when enter update mode, switch off esd protect by liushilong@yulong.com on 2014-11-6 11:26*/
#if GTP_ESD_PROTECT
    gtp_esd_switch(ts->client, SWITCH_OFF);
#endif
/* end to transplant TW GTP_ESD_PROTECT code from 8675_C00, when enter update mode, switch off esd protect by liushilong@yulong.com on 2014-11-6 11:26*/

    ret = gup_enter_update_mode(i2c_connect_client);
    if(FAIL == ret)
    {
         GTP_ERROR("[update_proc]enter update mode fail.");
         goto update_fail;
    }
    
    while(retry++ < 5)
    {
        show_len = 10;
        total_len = 100;
        update_msg.fw_burned_len = 0;
        ret = gup_burn_dsp_isp(i2c_connect_client);
        if(FAIL == ret)
        {
            GTP_ERROR("[update_proc]burn dsp isp fail.");
            continue;
        }
        
        show_len = 30;
        ret = gup_burn_fw_ss51(i2c_connect_client);
        if(FAIL == ret)
        {
            GTP_ERROR("[update_proc]burn ss51 firmware fail.");
            continue;
        }
        
        show_len = 40;
        ret = gup_burn_fw_dsp(i2c_connect_client);
        if(FAIL == ret)
        {
            GTP_ERROR("[update_proc]burn dsp firmware fail.");
            continue;
        }
        
        show_len = 50;
        ret = gup_burn_fw_boot(i2c_connect_client);
        if(FAIL == ret)
        {
            GTP_ERROR("[update_proc]burn bootloader firmware fail.");
            continue;
        }
        show_len = 60;
        
        ret = gup_burn_fw_boot_isp(i2c_connect_client);
        if (FAIL == ret)
        {
            GTP_ERROR("[update_proc]burn boot_isp firmware fail.");
            continue;
        }
        
        show_len = 70;
        ret = gup_burn_fw_link(i2c_connect_client);
        if (FAIL == ret)
        {
            GTP_ERROR("[update_proc]burn link firmware fail.");
            continue;
        }
        
        show_len = 80;
        ret = gup_burn_fw_app_code(i2c_connect_client);
        if (FAIL == ret)
        {
            GTP_ERROR("[update_proc]burn app_code firmware fail.");
            continue;
        }       
        show_len = 90;
        
        ret = gup_burn_fw_finish(i2c_connect_client);
        if (FAIL == ret)
        {
            GTP_ERROR("[update_proc]burn finish fail.");
            continue;
        }
        show_len = 95;
        GTP_INFO("[update_proc]UPDATE SUCCESS.");
        retry = 0;
        break;
    }
    if(retry >= 5)
    {
        GTP_ERROR("[update_proc]retry timeout,UPDATE FAIL.");
        goto update_fail;
    }
    
    GTP_DEBUG("[update_proc]leave update mode.");
    gup_leave_update_mode();
    
    msleep(100);
	if (ts->fw_error)
    {
        printk("firmware error auto update, resent config!");
        gtp_init_panel(ts);
        ts->fw_error = 0;
    }
    
    show_len = 100;
    total_len = 100;
    ts->enter_update = 0;
    if (ts->use_irq)
    {
   		gtp_irq_enable(ts);
    }

/* begin to transplant TW GTP_ESD_PROTECT code from 8675_C00, when update finish, switch on esd protect by liushilong@yulong.com on 2014-11-6 11:30 */
#if GTP_ESD_PROTECT
    gtp_esd_switch(ts->client, SWITCH_ON);
#endif
/* end to transplant TW GTP_ESD_PROTECT code from 8675_C00, when update finish, switch on esd protect by liushilong@yulong.com on 2014-11-6 11:30 */

	_CLOSE_FILE(update_msg.file);
    return SUCCESS;

update_fail:
file_fail:
    ts->enter_update = 0;
    if (ts->use_irq)
    {
   		gtp_irq_enable(ts);
    }

/* begin to transplant TW GTP_ESD_PROTECT code from 8675_C00, when file fail, switch on esd protect by liushilong@yulong.com on 2014-11-6 11:31 */
#if GTP_ESD_PROTECT
    gtp_esd_switch(ts->client, SWITCH_ON);
#endif
/* end to transplant TW GTP_ESD_PROTECT code from 8675_C00, when file fail, switch on esd protect by liushilong@yulong.com on 2014-11-6 11:31 */

	_CLOSE_FILE(update_msg.file);

    show_len = 200;
    total_len = 100;
    return FAIL;
}

u8 gup_init_update_proc(struct goodix_ts_data *ts)
{
//    struct task_struct *thread = NULL;
 //   GTP_INFO("Ready to run update thread.");

    //guitar_client = ts->client;
    
/*	
    thread = kthread_run(gup_update_proc, (void*)NULL, "guitar_update");
    if (IS_ERR(thread))
    {
        GTP_ERROR("Failed to create update thread.\n");
        return -1;
    }
*/
	return gup_update_proc(NULL);
}
