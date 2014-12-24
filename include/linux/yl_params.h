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

#ifndef _YL_PARAMS_H_
#define _YL_PARAMS_H_

#include <linux/types.h>

enum yl_params_index {
	YL_DEVICE = 0,
	YL_CONFIGURATION,
	YL_PRODUCTLINE,
	YL_DYNAMIC,
	YL_GUARD,
	YL_CMDLINE,
	YL_TOUCHSCREEN0,
	YL_TOUCHSCREEN1,
	YL_TOUCHSCREEN2,
	YL_TOUCHSCREEN3,
	YL_RESERVE0,
	YL_RESERVE1,
	YL_PROJECT0,
	YL_PROJECT1,
	YL_PROJECT2,
	YL_PROJECT3,
	YL_FETCH_PASSWD,
	YL_FCT_DIAG,
	YL_RCP,
	YL_RETURNZERO,
	YL_PARAMS_COUNT,
};

enum yl_params_index_v0 {
	YL_OBSOLETE_DEVICE = 0,
	YL_OBSOLETE_WLAN,
	YL_OBSOLETE_ACCTW_CAL,
	YL_OBSOLETE_DOWNFLAG,
	YL_OBSOLETE_GUARD,
	YL_OBSOLETE_odm_lckc,
	YL_OBSOLETE_odm_lckl,
	YL_OBSOLETE_ALM_TIME,
	YL_OBSOLETE_mms_imsi,
	YL_OBSOLETE_CAM_CAL,
	YL_OBSOLETE_AUTO_REG,
	YL_OBSOLETE_VOL_TAB,
	YL_OBSOLETE_PASSWRD,
	YL_OBSOLETE_MMS_VER,
	YL_OBSOLETE_ALM_POFF,
	YL_OBSOLETE_RESERVED,
};

struct DeviceInfo_v0 {
	uint8_t SyncByte[10];
	uint8_t DeviceName[32];
	uint8_t BSP[32];
	uint8_t ESN[32];
	uint8_t SN[32];
	uint8_t CommunicationModel1[32];
	uint8_t CommunicationModel2[32];
	uint8_t BlueModel[32];
	uint8_t ImageSensorModel[32];
	uint8_t WiFi[32];
	uint8_t HardwareVersionMajor;
	uint8_t HardwareVersionAux;
	uint8_t Year;
	uint8_t Month;
	uint8_t Day;
	uint8_t GPS[32];
	uint8_t IMEI1[32];
	uint8_t IMEI2[32];
	uint8_t Module1CalStatus;
	uint8_t Module2CalStatus;
	uint8_t Module1RFTestStatus;
	uint8_t Module2RFTestStatus;
	uint8_t PCBAInformation[32];
	uint8_t AccInfo[12];
	uint8_t DMtag;
	uint8_t sensor_cal_flag;
	uint8_t RPtag;
	uint8_t SupperPassword[8];
	uint8_t pad[54];
};

struct DeviceInfo {
	uint8_t SyncByte[16];
	uint8_t ParamVer[2];
	uint8_t Date[6];
	uint8_t CommunicationModel1[16];
	uint8_t CommunicationModel2[16];
	uint8_t ImageSensorModel[16];
	uint8_t SafeboxKey[128];
	uint8_t pad1[128];
	uint8_t Sim1Capacity[2];
	uint8_t Sim2Capacity[2];
	uint8_t Sim3Capacity[2];
	uint8_t NET_CARRIER;
	uint8_t SimSlots;
	uint8_t pad[176];
};

struct SensorCalInfo {
	uint8_t SensorCalStatus;
	uint8_t value[7];
} __attribute__ ((packed));

struct ProductlineInfo {
	uint8_t SyncByte[16];
	uint8_t SN[16];
	uint8_t IMEI1[32];
	uint8_t IMEI2[32];
	uint8_t ModuleCalStatus1;
	uint8_t ModuleCalStatus2;
	uint8_t ModuleRFTestStatus1;
	uint8_t ModuleRFTestStatus2;
	uint8_t ModuleCouplingTestStatus1;
	uint8_t ModuleCouplingTestStatus2;
	uint8_t DMtag;
	uint8_t CameraCal;
	uint8_t RPtag;
	uint8_t BatteryTest;
	uint8_t ModuleSoftVersion1[48];
	uint8_t ModuleSoftVersion2[48];
	uint8_t ModuleAudioVersion1[48];
	uint8_t ModuleAudioVersion2[48];
	uint8_t FuseBurnStatus;
	uint8_t MiscStatus[5];
	uint8_t LightProxInfo[8];
	uint8_t AccInfo[8];
	uint8_t PressInfo[8];
	uint8_t SensorReserved1[8];
	uint8_t SensorReserved2[8];
	uint8_t SensorReserved3[8];
	uint8_t DSDS_IMEI[32];
	uint8_t WIFI_MAC[6];
	uint8_t BT_MAC[6];
	uint8_t pad[112];
};

struct DynamicInfo {
	uint8_t SyncByte[16];
	uint8_t BSP[32];
	uint8_t Password[16];
	uint8_t SupperPassword[16];
	uint8_t DownloadFlag[16];
	uint8_t DownloadTool[32];
	uint8_t SoftwareVersion[48];
	uint8_t DIYimageFlag[16];
	uint8_t SecurityFlag;
	uint8_t CTSFlag;
	uint8_t DRMFlag;
	uint8_t USBChargeFlag;
	uint8_t LTEState;
	uint8_t MultiBootloader;
	uint8_t GMSDownload;
	uint8_t CPBDownload;
	uint8_t MiscFlags[56];
	uint8_t Virgin[16];
	uint8_t NfcUnlockScreenKey[32];
	uint8_t pad[208];
};

struct MainDevInfo {
	uint8_t name[8];
	uint8_t Vendor[16];
	uint8_t model[16];
};

struct ConfigurationInfo {
	uint8_t SyncByte[16];
	uint8_t ProductName[16];
	uint8_t HardwareVersionMajor[6];
	uint8_t HardwareVersionAux[6];
	uint8_t HardwareRF_NV[6];
	struct MainDevInfo DevInfo[11];
	uint8_t pad[22];
};

struct CommandlineInfo {
	uint8_t SyncByte[16];
	uint16_t crc;
	uint16_t len;
	uint8_t data[492];
};

struct Reserve0Info {
	uint8_t SyncByte[16];
	uint8_t LockCode[8];
	uint8_t Reserved[7];
	uint8_t LockLevel;
	uint8_t RecordVersion[32];
	uint8_t YL_IMSI[12 * 16];
	uint8_t Tele_IMSI[12 * 16];
	uint8_t pad[64];
};

struct FctDiagInfo {
	uint8_t SyncByte[16];
	uint8_t todo[496];
};

struct RcpInfo {
	uint8_t SyncByte[16];
	uint8_t RFlag[4];
	uint8_t RTime[4];
	uint8_t AuthCode[260];
	uint8_t EncryptoCode[116];
	uint8_t CoolyunID[8];
	uint8_t CoolyunPassword[32];
	uint8_t LogStatus[1];
	uint8_t pad[71];
};

struct ReturnZeroInfo {
	uint8_t SyncByte[16];
	uint8_t AlarmTime[4];
	uint8_t AlarmAssigned;
	uint8_t USBChargerType;
	uint8_t BootNoVib;
	uint8_t res[1];
	uint8_t CommRunMode[4];
	uint8_t pad[484];
};

#define TAG_LENGTH			16
#define ONE_BLOCK_SIZE			512

#define RETURNZERO_ALARM_TIME		16
#define RETURNZERO_ALARM_ASSIGNED	20
#define RETURNZERO_USB_CHARGER_TYPE	21
#define RETURNZERO_BOOT_NO_VIB		22
#define RETURNZERO_COMM_RUN_MODE	

ssize_t yl_params_kernel_write(const uint8_t *buf, ssize_t count);
ssize_t yl_params_kernel_read(uint8_t *buf, ssize_t count);

#endif /* _YL_PARAMS_H_ */
