#ifndef _YL_PARAMS_H_
#define _YL_PARAMS_H_

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
	char	SyncByte[10];
	char	DeviceName[32];
	char	BSP[32];
	char	ESN[32];
	char	SN[32];
	char	CommunicationModel1[32];
	char	CommunicationModel2[32];
	char	BlueModel[32];
	char	ImageSensorModel[32];
	char	WiFi[32];
	char	HardwareVersionMajor;
	char	HardwareVersionAux;
	char	Year;
	char	Month;
	char	Day;
	char	GPS[32];
	char	IMEI1[32];
	char	IMEI2[32];
	char	Module1CalStatus;
	char	Module2CalStatus;
	char	Module1RFTestStatus;
	char	Module2RFTestStatus;
	char	PCBAInformation[32];
	char	AccInfo[12];
	char	DMtag;
	char	sensor_cal_flag;
	char	RPtag;
	char	SupperPassword[8];
	char	pad[54];
};

#define TAG_LENGTH	16
#define ONE_BLOCK_SIZE	512

struct DeviceInfo {
	char 	SyncByte[16];
	char 	ParamVer[2];
	char 	Date[6];
	char 	CommunicationModel1[16];
	char 	CommunicationModel2[16];
	char 	ImageSensorModel[16];
	char	SafeboxKey[128];
	char 	pad[312];
};

struct SensorCalInfo {
	char SensorCalStatus;
	char value[7];
} __attribute__ ((packed));

struct ProductlineInfo {
	char	SyncByte[16];
	char	SN[16];
	char	IMEI1[32];
	char	IMEI2[32];
	char	ModuleCalStatus1;
	char	ModuleCalStatus2;
	char	ModuleRFTestStatus1;
	char	ModuleRFTestStatus2;
	char	ModuleCouplingTestStatus1;
	char	ModuleCouplingTestStatus2;
	char	DMtag;
	char	CameraCal;
	char	RPtag;
	char	BatteryTest;
	char	ModuleSoftVersion1[48];
	char	ModuleSoftVersion2[48];
	char	ModuleAudioVersion1[48];
	char	ModuleAudioVersion2[48];
	char	FuseBurnStatus;
	char	MiscStatus[5];
	struct	SensorCalInfo	LightProxInfo;
	struct	SensorCalInfo	AccInfo;
	struct	SensorCalInfo	PressInfo;
	struct	SensorCalInfo	SensorReserved1;
	struct	SensorCalInfo	SensorReserved2;
	struct	SensorCalInfo	SensorReserved3;
	char	DSDS_IMEI[32];
	char	pad[128];
} __attribute__ ((packed));

struct DynamicInfo {
	char	SyncByte[16];
	char	BSP[32];
	char	Password[16];
	char	SupperPassword[16];
	char	DownloadFlag[16];
	char	DownloadTool[32];
	char	SoftwareVersion[48];
	char 	DIYimageFlag[16];
	char 	SecurityFlag;
	char	CTSFlag;
	char	DRMFlag;
	char 	USBChargeFlag;
	char 	MiscFlags[12];
	char 	pad[304];
};

struct RcpInfo{
	char SyncByte[16];
	char RFlag[4];
	char RTime[4];
	char pad[488];
};

struct ReturnZeroInfo{
	char SyncByte[16];
	char AlarmTime[4];
	char AlarmAssigned;
	char USBChargerType;
	char pad[490];
};

struct MainDevInfo {
	char	name[8];
	char	Vendor[16];
	char	model[16];
};

struct ConfigurationInfo {
	char	SyncByte[16];
	char	ProductName[16];
	char	HardwareVersionMajor[6];
	char	HardwareVersionAux[6];
	char	HardwareRF_NV[6];
	struct	MainDevInfo DevInfo[11];
	char	pad[22];
};

struct CommandlineInfo {
	char	SyncByte[16];
	unsigned short crc;
	unsigned short len;
	char	data[492];
};

struct Reserve0Info {
	char	SyncByte[16];
	char	LockCode[8];
	char	Reserved[7];
	char	LockLevel;
	char	RecordVersion[32];
	char	YL_IMSI[12 * 16];
	char	Tele_IMSI[12 * 16];
	char	pad[64];
};

struct Project2_Info {
	char	SyncByte[16];
	int 	battery_check;
	char	pad[492];
};
#endif
