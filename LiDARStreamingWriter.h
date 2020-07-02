#pragma once
#include "DataStruct.h"
#include <array>
#include "cybertron/network/SocketUdpSender.hpp"
#pragma pack(push)
#pragma pack(1)

#define MAX_CHANNEL 64
#define MSOP_ID (0xA050A55A0A05AA55)
#define DIFOP_ID (0x555511115A00FFA5)
#define DIFOP_RPM 600

class LiDARStreamingWriter
{
public:
	LiDARStreamingWriter(const UnrealBridge::ConfigSensorLaserRadar& config);
	~LiDARStreamingWriter();

	bool addFrame(const std::vector<uint8_t>& pointCloudData);

private:
	bool sendDeviceInfo();

private:
	UnrealBridge::ConfigSensorLaserRadar	mLidarConfig;
	int										mLines;
	std::array<float, MAX_CHANNEL>			mVerticalAngles;
	std::array<float, MAX_CHANNEL>			mHorizontalAngles;
	cybertron::SocketUdpSender*				mpDataSender;
	cybertron::SocketUdpSender*				mpDeviceInfoSender;
};
struct PointXYZIR
{
	float x, y, z;     
	uint16_t  intensity;       
	uint16_t  ring;
};

struct MSOPHeader {
	uint64_t	mStart = { 0 };                                 //8byte 0~8
	uint32_t	mUndefined[3] = { 0 };                          //12byte 9~20
	uint8_t		mDate[6] = { 0 };                               //6byte 21~26 year¡¢month¡¢date¡¢hour minute second
	uint16_t	mTime[2] = { 0 };                               //4byte 27~30
	uint8_t		mLidarModel = 0;								//2byte 31, 01 L16  02 L32
	uint8_t		mHReserved[11] = { 0 };                        	//reserve 10 byte 32~42
};

struct MSOPChannel {
	uint16_t	mDistance = 0;
	uint8_t		mIntensity = 0;
};
struct MSOPBlock {
	uint16_t	mFlag = 0;
	uint16_t	mAngle = 0;
	MSOPChannel	mChannel[32];
};

struct MSOPPacket {
	MSOPHeader	mHeader;                        //42
	MSOPBlock	mBlock[12];                     //1200
	uint8_t		mTail[6] = { 0 };               //6
};

typedef struct
{
	uint8_t lidar_ip[4];
	uint8_t host_ip[4];
	uint8_t mac_addr[6];
	uint16_t local_port;
	uint16_t dest_port;
	uint16_t port3;
	uint16_t port4;
}DIFOPEthNet;
typedef struct
{
	uint16_t start_angle;
	uint16_t end_angle;
}DIFOPROV;
typedef struct
{
	uint8_t reserved[240];
	uint8_t coef;
	uint8_t ver;
}DIFOPIntensity;

typedef struct
{
	uint8_t year;
	uint8_t month;
	uint8_t day;
	uint8_t hour;
	uint8_t minute;
	uint8_t second;
	uint16_t ms;
	uint16_t us;
}DIFOPTimestamp;

typedef struct
{
	uint8_t top_ver[5];
	uint8_t bottom_ver[5];
}DIFOPVersion;
typedef struct
{
	uint8_t num[6];
}DIFOPSn;
typedef struct
{
	uint8_t device_current[3];
	uint8_t main_current[3];
	uint16_t vol_12v;
	uint16_t vol_12vm;
	uint16_t vol_5v;
	uint16_t vol_3v3;
	uint16_t vol_2v5;
	uint16_t vol_1v2;
}DIFOPStatus;
typedef struct
{
	uint8_t reserved1[10];
	uint8_t checksum;
	uint16_t manc_err1;
	uint16_t manc_err2;
	uint8_t gps_DIFOPStatus;
	uint16_t temperature1;
	uint16_t temperature2;
	uint16_t temperature3;
	uint16_t temperature4;
	uint16_t temperature5;
	uint8_t reserved2[5];
	uint16_t cur_rpm;
	uint8_t reserved3[7];
}DIFOPDiagno;

typedef struct
{
	uint8_t neg_flag;
	uint16_t vertical_angle;
}DIFOPVerChannelShift;

typedef struct
{
	uint8_t neg_flag;
	uint16_t Hori_angle;
}DIFOPHoriChannelShift;


typedef struct
{
	uint64_t id;
	uint16_t rpm;
	DIFOPEthNet eth;
	DIFOPROV fov;
	uint16_t reserved0;
	uint16_t phase_lock_angle;
	DIFOPVersion DIFOPVersion;
	DIFOPIntensity DIFOPIntensity;
	DIFOPSn DIFOPSn;
	uint16_t zero_cali;
	uint8_t return_mode;
	uint16_t sw_ver;
	DIFOPTimestamp DIFOPTimestamp;
	DIFOPStatus DIFOPStatus;
	uint8_t reserved1[11];
	DIFOPDiagno DIFOPDiagno;
	uint8_t gprmc[86];
	//uint8_t pitch_cali[96];
	//uint8_t yaw_cali[96];
	DIFOPVerChannelShift  pitch_cali[32];
	DIFOPHoriChannelShift yaw_cali[32];
	uint8_t reserved2[586];
	uint16_t tail;
}DIFOPPacket;

#pragma pack(pop)