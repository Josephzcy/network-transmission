#include "LiDARStreamingWriter.h"
#include <cmath>
#include "cybertron/glm/function.hpp"

uint32_t reversebytes_uint32t(uint32_t value) {
	return (value & 0x000000FFU) << 24 | (value & 0x0000FF00U) << 8 |
		(value & 0x00FF0000U) >> 8 | (value & 0xFF000000U) >> 24;
}
uint16_t reversebytes_uint16t(uint16_t value) {
	return (value & 0x00FFU) << 8 | (value & 0xFF00U) >> 8;
}

LiDARStreamingWriter::LiDARStreamingWriter(const UnrealBridge::ConfigSensorLaserRadar& config)
	: mLidarConfig(config), mpDataSender(nullptr), mpDeviceInfoSender(nullptr)
{
	if (mLidarConfig.subscriptionChannel.protocol == "udp" && !mLidarConfig.subscriptionChannel.ip.empty() && mLidarConfig.subscriptionChannel.port > 0) {
		mpDataSender = new cybertron::SocketUdpSender();
		mpDataSender->initialize(mLidarConfig.subscriptionChannel.ip, mLidarConfig.subscriptionChannel.port);
	}
	if (mLidarConfig.deviceInfoChannel.protocol == "udp" && !mLidarConfig.deviceInfoChannel.ip.empty() && mLidarConfig.deviceInfoChannel.port > 0) {
		mpDeviceInfoSender = new cybertron::SocketUdpSender();
		mpDeviceInfoSender->initialize(mLidarConfig.deviceInfoChannel.ip, mLidarConfig.deviceInfoChannel.port);
	}
	mLines = mLidarConfig.channels;
	if (mLines > MAX_CHANNEL)
		mLines = MAX_CHANNEL;
	mVerticalAngles.fill(0);
	mHorizontalAngles.fill(0);
	if (!mLidarConfig.scanAngles.empty() && mLidarConfig.scanAngles.size() <= MAX_CHANNEL) {
		std::copy(mLidarConfig.scanAngles.begin(), mLidarConfig.scanAngles.end(), mVerticalAngles.begin());
	}
	else if(mLines > 1){
		float firstAngle = mLidarConfig.scanAngleUp;
		float step = (mLidarConfig.scanAngleDown - firstAngle) / (mLines - 1);
		for (int i = 0; i < mLines; i++) {
			mVerticalAngles[i] = firstAngle + step * i;
		}
	}
}

LiDARStreamingWriter::~LiDARStreamingWriter()
{
	if (mpDataSender) {
		delete mpDataSender;
		mpDataSender = nullptr;
	}
	if (mpDeviceInfoSender) {
		delete mpDeviceInfoSender;
		mpDeviceInfoSender = nullptr;
	}
}

bool LiDARStreamingWriter::addFrame(const std::vector<uint8_t>& pointCloudData)
{
	if (!mpDataSender || pointCloudData.empty()) {
		return false;
	}
	sendDeviceInfo();
	
	const PointXYZIR * pClound = reinterpret_cast<const PointXYZIR*>(pointCloudData.data());
	std::uint32_t PointsNum = (std::uint32_t)(pointCloudData.size() / sizeof(PointXYZIR));

	std::map<std::uint16_t, std::array<MSOPChannel, MAX_CHANNEL> > anglePointsMap;

	std::uint16_t azimuthResolution = std::uint16_t(mLidarConfig.horizontalResolution * 100);
	MSOPChannel channelPoint;
	static const float DistanceResolution = 0.005f;
	for (std::uint32_t i = 0; i < PointsNum; i++) {
		std::uint8_t channel = (std::uint8_t)pClound->ring;
		if (channel > MAX_CHANNEL) {
			continue;
		}
		float distance = sqrt(pow(pClound->x, 2) + pow(pClound->y, 2) + pow(pClound->z, 2));
		channelPoint.mDistance = std::uint16_t(distance / DistanceResolution);
		channelPoint.mIntensity = (std::uint8_t)pClound->intensity;
		double azimuth = -cybertron::radianToDegree(atan2(pClound->y, pClound->x));
		if (azimuth < 0) azimuth += 360;
		std::uint16_t azimuth100 = (std::uint16_t)((azimuth + mLidarConfig.horizontalResolution / 2) * 100) / azimuthResolution * azimuthResolution;
		if (anglePointsMap.find(azimuth100) == anglePointsMap.end()) {
			anglePointsMap[azimuth100] = std::array<MSOPChannel, MAX_CHANNEL>();
		}
		anglePointsMap[azimuth100][channel] = channelPoint;
		pClound++;
	}
	pClound = nullptr;

	uint16_t sizePacket = sizeof(MSOPPacket);
	MSOPPacket* pPacket = (MSOPPacket*)malloc(sizePacket);
	memset(pPacket, 0, sizePacket);
	pPacket->mHeader.mStart = MSOP_ID;
	pPacket->mHeader.mLidarModel = 0x02;
	for (uint16_t i = 0; i < 12; i++) {
		pPacket->mBlock[i].mFlag = 0xEEFF;
	}
	pPacket->mTail[5] = 0xFF;

	
	std::string sPacket;
	sPacket.resize(sizePacket);

	std::vector<std::string> messages;
	int blockIndex = 0;
	for (auto iter = anglePointsMap.begin(); iter != anglePointsMap.end(); iter++) {
		MSOPBlock* pCurrentBlock = &pPacket->mBlock[blockIndex];
		const std::uint16_t azimuth = reversebytes_uint16t(iter->first);
		for (int i = 0; i < mLines; i++) {
			MSOPChannel& channel = iter->second[i];
			int channelIndex = i % 32;
			pCurrentBlock->mAngle = azimuth;
			pCurrentBlock->mChannel[channelIndex].mDistance = reversebytes_uint16t(channel.mDistance);
			pCurrentBlock->mChannel[channelIndex].mIntensity = channel.mIntensity;
			if (channelIndex == 31 || i == mLines - 1) {
				blockIndex++;
				if (blockIndex % 12 == 0) { // packet full
					memcpy(&sPacket.at(0), pPacket, sizePacket);
					messages.push_back(sPacket);
					// reset block data.
					memset(&pPacket->mBlock[0], 0, sizeof(MSOPBlock) * 12);
					for (uint16_t i = 0; i < 12; i++) {
						pPacket->mBlock[i].mFlag = 0xEEFF;
					}
					blockIndex = 0;
				}
				pCurrentBlock = &pPacket->mBlock[blockIndex];
			}
		}
	}
	if (blockIndex > 0) {
		memcpy(&sPacket.at(0), pPacket, sizePacket);
		messages.push_back(sPacket);
	}
	free(pPacket);
	mpDataSender->send(messages);
	return true;
}

bool LiDARStreamingWriter::sendDeviceInfo()
{
	if (!mpDeviceInfoSender) {
		return false;
	}
	uint16_t sizeDIFOPPacket = sizeof(DIFOPPacket);
	DIFOPPacket* pDIFOPPacket = (DIFOPPacket*)malloc(sizeDIFOPPacket);
	memset(pDIFOPPacket, 0, sizeDIFOPPacket);
	pDIFOPPacket->id = DIFOP_ID;
	pDIFOPPacket->rpm = reversebytes_uint16t(DIFOP_RPM);
	pDIFOPPacket->return_mode = 0x01;    //Strongest
	pDIFOPPacket->DIFOPVersion.top_ver[0] = 0x06;
	pDIFOPPacket->DIFOPVersion.top_ver[1] = 0x23;
	pDIFOPPacket->DIFOPVersion.top_ver[2] = 0x06;
	pDIFOPPacket->DIFOPVersion.top_ver[3] = 0x06;
	pDIFOPPacket->DIFOPVersion.top_ver[4] = 0xa0;
	pDIFOPPacket->DIFOPVersion.bottom_ver[0] = 0x06;
	pDIFOPPacket->DIFOPVersion.bottom_ver[1] = 0x23;
	pDIFOPPacket->DIFOPVersion.bottom_ver[2] = 0x06;
	pDIFOPPacket->DIFOPVersion.bottom_ver[3] = 0x06;
	pDIFOPPacket->DIFOPVersion.bottom_ver[4] = 0xf0;
	pDIFOPPacket->DIFOPIntensity.ver = 1; // intensity view mode
	pDIFOPPacket->fov.start_angle = 0x0000;
	pDIFOPPacket->fov.end_angle = reversebytes_uint16t(0x8cA0);
	pDIFOPPacket->tail = 0xF00F;

	for (uint16_t i = 0; i < mLines; i++) {
		uint16_t index = i % 32;
		if (mVerticalAngles[index] > 0)
			pDIFOPPacket->pitch_cali[index].neg_flag = 0;
		else
			pDIFOPPacket->pitch_cali[index].neg_flag = 1;
		pDIFOPPacket->pitch_cali[index].vertical_angle = reversebytes_uint16t((uint16_t)(fabs(mVerticalAngles[index]) * 1000));
	}
	std::string sPacket;
	sPacket.resize(sizeDIFOPPacket);
	memcpy(&sPacket.at(0), pDIFOPPacket, sizeDIFOPPacket);
	free(pDIFOPPacket);
	mpDeviceInfoSender->send(sPacket);
	return true;
}
