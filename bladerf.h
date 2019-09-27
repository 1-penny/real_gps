#pragma once

//#define USE_BLADERF

#if defined(USE_BLADERF)
#include "libbladeRF.h"
#else
struct bladerf {
	void* dummy;
};
#endif

#include <stdint.h>


// BladeRF SDR 
class BladeRf
{
public:
	BladeRf();
	virtual ~BladeRf() {}

	// 打开设备.
	int open(const char* str = nullptr);

	int config(int xb_board);

	int sync_config(unsigned int num_buffers,
		unsigned int buffer_size,
		unsigned int num_transfers,
		unsigned int stream_timeout);

	// 关闭设备.
	void close();

	// 启用模块.
	int enable(bool on);

	// 错误信息.
	static const char* strerror(int code);

	// 设置载频.
	int set_frequency(unsigned int freq);

	// 设置采样速率
	int set_sample_rate(unsigned int fs);

	int set_bandwidth(unsigned int bw);

	int set_txvga1(int txvga1);

	int set_txvga2(int txvga2);


	// 同步发射.
	int send(int16_t* buffer, int count, unsigned int timeout);

private:
	bladerf* dev;
};
