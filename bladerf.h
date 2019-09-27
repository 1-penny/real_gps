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

	// 配置设备.
	int config(int xb_board);

	// 配置同步发送参数.
	int sync_config(unsigned int num_buffers,
		unsigned int buffer_size,
		unsigned int num_transfers,
		unsigned int stream_timeout);

	// 关闭设备.
	void close();

	// 启用模块.
	int enable_module(bool on);

	// 错误信息.
	static const char* strerror(int code);

	// 设置载频（Hz）
	int set_frequency(unsigned int freq);

	// 设置采样速率（Hz）
	int set_sample_rate(unsigned int fs);

	// 设置带宽（Hz）
	int set_bandwidth(unsigned int bw);

	// 设置发送VGA1
	int set_txvga1(int txvga1);

	// 设置发送VGA2
	int set_txvga2(int txvga2);
	
	// 同步发射.
	int send(int16_t* buffer, int count, unsigned int timeout);

private:
	bladerf* dev;
};
