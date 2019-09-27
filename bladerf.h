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

	// ���豸.
	int open(const char* str = nullptr);

	// �����豸.
	int config(int xb_board);

	// ����ͬ�����Ͳ���.
	int sync_config(unsigned int num_buffers,
		unsigned int buffer_size,
		unsigned int num_transfers,
		unsigned int stream_timeout);

	// �ر��豸.
	void close();

	// ����ģ��.
	int enable_module(bool on);

	// ������Ϣ.
	static const char* strerror(int code);

	// ������Ƶ��Hz��
	int set_frequency(unsigned int freq);

	// ���ò������ʣ�Hz��
	int set_sample_rate(unsigned int fs);

	// ���ô���Hz��
	int set_bandwidth(unsigned int bw);

	// ���÷���VGA1
	int set_txvga1(int txvga1);

	// ���÷���VGA2
	int set_txvga2(int txvga2);
	
	// ͬ������.
	int send(int16_t* buffer, int count, unsigned int timeout);

private:
	bladerf* dev;
};
