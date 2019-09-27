#include "bladerf.h"

#include <stdio.h>
#include <thread>

BladeRf::BladeRf() : dev(nullptr)
{
}

int BladeRf::enable_module(bool on)
{
	int status = 0;

#ifdef USE_BLADERF
	status = bladerf_enable_module(dev, BLADERF_MODULE_TX, on);
#endif

	return status;
}

int BladeRf::open(const char *devstr)
{
	int status = 0;

#ifdef USE_BLADERF
	status = bladerf_open(&dev, devstr);
#endif

	return status;
}

int BladeRf::config(int xb_board)
{
	int status = 0;

#ifdef USE_BLADERF
	if (xb_board == 200)
	{
		printf("Initializing XB200 expansion board...\n");

		status = bladerf_expansion_attach(dev, BLADERF_XB_200);
		if (status != 0)
		{
			fprintf(stderr, "Failed to enable XB200: %s\n", BladeRf::strerror(status));
		}

		status = bladerf_xb200_set_filterbank(dev, BLADERF_MODULE_TX, BLADERF_XB200_CUSTOM);
		if (status != 0)
		{
			fprintf(stderr, "Failed to set XB200 TX filterbank: %s\n", BladeRf::strerror(status));
		}

		status = bladerf_xb200_set_path(dev, BLADERF_MODULE_TX, BLADERF_XB200_BYPASS);
		if (status != 0)
		{
			fprintf(stderr, "Failed to enable TX bypass path on XB200: %s\n", BladeRf::strerror(status));
		}

		//For sake of completeness set also RX path to a known good state.
		status = bladerf_xb200_set_filterbank(dev, BLADERF_MODULE_RX, BLADERF_XB200_CUSTOM);
		if (status != 0)
		{
			fprintf(stderr, "Failed to set XB200 RX filterbank: %s\n", BladeRf::strerror(status));
		}

		status = bladerf_xb200_set_path(dev, BLADERF_MODULE_RX, BLADERF_XB200_BYPASS);
		if (status != 0)
		{
			fprintf(stderr, "Failed to enable RX bypass path on XB200: %s\n", BladeRf::strerror(status));
		}
	}

	if (xb_board == 300)
	{
		fprintf(stderr, "XB300 does not support transmitting on GPS frequency\n");
		status = -1;
	}
#endif

	return status;
}

int BladeRf::sync_config(unsigned int num_buffers, unsigned int buffer_size, unsigned int num_transfers, unsigned int stream_timeout)
{
	int status = 0;

#ifdef USE_BLADERF
	status = bladerf_sync_config(
		dev, BLADERF_MODULE_TX, BLADERF_FORMAT_SC16_Q11,
		num_buffers, buffer_size, num_transfers, stream_timeout);
#endif

	return status;
}

const char *BladeRf::strerror(int code)
{
	//return bladerf_strerror(code);
	return "bladerf error";
}

void BladeRf::close()
{
#ifdef USE_BLADERF
	bladerf_close(dev);
#endif
}

int BladeRf::set_frequency(unsigned int freq)
{
	int status = 0;

#ifdef USE_BLADERF
	status = bladerf_set_frequency(dev, BLADERF_MODULE_TX, freq);
#endif

	return status;
}

int BladeRf::set_sample_rate(unsigned int fs)
{
	int status = 0;

#ifdef USE_BLADERF
	status = bladerf_set_sample_rate(dev, BLADERF_MODULE_TX, fs, NULL);
#endif

	return status;
}

int BladeRf::set_bandwidth(unsigned int bw)
{
	int status = 0;

#ifdef USE_BLADERF
	status = bladerf_set_bandwidth(dev, BLADERF_MODULE_TX, bw, NULL);
#endif

	return status;
}

int BladeRf::set_txvga1(int txvga)
{
	int status = 0;

#ifdef USE_BLADERF
	if (txvga < BLADERF_TXVGA1_GAIN_MIN)
		txvga = BLADERF_TXVGA1_GAIN_MIN;
	else if (txvga > BLADERF_TXVGA1_GAIN_MAX)
		txvga = BLADERF_TXVGA1_GAIN_MAX;
	
	status = bladerf_set_txvga1(dev, txvga);
#endif

	return status;
}

int BladeRf::set_txvga2(int txvga)
{
	int status = 0;

#ifdef USE_BLADERF
	status = bladerf_set_txvga2(dev, txvga);
#endif

	return status;
}

int BladeRf::send(int16_t *buffer, int count, unsigned int timeout)
{
	int status = 0;

#ifdef USE_BLADERF
	status = bladerf_sync_tx(dev, buffer, count, NULL, timeout);
#else
	//std::this_thread::sleep_for(std::chrono::milliseconds(8));
#endif

	return status;
}
