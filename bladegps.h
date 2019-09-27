#ifndef _BLADEGPS_H
#define _BLADEGPS_H

#include <stdlib.h>
#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include <time.h>
#include <math.h>
#include <vector>

#ifdef _WIN32
// To avoid conflict between time.h and pthread.h on Windows
#define HAVE_STRUCT_TIMESPEC
#endif

#include <thread>
#include <mutex>
#include <condition_variable>
//#include <pthread.h>

#include "gpssim.h"
#include "bladerf.h"


#define TX_FREQUENCY	1575420000
#define TX_SAMPLERATE	2600000
#define TX_BANDWIDTH	2500000
#define TX_VGA1			-25
#define TX_VGA2			0

#define NUM_BUFFERS			32
#define SAMPLES_PER_BUFFER	(32 * 1024)
#define NUM_TRANSFERS		16
#define TIMEOUT_MS			1000

#define NUM_IQ_SAMPLES  (TX_SAMPLERATE / 10)
#define FIFO_LENGTH     (NUM_IQ_SAMPLES * 2)

// Interactive mode directions
#define UNDEF 0
#define NORTH 1
#define SOUTH 2
#define EAST  3
#define WEST  4
#define UP    5
#define DOWN  6
// Interactive keys
#define NORTH_KEY 'w'
#define SOUTH_KEY 's'
#define EAST_KEY  'd'
#define WEST_KEY  'a'
#define UP_KEY    'e'
#define DOWN_KEY  'q'
// Interactive motion
#define MAX_VEL 2.7 // 2.77 m/s = 10 km/h
#define DEL_VEL 0.2

// 仿真设置.
struct option_t
{
	option_t()
	{
		navfile[0] = 0;
		almfile[0] = 0;
		umfile[0] = 0;
		g0.week = -1;
		g0.sec = 0.0;
		iduration = USER_MOTION_SIZE;
		verb = TRUE;
		nmeaGGA = FALSE;
		staticLocationMode = TRUE; // default user motion
		llh[0] = 40.7850916 / R2D;
		llh[1] = -73.968285 / R2D;
		llh[2] = 100.0;
		interactive = FALSE;
		timeoverwrite = FALSE;
		iono_enable = TRUE;
		path_loss_enable = TRUE;
	}

	char navfile[MAX_CHAR];
	char almfile[MAX_CHAR];
	char umfile[MAX_CHAR];
	int staticLocationMode;
	int nmeaGGA;
	int iduration;
	int verb;
	gpstime_t g0;
	double llh[3];
	int interactive;
	int timeoverwrite;
	int iono_enable;
	int path_loss_enable;
};

// 信号发射结构.
struct tx_t
{
	tx_t() 
	{
		//buffer = nullptr;
		xb_board = 0;
		txvga1 = TX_VGA1;
	}

	std::thread thread; //发送线程.
	std::mutex lock; // 发送线程互斥量.

	BladeRf dev2; // 射频设备.
	
	std::vector<int16_t> buffer;
	//int16_t* buffer; // 发送缓冲区，长度为 SAMPLES_PER_BUFFER * sizeof(int16_t) * 2

	int xb_board;
	int txvga1;
};

struct gps_t
{
	gps_t()
	{
		ready = 0;
	}

	std::thread thread; //pthread_t thread;
	std::mutex lock; //pthread_mutex_t lock;

	int ready;
	std::condition_variable initialization_done; // 初始化条件.
};

struct sim_t
{
	sim_t()
	{
		status = 0;
		head = 0;
		tail = 0;
		sample_length = 0;

		finished = false;

		//fifo = nullptr;

		time = 0.0;
	}

	option_t opt;

	tx_t tx;
	gps_t gps;

	int status;
	bool finished;

	std::vector<int16_t> fifo;
	//int16_t* fifo;
	
	long head, tail;
	size_t sample_length;

	double time;

	std::condition_variable fifo_read_ready; // 读取FIFO标志.
	std::condition_variable fifo_write_ready;// 写入FIFO标志.
};

extern void* gps_task(void* arg);
extern int is_fifo_write_ready(sim_t* s);

#endif
