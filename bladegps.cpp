#define _CRT_SECURE_NO_WARNINGS

#include "bladegps.h"

//haha

// for _getch used in Windows runtime.
#ifdef _WIN32
#include <conio.h>
#include "getopt.h"
#else
#include <unistd.h>
#endif

void usage(void)
{
	printf("Usage: bladegps [options]\n"
		"Options:\n"
		"  -e <gps_nav>     RINEX navigation file for GPS ephemerides (required)\n"
		"  -y <yuma_alm>    YUMA almanac file for GPS almanacs\n"
		"  -u <user_motion> User motion file (dynamic mode)\n"
		"  -g <nmea_gga>    NMEA GGA stream (dynamic mode)\n"
		"  -l <location>    Lat,Lon,Hgt (static mode) e.g. 35.274,137.014,100\n"
		"  -t <date,time>   Scenario start time YYYY/MM/DD,hh:mm:ss\n"
		"  -T <date,time>   Overwrite TOC and TOE to scenario start time\n"
		"  -d <duration>    Duration [sec] (max: %.0f)\n"
		"  -x <XB number>   Enable XB board, e.g. '-x 200' for XB200\n"
		"  -a <tx_vga1>     TX VGA1 (default: %d)\n"
		"  -i               Interactive mode: North='%c', South='%c', East='%c', West='%c'\n"
		"  -I               Disable ionospheric delay for spacecraft scenario\n"
		"  -p               Disable path loss and hold power level constant\n",
		((double)USER_MOTION_SIZE) / 10.0,
		TX_VGA1,
		NORTH_KEY, SOUTH_KEY, EAST_KEY, WEST_KEY);
}

bool init_device(sim_t& s)
{
	char* devstr = NULL;

	s.status = s.tx.dev2.open(devstr);
	if (s.status != 0) {
		//fprintf(stderr, "Failed to open device: %s\n", bladerf_strerror(s.status));
		fprintf(stderr, "Failed to open device: %s\n", BladeRf::strerror(s.status));
		return false;
	}

	s.status = s.tx.dev2.config(s.tx.xb_board);
	if (s.status != 0) {
		return false;
	}

	//s.status = bladerf_set_frequency(s.tx.dev, BLADERF_MODULE_TX, TX_FREQUENCY);
	s.status = s.tx.dev2.set_frequency(TX_FREQUENCY);
	if (s.status != 0) {
		fprintf(stderr, "Faield to set TX frequency: %s\n", BladeRf::strerror(s.status));
		return false;
	}
	else {
		printf("TX frequency: %u Hz\n", TX_FREQUENCY);
	}

	s.status = s.tx.dev2.set_sample_rate(TX_SAMPLERATE);
	if (s.status != 0) {
		fprintf(stderr, "Failed to set TX sample rate: %s\n", BladeRf::strerror(s.status));
		return false;
	}
	else {
		printf("TX sample rate: %u sps\n", TX_SAMPLERATE);
	}

	s.status = s.tx.dev2.set_bandwidth(TX_BANDWIDTH);
	if (s.status != 0) {
		fprintf(stderr, "Failed to set TX bandwidth: %s\n", BladeRf::strerror(s.status));
		return false;
	}
	else {
		printf("TX bandwidth: %u Hz\n", TX_BANDWIDTH);
	}

	//s.status = bladerf_set_txvga1(s.tx.dev, TX_VGA1);
	s.status = s.tx.dev2.set_txvga1(TX_VGA1);
	if (s.status != 0) {
		fprintf(stderr, "Failed to set TX VGA1 gain: %s\n", BladeRf::strerror(s.status));
		return false;
	}
	else {
		//printf("TX VGA1 gain: %d dB\n", TX_VGA1);
		printf("TX VGA1 gain: %d dB\n", s.tx.txvga1);
	}

	//s.status = bladerf_set_txvga2(s.tx.dev, TX_VGA2);
	s.status = s.tx.dev2.set_txvga2(TX_VGA2);
	if (s.status != 0) {
		fprintf(stderr, "Failed to set TX VGA2 gain: %s\n", BladeRf::strerror(s.status));
		return false;
	}
	else {
		printf("TX VGA2 gain: %d dB\n", TX_VGA2);
	}

	return true;
}

void init_sim(sim_t& s, int argc, char* argv[])
{
	double duration = -1;
	datetime_t t0;

	bool use_default_arg = true;
	int default_argc = 7;
	const char* default_argv[] = {
		"real_gps.exe",
		"-l", "130,30,0",
		"-e", "brdc3300.18n",
		"-d", "5",
	};

	if (!use_default_arg && argc < 3)
	{
		usage();
		exit(1);
	}

	int argc2 = use_default_arg ? default_argc : argc;
	const char** argv2 = use_default_arg ? default_argv : argv;

	int result = 0;
	while ((result = getopt(argc2, (char* const*)argv2, "e:y:u:g:l:T:t:d:x:a:iIp")) != -1)
	{
		switch (result)
		{
		case 'e':
			strcpy(s.opt.navfile, optarg);
			break;
		case 'y':
			strcpy(s.opt.almfile, optarg);
			break;
		case 'u':
			strcpy(s.opt.umfile, optarg);
			s.opt.nmeaGGA = FALSE;
			s.opt.staticLocationMode = FALSE;
			break;
		case 'g':
			strcpy(s.opt.umfile, optarg);
			s.opt.nmeaGGA = TRUE;
			s.opt.staticLocationMode = FALSE;
			break;
		case 'l':
			// Static geodetic coordinates input mode
			// Added by scateu@gmail.com
			s.opt.nmeaGGA = FALSE;
			s.opt.staticLocationMode = TRUE;
			sscanf(optarg, "%lf,%lf,%lf", &s.opt.llh[0], &s.opt.llh[1], &s.opt.llh[2]);
			s.opt.llh[0] /= R2D; // convert to RAD
			s.opt.llh[1] /= R2D; // convert to RAD
			break;
		case 'T':
			s.opt.timeoverwrite = TRUE;
			if (strncmp(optarg, "now", 3) == 0)
			{
				time_t timer;
				struct tm* gmt;

				time(&timer);
				gmt = gmtime(&timer);

				t0.y = gmt->tm_year + 1900;
				t0.m = gmt->tm_mon + 1;
				t0.d = gmt->tm_mday;
				t0.hh = gmt->tm_hour;
				t0.mm = gmt->tm_min;
				t0.sec = (double)gmt->tm_sec;

				date2gps(&t0, &s.opt.g0);

				break;
			}
		case 't':
			sscanf(optarg, "%d/%d/%d,%d:%d:%lf", &t0.y, &t0.m, &t0.d, &t0.hh, &t0.mm, &t0.sec);
			if (t0.y <= 1980 || t0.m < 1 || t0.m>12 || t0.d < 1 || t0.d>31 ||
				t0.hh < 0 || t0.hh>23 || t0.mm < 0 || t0.mm>59 || t0.sec < 0.0 || t0.sec >= 60.0)
			{
				printf("ERROR: Invalid date and time.\n");
				exit(1);
			}
			t0.sec = floor(t0.sec);
			date2gps(&t0, &s.opt.g0);
			break;
		case 'd':
			duration = atof(optarg);
			if (duration<0.0 || duration>((double)USER_MOTION_SIZE) / 10.0)
			{
				printf("ERROR: Invalid duration.\n");
				exit(1);
			}
			s.opt.iduration = (int)(duration * 10.0 + 0.5);
			break;
		case 'x':
			s.tx.xb_board = atoi(optarg);
			break;
		case 'a':
			s.tx.txvga1 = atoi(optarg);
			if (s.tx.txvga1 > 0)
				s.tx.txvga1 *= -1;
			break;
		case 'i':
			s.opt.interactive = TRUE;
			break;
		case 'I':
			s.opt.iono_enable = FALSE; // Disable ionospheric correction
			break;
		case 'p':
			s.opt.path_loss_enable = FALSE; // Disable path loss
			break;
		case ':':
		case '?':
			usage();
			exit(1);
		default:
			break;
		}
	}

	if (s.opt.navfile[0] == 0)
	{
		printf("ERROR: GPS ephemeris file is not specified.\n");
		exit(1);
	}

	if (s.opt.umfile[0] == 0 && !s.opt.staticLocationMode)
	{
		printf("ERROR: User motion file / NMEA GGA stream is not specified.\n");
		printf("You may use -l to specify the static location directly.\n");
		exit(1);
	}
}

size_t get_sample_length(sim_t* s)
{
	long length;

	length = s->head - s->tail;
	if (length < 0)
		length += FIFO_LENGTH;

	return((size_t)length);
}

size_t fifo_read(int16_t* buffer, size_t samples, sim_t* s)
{
	size_t length;
	size_t samples_remaining;
	int16_t* buffer_current = buffer;

	length = get_sample_length(s);

	if (length < samples)
		samples = length;

	length = samples; // return value

	samples_remaining = FIFO_LENGTH - s->tail;

	if (samples > samples_remaining) {
		memcpy(buffer_current, &(s->fifo[s->tail * 2]), samples_remaining * sizeof(int16_t) * 2);
		s->tail = 0;
		buffer_current += samples_remaining * 2;
		samples -= samples_remaining;
	}

	memcpy(buffer_current, &(s->fifo[s->tail * 2]), samples * sizeof(int16_t) * 2);
	s->tail += (long)samples;
	if (s->tail >= FIFO_LENGTH)
		s->tail -= FIFO_LENGTH;

	return(length);
}

bool is_finished_generation(sim_t* s)
{
	return s->finished;
}

int is_fifo_write_ready(sim_t* s)
{
	int status = 0;

	s->sample_length = get_sample_length(s);
	if (s->sample_length < NUM_IQ_SAMPLES)
		status = 1;

	return(status);
}

void* tx_task(void* arg)
{
	sim_t* s = (sim_t*)arg;
	size_t samples_populated;

	while (true) {
		int16_t* tx_buffer_current = s->tx.buffer.data();
		unsigned int buffer_samples_remaining = SAMPLES_PER_BUFFER;
		
		while (buffer_samples_remaining > 0) {
			/// 此部分代码牵涉条件同步.
			{
				std::unique_lock<std::mutex> lock(s->gps.lock);
				while (get_sample_length(s) == 0/* && !is_finished_generation(s)*/)
				{
					s->fifo_read_ready.wait(lock);
				}

				//if (is_finished_generation(s)) {
				//	break;
				//}

				samples_populated = fifo_read(
					tx_buffer_current, buffer_samples_remaining, s);

				s->fifo_write_ready.notify_all();
			}
			
#if 0
			if (is_fifo_write_ready(s)) {
				/*
				printf("\rTime = %4.1f", s->time);
				s->time += 0.1;
				fflush(stdout);
				*/
			}
			else if (is_finished_generation(s))
			{
				goto out;
			}
#endif
			// Advance the buffer pointer.
			buffer_samples_remaining -= (unsigned int)samples_populated;
			tx_buffer_current += (2 * samples_populated);
		}

		// If there were no errors, transmit the data buffer.
		s->tx.dev2.send(s->tx.buffer.data(), SAMPLES_PER_BUFFER, TIMEOUT_MS);
		//if (is_fifo_write_ready(s)) {
		//	printf("\rTime = %4.1f", s->time);
		//	s->time += 0.1;
		//	fflush(stdout);
		//}

		if (is_finished_generation(s)) {
			break;
		}
	}
out:
	printf("\n Tx Task Finish.\n");

	return NULL;
}

int start_tx_task(sim_t* s)
{
	int status = 0;

	s->tx.thread = std::thread(tx_task, s);
	
	return status;
}

int start_gps_task(sim_t* s)
{
	int status = 0;

	s->gps.thread = std::thread(gps_task, s);
	
	return status;
}

int main(int argc, char* argv[])
{
	sim_t s;
	bool result = false;

	auto t0 = std::chrono::system_clock::now();

	// Initialize simulator
	init_sim(s, argc, argv);

	// Allocate TX buffer to hold each block of samples to transmit.
	s.tx.buffer.resize(SAMPLES_PER_BUFFER * 2);

	if (s.tx.buffer.empty()) {
		fprintf(stderr, "Failed to allocate TX buffer.\n");
		return 0;
	}

	// Allocate FIFOs to hold 0.1 seconds of I/Q samples each.
	s.fifo.resize(FIFO_LENGTH * 2);// for 16-bit I and Q samples
	if (s.fifo.empty()) {
		fprintf(stderr, "Failed to allocate I/Q sample buffer.\n");
		return 0;
	}

	// Initializing device.
	printf("Opening and initializing device...\n");

	result = init_device(s);
	if (!result) {
		return 0;
	}

	// Start GPS task.
	s.status = start_gps_task(&s);
	if (s.status < 0) {
		fprintf(stderr, "Failed to start GPS task.\n");
		return 0;
	}
	else
		printf("Creating GPS task...\n");

	// Wait until GPS task is initialized
	std::unique_lock<std::mutex> lock(s.tx.lock);
	while (!s.gps.ready) {
		s.gps.initialization_done.wait(lock);
	}

	// Fillfull the FIFO.
	if (is_fifo_write_ready(&s)) {
		s.fifo_write_ready.notify_all();
	}

	// Configure the TX module for use with the synchronous interface.
	s.status = s.tx.dev2.sync_config(
		NUM_BUFFERS, SAMPLES_PER_BUFFER, NUM_TRANSFERS, TIMEOUT_MS);
	if (s.status != 0) {
		fprintf(stderr, "Failed to configure TX sync interface: %s\n", BladeRf::strerror(s.status));
		return 0;
	}

	// We must always enable the modules *after* calling bladerf_sync_config().
	s.status = s.tx.dev2.enable_module(true);
	if (s.status != 0) {
		fprintf(stderr, "Failed to enable TX module: %s\n", BladeRf::strerror(s.status));
		return 0;
	}

	// Start TX task
	s.status = start_tx_task(&s);
	if (s.status < 0) {
		fprintf(stderr, "Failed to start TX task.\n");
		return 0;
	}
	else
		printf("Creating TX task...\n");

	// Running...
	printf("Running...\n");
	printf("Press 'q' to quit.\n");



	// Wainting for TX task to complete.
	s.tx.thread.join();
	s.gps.thread.join();

	printf("\nDone!\n");

	// print run time.
	auto td = std::chrono::duration_cast<std::chrono::milliseconds>(
		std::chrono::system_clock::now() - t0);
	printf("\n time used %d s\n", td.count());

	// Disable TX module and shut down underlying TX stream.
	s.status = s.tx.dev2.enable_module(false);
	if (s.status != 0)
		fprintf(stderr, "Failed to disable TX module: %s\n", BladeRf::strerror(s.status));

out:
	// Free up resources

	printf("Closing device...\n");

	s.tx.dev2.close();

	return(0);
}

