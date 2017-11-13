/****************************************************************************
 *
 *   Copyright (c) 2016 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/**
 * @file jeilin_flow.cpp
 * @author Roman Bapst <roman@uaventure.com>
 *
 * Driver for the jeilin optical flow from Aerotenna
 */

#include <termios.h>

#include <drivers/drv_hrt.h>
#include <drivers/device/device.h>
#include <uORB/topics/optical_flow.h>

#define FLOW_DEFAULT_PORT		"/dev/ttyS6"	// designated SERIAL4/5 on Pixhawk
#define BUF_LEN 		60//20*3

#define KM	0.175
#define DT 	0.02    //50fps


extern "C" __EXPORT int jeilin_flow_main(int argc, char *argv[]);

class JEILIN_FLOW
{
public:
	JEILIN_FLOW(const char *port = FLOW_DEFAULT_PORT);
	virtual ~JEILIN_FLOW();

	virtual int 			init();

	int				start();

private:
	bool				_task_should_exit;
	int 				_task_handle;
	char 				_port[20];
	int				_class_instance;
	int				_orb_class_instance;
	orb_advert_t			_jeilin_flow_topic;

	unsigned 	_head;
	unsigned 	_tail;
	uint8_t 	_buf[BUF_LEN];

	static void task_main_trampoline(int argc, char *argv[]);
	void task_main();

	bool read_and_parse(uint8_t *buf, int len, float *range);
};

namespace jeilin_flow
{
JEILIN_FLOW	*g_dev;
}

JEILIN_FLOW::JEILIN_FLOW(const char *port) :
	_task_should_exit(false),
	_task_handle(-1),
	_class_instance(-1),
	_orb_class_instance(-1),
	_jeilin_flow_topic(nullptr),
	_head(0),
	_tail(0)
{
	/* store port name */
	strncpy(_port, port, sizeof(_port));
	/* enforce null termination */
	_port[sizeof(_port) - 1] = '\0';

	// disable debug() calls
	//_debug_enabled = false;

	memset(&_buf[0], 0, sizeof(_buf));
}

JEILIN_FLOW::~JEILIN_FLOW()
{
	if (_task_handle != -1) {
		/* task wakes up every 100ms or so at the longest */
		_task_should_exit = true;

		/* wait for a second for the task to quit at our request */
		unsigned i = 0;

		do {
			/* wait 20ms */
			usleep(20000);

			/* if we have given up, kill it */
			if (++i > 50) {
				px4_task_delete(_task_handle);
				break;
			}
		} while (_task_handle != -1);
	}

	orb_unadvertise(_jeilin_flow_topic);
}

int
JEILIN_FLOW::init()
{
	/* status */
	int ret = 0;

	do { /* create a scope to handle exit conditions using break */

		/* open fd */
		int fd = px4_open(_port, O_RDWR | O_NOCTTY);

		if (fd < 0) {
			PX4_WARN("failed to open serial device");
			ret = 1;
			break;
		}

		struct termios uart_config;

		int termios_state;

		/* fill the struct for the new configuration */
		tcgetattr(fd, &uart_config);

		/* clear ONLCR flag (which appends a CR for every LF) */
		uart_config.c_oflag &= ~ONLCR;

		/* no parity, one stop bit */
		uart_config.c_cflag	 &= ~(CSTOPB | PARENB);

		unsigned speed = B115200;

		/* set baud rate */
		if ((termios_state = cfsetispeed(&uart_config, speed)) < 0) {
			PX4_WARN("ERR CFG: %d ISPD", termios_state);
			ret = 1;
			break;
		}

		if ((termios_state = cfsetospeed(&uart_config, speed)) < 0) {
			PX4_WARN("ERR CFG: %d OSPD\n", termios_state);
			ret = 1;
			break;
		}

		if ((termios_state = tcsetattr(fd, TCSANOW, &uart_config)) < 0) {
			PX4_WARN("ERR baud %d ATTR", termios_state);
			ret = 1;
			break;
		}

		px4_close(fd);

	} while (0);

	return ret;
}

void
JEILIN_FLOW::task_main_trampoline(int argc, char *argv[])
{
	jeilin_flow::g_dev->task_main();
}

int
JEILIN_FLOW::start()
{
	ASSERT(_task_handle == -1);

	/* start the task */
	_task_handle = px4_task_spawn_cmd("jeilin_flow",
					  SCHED_DEFAULT,
					  SCHED_PRIORITY_MAX - 30,
					  800,
					  (px4_main_t)&JEILIN_FLOW::task_main_trampoline,
					  nullptr);

	if (_task_handle < 0) {
		PX4_WARN("task start failed");
		return -errno;
	}

	return OK;
}

bool JEILIN_FLOW::read_and_parse(uint8_t *buf, int len, float *range)
{
	bool ret = false;

	// write new data into a ring buffer
	for (unsigned i = 0; i < len; i++) {

		_head++;

		if (_head >= BUF_LEN) {
			_head = 0;
		}

		if (_tail == _head) {
			_tail = (_tail == BUF_LEN - 1) ? 0 : _head + 1;
		}

		_buf[_head] = buf[i];
	}

	// check how many bytes are in the buffer, return if it's lower than the size of one package
	int num_bytes = _head >= _tail ? (_head - _tail + 1) : (_head + 1 + BUF_LEN - _tail);

	if (num_bytes < 20) {
		return false;
	}

	int index = _head;
	uint8_t no_header_counter = 0;	// counter for bytes which are non header bytes

	// go through the buffer backwards starting from the newest byte
	// if we find a header byte and the previous two bytes weren't header bytes
	// then we found the newest package.
	for (unsigned i = 0; i < num_bytes; i++) {
		if ((_buf[index]==0x23)&&(_buf[index+1]==0x4A)&&(_buf[index+2]==0x42)&&(_buf[index+3]==0x23)) {
			if (no_header_counter >= 19) {

				struct optical_flow_s report;

				report.timestamp = hrt_absolute_time();

				//X-Offset-cm = X-Offset count*kM*Height
				//Y-Offset-cm = Y-Offset count*kM*Height
				report.pixel_flow_x_integral = ((int8_t)_buf[index+6]) * KM / DT / 100;//0.05f; //convert to radians
				report.pixel_flow_y_integral = ((int8_t)_buf[index+7]) * KM / DT / 100;//0.05f; //convert to radians
				//report.frame_count_since_last_readout = 0.0f;
				//report.ground_distance_m = 0.0f;//convert to meters
				report.quality = _buf[index+5]; //0:bad ; 255 max quality
				//report.gyro_x_rate_integral = 0.0f; //convert to radians
				//report.gyro_y_rate_integral = 0.0f; //convert to radians
				//report.gyro_z_rate_integral = 0.0f; //convert to radians
				//report.integration_timespan = static_cast<float>(_buf[index+13]) / 1.0f; //microseconds
				//report.time_since_last_sonar_update = 0.0f;//microseconds
				//report.gyro_temperature = 0.0f;//Temperature * 100 in centi-degrees Celsius
				report.sensor_id = 0;

				if (_jeilin_flow_topic == nullptr) {
					_jeilin_flow_topic = orb_advertise(ORB_ID(optical_flow), &report);

				} else {
					/* publish it */
					orb_publish(ORB_ID(optical_flow), _jeilin_flow_topic, &report);
				}

				// set the tail to one after the index because we neglect
				// any data before the one we just read
				_tail = index == BUF_LEN - 1 ? 0 : index++;
				ret = true;
				break;

			}

			no_header_counter = 0;

		} else {
			no_header_counter++;
		}

		index--;

		if (index < 0) {
			index = BUF_LEN - 1;
		}

	}

	return ret;
}

void
JEILIN_FLOW::task_main()
{

	int fd = px4_open(_port, O_RDWR | O_NOCTTY);

	// we poll on data from the serial port
	px4_pollfd_struct_t fds[1];
	fds[0].fd = fd;
	fds[0].events = POLLIN;

	// read buffer, one measurement consists of three bytes
	uint8_t buf[BUF_LEN];

	while (!_task_should_exit) {
		// wait for up to 100ms for data
		int pret = px4_poll(&fds[0], (sizeof(fds) / sizeof(fds[0])), 100);

		// timed out
		if (pret == 0) {
			continue;
		}

		if (pret < 0) {
			PX4_DEBUG("Jeilin flow serial port poll error");
			// sleep a bit before next try
			usleep(100000);
			continue;
		}

		if (fds[0].revents & POLLIN) {
			memset(&buf[0], 0, sizeof(buf));
			int len = px4_read(fd, &buf[0], sizeof(buf));

			if (len <= 0) {
				PX4_DEBUG("error reading jeilin flow");
			}

			float range = 0;

			if (read_and_parse(&buf[0], len, &range)) {
				
			}

		}
	}

	px4_close(fd);
}

int jeilin_flow_main(int argc, char *argv[])
{
	/*
	 * Start/load the driver.
	 */
	if (!strcmp(argv[1], "start")) {
		if (jeilin_flow::g_dev != nullptr) {
			PX4_WARN("driver already started");
			return 0;
		}

		if (argc > 2) {
			jeilin_flow::g_dev = new JEILIN_FLOW(argv[2]);

		} else {
			jeilin_flow::g_dev = new JEILIN_FLOW(FLOW_DEFAULT_PORT);
		}

		if (jeilin_flow::g_dev == nullptr) {
			PX4_ERR("failed to create instance of JEILIN_FLOW");
			return 1;
		}

		if (PX4_OK != jeilin_flow::g_dev->init()) {
			delete jeilin_flow::g_dev;
			jeilin_flow::g_dev = nullptr;
			return 1;
		}

		if (OK != jeilin_flow::g_dev->start()) {
			delete jeilin_flow::g_dev;
			jeilin_flow::g_dev = nullptr;
			return 1;
		}

		return 0;
	}

	/*
	 * Stop the driver
	 */
	if (!strcmp(argv[1], "stop")) {
		if (jeilin_flow::g_dev != nullptr) {
			delete jeilin_flow::g_dev;
			jeilin_flow::g_dev = nullptr;

		} else {
			PX4_WARN("driver not running");
		}

		return 0;
	}

	if (!strcmp(argv[1], "info")) {
		PX4_INFO("Jeilin flow from Aerotenna");
		//PX4_INFO("min distance %.2f m", (double)JEILIN_FLOW_MIN_DISTANCE);
		//PX4_INFO("max distance %.2f m", (double)JEILIN_FLOW_MAX_DISTANCE);
		PX4_INFO("update rate: 500 Hz");
		return 0;

	}

	PX4_WARN("unrecognized arguments, try: start [device_name], stop, info ");
	return 1;
}
