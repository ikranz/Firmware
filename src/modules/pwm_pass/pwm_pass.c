/**
 * @file pwm_pass.c
 * Minimal application example for PX4 autopilot
 *
 * @author Example User <mail@example.com>
 */

#include <px4_config.h>
#include <px4_tasks.h>
#include <px4_posix.h>
#include <px4_getopt.h>
#include <px4_defines.h>
#include <px4_log.h>
#include <px4_module.h>

#include <stdlib.h>
#include <fcntl.h>
#include <unistd.h>
#include <stdio.h>
#include <poll.h>
#include <string.h>
#include <math.h>
#include <sys/ioctl.h>
#include <sys/stat.h>
#include <stdbool.h>
#include <float.h>
#include <time.h>

#ifdef __PX4_NUTTX
#include <nuttx/fs/ioctl.h>
#endif
#include <nuttx/sched.h>

#include <uORB/uORB.h>
#include <uORB/topics/sensor_combined.h>
#include <uORB/topics/vehicle_attitude.h>
#include <uORB/topics/rc_channels.h>
#include <uORB/topics/actuator_controls.h>
#include <uORB/topics/actuator_armed.h>
#include <uORB/topics/actuator_outputs.h>
#include <uORB/topics/output_pwm.h>

#include "systemlib/systemlib.h"
#include "systemlib/err.h"
#include "systemlib/param/param.h"
#include "drivers/drv_pwm_output.h"

#include <sys/prctl.h>
#include <drivers/drv_hrt.h>
#include <termios.h>
#include <errno.h>
#include <limits.h>
#include <drivers/drv_accel.h>
#include <drivers/drv_gyro.h>
#include <systemlib/perf_counter.h>
#include <systemlib/err.h>

__EXPORT int pwm_pass_main(int argc, char *argv[]);

int pwm_pass_main(int argc, char *argv[]) {

	time_t rawtime;
	struct tm * timeinfo;

	/* subscribe to sensor_combined topic */
	int sensor_sub_fd = orb_subscribe(ORB_ID(sensor_combined));
	/* limit the update rate to 5 Hz */
	orb_set_interval(sensor_sub_fd, 50);
	/* advertise attitude topic */
	struct vehicle_attitude_s att;
	memset(&att, 0, sizeof(att));

	/*ESTABLISH SERIAL CONNECTION*/
	char *uart_name = "/dev/ttyACM0";

	int serial_fd = open(uart_name, O_RDWR | O_NOCTTY);

	unsigned speed = 921600;

	if (serial_fd < 0) {
		err(1, "failed to open port: %s", uart_name);
	}

	/* Try to set baud rate */
	struct termios uart_config;
	int termios_state;

	/* Back up the original uart configuration to restore it after exit */
	if ((termios_state = tcgetattr(serial_fd, &uart_config)) < 0) {
		warnx("ERR GET CONF %s: %d\n", uart_name, termios_state);
		close(serial_fd);
		return -1;
	}

	/* Clear ONLCR flag (which appends a CR for every LF) */
	uart_config.c_oflag &= ~ONLCR;

	cfsetispeed(&uart_config, speed);
	cfsetospeed(&uart_config, speed);



	/*CONFIGURE PPM AND PWM*/
	const char *dev = PWM_OUTPUT0_DEVICE_PATH;
	int fd = px4_open(dev, 0);

	//arm servo output
	px4_ioctl(fd, PWM_SERVO_SET_ARM_OK, 0);
	px4_ioctl(fd, PWM_SERVO_ARM, 0);

	//400 Hz output
	px4_ioctl(fd, PWM_SERVO_SET_UPDATE_RATE, 400);

	/*subscribe to receiver input topic*/
	int rc_sub_fd = orb_subscribe(ORB_ID(rc_channels));

	/*update rate limit 100Hz*/
	orb_set_interval(rc_sub_fd, 50);

	/*create list of file descriptors*/
	px4_pollfd_struct_t fds[] = {
		{.fd = rc_sub_fd, .events = POLLIN},
		{ .fd = sensor_sub_fd,   .events = POLLIN }
	};

	while(1){
		/*wait 1000ms for one file descriptor change*/
		//dprintf(serial_fd, "value found");
		int poll_ret = px4_poll(fds, 1, 1000);
		int val;

		/*handle no data change*/
		if(poll_ret == 0) {
			PX4_ERR("Got no data");
		}

		/*handle data change*/
		else {

			if(fds[0].revents & fds[1].revents & POLLIN) {
				struct rc_channels_s data;
				orb_copy(ORB_ID(rc_channels), rc_sub_fd, &data);

				/* obtained data for the first file descriptor */
				struct sensor_combined_s raw;
				/* copy sensors raw data into local buffer */
				orb_copy(ORB_ID(sensor_combined), sensor_sub_fd, &raw);
				PX4_INFO("Accelerometer:\t%8.4f\t%8.4f\t%8.4f",
					 (double)raw.accelerometer_m_s2[0],
					 (double)raw.accelerometer_m_s2[1],
					 (double)raw.accelerometer_m_s2[2]);

				time (&rawtime);
					timeinfo = localtime (&rawtime);

					dprintf(serial_fd, ".rc");
				for (unsigned i = 0; i < 6; i++) {
					val = ((double)data.channels[i]+1.5) * 1000;
					dprintf(serial_fd, ".%d.", val);
				}

				dprintf(serial_fd, "rc. %s", asctime(timeinfo));

				if((double)data.channels[3] > 0.5) {

					for (unsigned i = 0; i < 6; i++) {
						//dprintf(serial_fd, "value found");
						//if(val > 2000){val = 2000;}
						px4_ioctl(fd, PWM_SERVO_SET(i), val);
					}
				}

				else {
					px4_ioctl(fd, PWM_SERVO_SET(0), 860);
				}

			}
			PX4_INFO("\n");
			
		}
		usleep(1000);
	}

	
	return 0;
}
