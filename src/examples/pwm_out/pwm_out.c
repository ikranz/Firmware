#include <px4_config.h>
#include <px4_tasks.h>
#include <px4_posix.h>
#include <px4_getopt.h>
#include <px4_defines.h>
#include <px4_log.h>
#include <px4_module.h>

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdbool.h>
#include <unistd.h>
#include <fcntl.h>
#include <poll.h>
#include <sys/ioctl.h>
#include <sys/stat.h>
#include <float.h>
#include <termios.h>
#include <errno.h>
#include <limits.h>
#include <math.h>

#include <uORB/uORB.h>

#include <sys/prctl.h>
#include <nuttx/sched.h>
#ifdef __PX4_NUTTX
#include <nuttx/fs/ioctl.h>
#endif

#include <arch/board/board.h>

#include <systemlib/perf_counter.h>
#include "systemlib/systemlib.h"
#include "systemlib/err.h"
#include "systemlib/param/param.h"
#include "drivers/drv_pwm_output.h"
#include <drivers/drv_hrt.h>
#include <drivers/drv_accel.h>
#include <drivers/drv_gyro.h>

__EXPORT int pwm_out_main(int argc, char *argv[]);

int pwm_out_main(int argc, char *argv[]) {

	/*Configure PWM*/
	const char *dev = PWM_OUTPUT0_DEVICE_PATH;
	//int ret;
	int fd = px4_open(dev, 0);
	int alt_rate = 400;
	int pwm_value = 1000;

	unsigned servo_count;
	px4_ioctl(fd, PWM_SERVO_GET_COUNT, (unsigned long)&servo_count);

	px4_ioctl(fd, PWM_SERVO_SET_ARM_OK, 0);

	px4_ioctl(fd, PWM_SERVO_ARM, 0);

	px4_ioctl(fd, PWM_SERVO_SET_UPDATE_RATE, alt_rate);

	/*struct pwm_output_values pwm_values;
	memset(&pwm_values, 0, sizeof(pwm_values));

	ret = px4_ioctl(fd, PWM_SERVO_GET_MIN_PWM, (long unsigned int)&pwm_values);*/

	while(1){
		for (unsigned i = 0; i < servo_count; i++) {
			px4_ioctl(fd, PWM_SERVO_SET(i), pwm_value);
		}
	}
	
	return 0;
}