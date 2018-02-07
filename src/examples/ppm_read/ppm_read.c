/**
 * @file ppm_read.c
 * Minimal application example for PX4 autopilot
 *
 * @author Example User <mail@example.com>
 */

#include <px4_config.h>
#include <px4_tasks.h>
#include <px4_posix.h>
#include <unistd.h>
#include <stdio.h>
#include <poll.h>
#include <string.h>
#include <math.h>

#include <uORB/uORB.h>
#include <uORB/topics/sensor_combined.h>
#include <uORB/topics/vehicle_attitude.h>
#include <uORB/topics/rc_channels.h>
#include <uORB/topics/actuator_controls.h>
#include <uORB/topics/actuator_armed.h>
#include <uORB/topics/actuator_outputs.h>
#include <uORB/topics/output_pwm.h>

__EXPORT int ppm_read_main(int argc, char *argv[]);

int ppm_read_main(int argc, char *argv[])
{
	PX4_INFO("PPM READING BEGIN");

	/*subscribe to receiver input topic*/
	int rc_sub_fd = orb_subscribe(ORB_ID(rc_channels));

	/*update rate limit 5Hz*/
	orb_set_interval(rc_sub_fd, 200);

	/*advertize the pwm topic*/
	struct actuator_controls_s pwm;
	memset(&pwm, 0, sizeof(pwm));
	//orb_advert_t pwm_pub = orb_advertise(ORB_ID(actuator_controls), &pwm);

	struct actuator_armed_s arm;
	memset(&arm, 0, sizeof(arm));
	orb_advert_t arm_pub = orb_advertise(ORB_ID(actuator_armed), &arm);
	arm.armed = true;
	orb_publish(ORB_ID(actuator_armed), arm_pub, &arm);

	/*create list of file descriptors*/
	px4_pollfd_struct_t fds[] = {
		{.fd = rc_sub_fd, .events = POLLIN},
	};

	while(1){
	//for(int i = 0; i < 50; i++) {
		/*wait 1000ms for one file descriptor change*/
		int poll_ret = px4_poll(fds, 1, 1000);

		/*handle no data change*/
		if(poll_ret == 0) {
			PX4_ERR("Got no data");
		}

		/*handle data change*/
		else {

			if(fds[0].revents & POLLIN) {
				struct rc_channels_s data;
				orb_copy(ORB_ID(rc_channels), rc_sub_fd, &data);
				PX4_INFO("\n THRO: \t%.4f\n ALIE: \t%.4f\n ELEV: \t%.4f\n RUDD: \t%.4f\n GEAR: \t%.4f\n AUX1: \t%.4f\n",
					(double)data.channels[0],
					(double)data.channels[1],
					(double)data.channels[2],
					(double)data.channels[3],
					(double)data.channels[4],
					(double)data.channels[5]);
			}
		}
	}

	return 0;
}

