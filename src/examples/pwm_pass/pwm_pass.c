/**
 * @file pwm_pass.c
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

__EXPORT int pwm_pass_main(int argc, char *argv[]);

int pwm_pass_main(int argc, char *argv[]) {

	struct actuator_armed_s arm;
	memset(&arm, 0, sizeof(arm));
	orb_advert_t arm_pub = orb_advertise(ORB_ID(actuator_armed), &arm);
	arm.armed = true;
	orb_publish(ORB_ID(actuator_armed), arm_pub, &arm);


	struct output_pwm_s pwm1;
	memset(&pwm1, 0, sizeof(pwm1));
	orb_advert_t pwm1_pub = orb_advertise(ORB_ID(output_pwm), &pwm1);

			pwm1.values[0] = 1500;
			pwm1.values[1] = 1500;
			pwm1.values[2] = 1500;
			pwm1.values[3] = 1500;
			pwm1.values[4] = 1500;
			pwm1.values[5] = 1500;

	PX4_INFO("PWM BEGIN");

		for(int i = 0; i < 1000000; i++) {

			/*pwm1.values[0] = 1500;
			pwm1.values[1] = 1500;
			pwm1.values[2] = 1500;
			pwm1.values[4] = 1500;
			pwm1.values[5] = 1500;
			orb_publish(ORB_ID(output_pwm), pwm1_pub, &pwm1);*/
			orb_publish(ORB_ID(output_pwm), pwm1_pub, &pwm1);

			/*PX4_INFO("\n THRO: \t%.4d\n ALIE: \t%.4d\n ELEV: \t%.4d\n RUDD: \t%.4d\n GEAR: \t%.4d\n AUX1: \t%.4d\n",
				pwm1.values[0],
				pwm1.values[1],
				pwm1.values[2],
				pwm1.values[3],
				pwm1.values[4],
				pwm1.values[5]);*/
		}

	PX4_INFO("PWM END");
	return 0;
}

