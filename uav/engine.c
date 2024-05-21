#include <stdlib.h>
#include <stdio.h>

#include <unistd.h>

#include "control/control.h"

#include "sincere/sincere.h"
#include "v2c.h"


#define V2C_SHM_NAME	"/shm-v2c-1"		/* V2C: Vision to Control */
#define ERR_THRESHOLD	5


int
main(void)
{
	ControlHandle *handle;
	SincereHandle *shm;
	V2C_Target *target;
	unsigned int error;
	
	handle = control_initialize("udp://:14540");
	if (handle == NULL) {
		printf("[ERROR] Could not initialize controller!\n");
		return 1;
	}

	if (!control_drone_takeoff(handle)) {
		exit(EXIT_FAILURE);
	};


	if (!control_drone_offboard_start(handle)) {
		exit(EXIT_FAILURE);
	};

	/* read and track vision node's output from shared memory */
	shm = sincere_shared_memory_initialize(
			V2C_SHM_NAME,
			sizeof(V2C_Target),
			true,
			false,
			false
			);

	target = sincere_shared_memory_get(shm);

	/* run loop */
	error = 0;
	while (1) {
		/* send instructions to flight controller */
		error += !control_drone_set_velocity_body(
				handle,
				target->forward_m_s,
				target->right_m_s,
				target->down_m_s,
				target->yawspeed_deg_s
				);

		/* ensure flight controller acknowledged instructions */
		if (error > ERR_THRESHOLD) {
			control_drone_land(handle);
			break;
		};
	}

	if (!control_drone_offboard_stop(handle)) {
		exit(EXIT_FAILURE);
	};

	control_finalize(handle);

	return 0;
}
