#include <stdlib.h>
#include <stdio.h>

#include <unistd.h>

#include "control/control.h"

#include "sincere/sincere.h"
#include "v2c.h"


#define V2C_SHM_NAME	"/shm-v2c-1"		/* V2C: Vision to Control */
#define ERR_THRESHOLD	5

#define FLAG_RESET	-1
#define FLAG_INIT	0
#define FLAG_ACTION	1
#define FLAG_FINALIZE	2


int
main(void)
{
	ControlHandle *handle;
	SincereHandle *shm;
	V2C_Memory *v2c;
	int timestep;
	unsigned int error;
	unsigned int kill;
	
	/* read and track vision node's output from shared memory */
	shm = sincere_shared_memory_initialize(
			V2C_SHM_NAME,
			sizeof(v2c),
			false,
			true,
			true
			);

	v2c = sincere_shared_memory_get(shm);

	/* engine run-loop */
	timestep = 0;
	v2c->timestep = timestep;
	error = 0;
	kill = 0;
	while (!kill) {
		/* wait for signal */
		while (!(v2c->timestep > timestep)) {
			sleep(1);
			continue;
		}
		
		switch (v2c->flag) {
			/* receive arm and takeoff singal */
			case FLAG_INIT:
				printf("[LOG] Received initialization command.\n");

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

				break;

			case FLAG_ACTION:
				printf("[LOG] Received action command.!\n");
				printf("Forward M/S = %f\n", v2c->forward_m_s);
				printf("Right M/S = %f\n", v2c->right_m_s);
				printf("Down M/S = %f\n", v2c->down_m_s);
				printf("------------------------\n");

				/* send instructions to flight controller */
				error += !control_drone_set_velocity_body(
						handle,
						v2c->forward_m_s,
						v2c->right_m_s,
						v2c->down_m_s,
						0
						// target->yawspeed_deg_s
						);

				/* ensure flight controller acknowledged instructions */
				if (error > ERR_THRESHOLD) {
					control_drone_land(handle);
					break;
				}

				break;
			
			case FLAG_FINALIZE:
				printf("[LOG] Received finalization command.!\n");
				if (!control_drone_land(handle)) {
					exit(EXIT_FAILURE);
				}

				if (!control_drone_offboard_stop(handle)) {
					exit(EXIT_FAILURE);
				};

				control_finalize(handle);

				kill = 1;

				break;

			case FLAG_RESET:
				printf("[LOG] Received reset command.!\n");
				/* finalizing */
				if (!control_drone_land(handle)) {
					exit(EXIT_FAILURE);
				}

				if (!control_drone_offboard_stop(handle)) {
					exit(EXIT_FAILURE);
				};

				control_finalize(handle);

				/* reset from initial position */


				/* initializing */
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

				break;

		}

		timestep += 2;
		v2c->timestep += 1;
	}


	sincere_finalize(shm);

	return 0;
}
