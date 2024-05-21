#include <cstdlib>
#include <iostream>
#include <future>
#include <chrono>
#include <thread>

#include <mavsdk/mavsdk.h>
#include <mavsdk/plugins/action/action.h>
#include <mavsdk/plugins/offboard/offboard.h>
#include <mavsdk/plugins/telemetry/telemetry.h>

#include "../control.h"


using namespace mavsdk;
using std::chrono::seconds;
using std::this_thread::sleep_for;


typedef struct ControlHandle {
	/* MAVSDK-specific */
	Mavsdk *mavsdk;
	Action *action;
	Offboard *offboard;
	Telemetry *telemetry;
} ControlHandle;


/* MAVSDK initialization */
ControlHandle *
control_initialize(const char *connection_url)
{
	/* initializing handle */
	ControlHandle *handle = (ControlHandle *)malloc(sizeof(ControlHandle));

	/* configuring ground station  */
	Mavsdk::Configuration mavsdk_config{
		Mavsdk::ComponentType::GroundStation
	};
	handle->mavsdk = new Mavsdk{mavsdk_config};

	/* connecting to device via `URL` */
	ConnectionResult connection_result = handle->mavsdk->add_any_connection(
			connection_url
			);
	if (connection_result != ConnectionResult::Success) {
		std::cerr << "[ERROR] Connection failed: " << connection_result << std::endl;
		return NULL;
	}

	/**
	 * discovering available UAV and initializing a `System` object 
	 * to control the first one found
	 */
	std::cout << "[LOG] Querying UAVs..." << std::endl;
	std::optional<std::shared_ptr<System>> sys = handle->mavsdk->first_autopilot(3.0);
	if (!sys.has_value()) {
		std::cerr << "[ERROR] Timed out waiting for system." << std::endl;
		return NULL;
	}
	std::shared_ptr<System> system = sys.value();

	/* initializing plugins */
	handle->action = new Action{system};
	handle->offboard = new Offboard{system};
	handle->telemetry = new Telemetry{system};

	/* health check */
	while (!handle->telemetry->health_all_ok()) {
		std::cout << "[LOG] Vehicle is getting ready to arm..." << std::endl;
		sleep_for(seconds(1));
	}

	/* arming */
	std::cout << "[LOG] Arming..." << std::endl;
	const Action::Result action_result = handle->action->arm();
	if (action_result != Action::Result::Success) {
		std::cerr << "[ERROR] Arming failed: " << action_result << std::endl;
		return NULL;
	}
	std::cout << "[LOG] Armed successfully!\n";

	
	return handle;
}


int
control_drone_takeoff(ControlHandle *handle)
{
	std::cout << "[LOG] Attempting takeoff..." << std::endl;
	const Action::Result takeoff_result = handle->action->takeoff();
	if (takeoff_result != Action::Result::Success) {
		std::cerr << "[ERROR] Takeoff failed: " << takeoff_result << std::endl;
		return 0;
	}
	
	sleep_for(seconds(1));



    auto in_air_promise = std::promise<void>{};
    auto in_air_future = in_air_promise.get_future();
    Telemetry::LandedStateHandle state_handle = handle->telemetry->subscribe_landed_state(
        [&](Telemetry::LandedState state) {
            if (state == Telemetry::LandedState::InAir) {
                std::cout << "Taking off has finished\n.";
                handle->telemetry->unsubscribe_landed_state(state_handle);
                in_air_promise.set_value();
            }
        });
    in_air_future.wait_for(seconds(10));
    if (in_air_future.wait_for(seconds(3)) == std::future_status::timeout) {
        std::cerr << "Takeoff timed out.\n";
        return 0;
    }


//	auto in_air_promise = std::promise<void>{};
//
//	auto in_air_future = in_air_promise.get_future();
//
//	Telemetry::LandedStateHandle state_handle = handle->telemetry->subscribe_landed_state(
//			[handle->telemetry, &in_air_promise, &state_handle](Telemetry::LandedState state) {
//				if (state == Telemetry::LandedState::InAir) {
//					std::cout << "[LOG] Taking off has finished." << std::endl;
//					telemetry.unsubscribe_landed_state(state_handle);
//					in_air_promise.set_value();
//					}
//			});
//
//	in_air_future.wait_for(seconds(10));
//	if (in_air_future.wait_for(seconds(3)) == std::future_status::timeout) {
//		std::cerr << "[ERROR] Takeoff timed out!" << std::endl;
//		return 0;
//	}

	std::cout << "[LOG] Takenoff successfully." << std::endl;

	return 1;
}


int
control_drone_land(ControlHandle *handle)
{
	const Action::Result land_result = handle->action->land();
	if (land_result != Action::Result::Success) {
		std::cerr << "[ERROR] Landing failed: " << land_result << std::endl;
		return 0;
	}
	
	sleep_for(seconds(1));

	std::cout << "[LOG] Landed successfully." << std::endl;

	return 1;
}


int
control_drone_offboard_start(ControlHandle *handle)
{
	std::cout << "[LOG] Starting Offboard velocity control in body coordinates." << std::endl;

	// Send it once before starting offboard, otherwise it will be rejected.
	Offboard::VelocityBodyYawspeed stay{};
	handle->offboard->set_velocity_body(stay);

	const Offboard::Result offboard_result = handle->offboard->start();
	if (offboard_result != Offboard::Result::Success) {
        	std::cerr << "[ERROR] Offboard start failed: " << offboard_result << std::endl;
		return 0;
	}
	
	std::cout << "[LOG] Offboard started." << std::endl;


	return 1;
}


int
control_drone_offboard_stop(ControlHandle *handle)
{
    const Offboard::Result offboard_result = handle->offboard->stop();
    if (offboard_result != Offboard::Result::Success) {
        std::cerr << "[ERROR] Offboard stop failed: " << offboard_result << std::endl;
        return 0;
    }
    std::cout << "[LOG] Offboard stopped successfully." << std::endl;
    
    return 1;
}


int
control_drone_set_velocity_body(ControlHandle *handle, 
		float forward_m_s, float right_m_s, float down_m_s, float yawspeed_deg_s)
{
	Offboard::VelocityBodyYawspeed velocity_body{};

	velocity_body.forward_m_s = forward_m_s;
	velocity_body.right_m_s = right_m_s;
	velocity_body.down_m_s = down_m_s;
	velocity_body.yawspeed_deg_s = yawspeed_deg_s;

	handle->offboard->set_velocity_body(velocity_body);

	return 1;
}

/*	
	std::cout << "Turn clock-wise and climb\n";
    Offboard::VelocityBodyYawspeed cc_and_climb{};
    cc_and_climb.down_m_s = -1.0f;
    cc_and_climb.yawspeed_deg_s = 60.0f;
    offboard.set_velocity_body(cc_and_climb);
    sleep_for(seconds(5));

    std::cout << "Turn back anti-clockwise\n";
    Offboard::VelocityBodyYawspeed ccw{};
    ccw.down_m_s = -1.0f;
    ccw.yawspeed_deg_s = -60.0f;
    offboard.set_velocity_body(ccw);
    sleep_for(seconds(5));

    std::cout << "Wait for a bit\n";
    offboard.set_velocity_body(stay);
    sleep_for(seconds(2));

    std::cout << "Fly a circle\n";
    Offboard::VelocityBodyYawspeed circle{};
    circle.forward_m_s = 5.0f;
    circle.yawspeed_deg_s = 30.0f;
    offboard.set_velocity_body(circle);
    sleep_for(seconds(15));

    std::cout << "Wait for a bit\n";
    offboard.set_velocity_body(stay);
    sleep_for(seconds(5));

    std::cout << "Fly a circle sideways\n";
    circle.right_m_s = -5.0f;
    circle.yawspeed_deg_s = 30.0f;
    offboard.set_velocity_body(circle);
    sleep_for(seconds(15));

    std::cout << "Wait for a bit\n";
    offboard.set_velocity_body(stay);
    sleep_for(seconds(8));

    offboard_result = offboard.stop();
    if (offboard_result != Offboard::Result::Success) {
        std::cerr << "Offboard stop failed: " << offboard_result << '\n';
        return false;
    }
    std::cout << "Offboard stopped\n";

*/


void
control_finalize(ControlHandle *handle)
{
	delete(handle->action);
	delete(handle->offboard);
	delete(handle->telemetry);

	delete(handle->mavsdk);

	free(handle);
}
