#ifdef __cplusplus
extern "C" {
#endif

typedef struct ControlHandle ControlHandle;

//int control_drone_position

ControlHandle *control_initialize(const char *connection_url);
void control_finalize(ControlHandle *handle);

int control_drone_takeoff(ControlHandle *handle);
int control_drone_land(ControlHandle *handle);

int control_drone_offboard_start(ControlHandle *handle);
int control_drone_offboard_stop(ControlHandle *handle);

int control_drone_set_velocity_body(ControlHandle *handle, 
		float forward_m_s, float right_m_s, float down_m_s, float yawspeed_deg_s);

#ifdef __cplusplus
}
#endif
