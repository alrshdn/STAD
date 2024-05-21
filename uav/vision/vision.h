typedef struct {
	double x;
	double y;
} Offset;

typedef struct {
	double relative_altitude;
	double absolute_altitude;
} Depth;

Offset vision_get_offset(void);
Depth vision_get_depth(void);
double vision_get_yaw(void);
