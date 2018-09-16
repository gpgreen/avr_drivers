#ifndef UM6_H_
#define UM6_H_

#include "defs.h"
#include <inttypes.h>

// define to 1 if debugging
/* #define UM6DEBUG   (1) */
// define to 1 for SPI debugging
/* #define UM6SPIDEBUG (1) */

// return error codes
#define UM6_OK      (0)
#define UM6_FAIL    (1)

// define some pins
//#define PORT_UM6SS PORTD
//#define P_UM6SS 0

// define baud rates
#define UM6_9600 0x0
#define UM6_14400 0x1
#define UM6_19200 0x2
#define UM6_38400 0x3
#define UM6_57600 0x4
#define UM6_115200 0x5

// we need to define the MCP clock
// this is not the same as the CPU
// define one of these in the board specific firmware
/* #define UM6_8MHZ    (1) */
/* #define UM6_16MHZ   (1) */

struct um6_dev_config
{
	// contents of UM6_COMMUNICATION register
	uint8_t comm[4];
	// contents of UM6_MISC_CONFIG register
	uint8_t misc;
};

struct um6_dev_version
{
	// version
	char fw_version[4];
};

struct um6_dev_att_data
{
	float quaternion_est[4];
};

struct um6_dev_euler_att_data
{
    float att[3]; /* pitch, roll, yaw */
};

struct um6_dev_mag_data
{
	float mag_vec[3];
};

struct um6_dev_acc_data
{
	float acc_vec[3];
};

struct um6_dev_gyro_data
{
	float gyro_vec[3];
};

extern int um6_broadcast_rate(const struct um6_dev_config *config);
extern void um6_set_broadcast_rate(struct um6_dev_config *config, int rate);
extern int um6_baud_rate(const struct um6_dev_config *config);
extern void um6_set_baud_rate(struct um6_dev_config *config, int rate);
extern int um6_gps_baud_rate(const struct um6_dev_config *config);
extern void um6_set_gps_baud_rate(struct um6_dev_config *config, int rate);
extern void um6_gps_config(const struct um6_dev_config *config,
						   int *sat_status_en,
						   int *sat_summary_en,
						   int *gps_velocity_en,
						   int *gps_rel_position_en,
						   int *gps_position_en);
extern void um6_set_gps_config(struct um6_dev_config *config,
							   int sat_status_en,
							   int sat_summary_en,
							   int gps_velocity_en,
							   int gps_rel_position_en,
							   int gps_position_en);
extern void um6_kalman_config(const struct um6_dev_config *config,
							  int *temperature_en,
							  int *covariance_en,
							  int *euler_en,
							  int *quaternion_en);
extern void um6_set_kalman_config(struct um6_dev_config *config,
								  int temperature_en,
								  int covariance_en,
								  int euler_en,
								  int quaternion_en);
extern void um6_output_config(const struct um6_dev_config *config,
							  int *proc_mag_en,
							  int *proc_acc_en,
							  int *proc_gyro_en,
							  int *raw_mag_en,
							  int *raw_acc_en,
							  int *raw_gyro_en,
							  int *broadcast_en);
extern void um6_set_output_config(struct um6_dev_config *config,
								  int proc_mag_en,
								  int proc_acc_en,
								  int proc_gyro_en,
								  int raw_mag_en,
								  int raw_acc_en,
								  int raw_gyro_en,
								  int broadcast_en);
extern void um6_misc_config(const struct um6_dev_config *config,
							int *pps_en,
							int *quaternion_est_en,
							int *start_gyro_cal_en,
							int *ekf_acc_updates_en,
							int *ekf_mag_updates_en);
extern void um6_set_misc_config(struct um6_dev_config *config,
								int pps_en,
								int quaternion_est_en,
								int start_gyro_cal_en,
								int ekf_acc_updates_en,
								int ekf_mag_updates_en);

// initialize the device
extern uint8_t um6_init(struct um6_dev_version *ver,
						struct um6_dev_config *settings);

// run self test on device
extern uint8_t um6_self_test(void);

// read configuration of device
extern void um6_read_config(struct um6_dev_config *config);

// write configuration of device
extern void um6_write_config(const struct um6_dev_config *config);

// get filtered attitude data
extern void um6_get_filtered_attitude(struct um6_dev_att_data *d);

// get filtered euler attitude data
extern void um6_get_filtered_euler_attitude(struct um6_dev_euler_att_data *d);

// get filtered magnetic data
extern void um6_get_filtered_magnetic(struct um6_dev_mag_data *d);

// get filtered acceleration data
extern void um6_get_filtered_acceleration(struct um6_dev_acc_data *d);

// get filtered gyro data
extern void um6_get_filtered_gyro_rate(struct um6_dev_gyro_data *d);

// do gyro zero-rate calibration
// sensor should be stationary, when this is commanded
extern void um6_set_gyro_zero_rate_calibration(void);

// do acceleration reference vector
// sensor should be level, when this is commanded
extern void um6_set_accel_ref_vector(void);

// do magnetic reference vector
// sensor should be oriented to magnetic north, when this is commanded
extern void um6_set_mag_ref_vector(void);

#endif
