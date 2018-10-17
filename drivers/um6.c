#include <stdio.h>
#include <avr/io.h>
#include <avr/pgmspace.h>
#include <util/delay.h>
#include <util/delay_basic.h>
#include "um6.h"
#include "um6_def.h"
#include "spi.h"
#include "globals.h"
#include "watchdog.h"
#include "timer.h"

// globals

#ifdef UM6DEBUG
static const char *k_um6nm = "UM6:";
#endif

// -----------------------------
// SPI functions
// the UM6 is a slave SPI device
// -----------------------------

// set CS low to select device
static void um6_select(void)
{
    PORT_UM6SS &= ~(_BV(P_UM6SS));
}

// set CS high to unselect device
static void um6_unselect(void)
{
    PORT_UM6SS |= _BV(P_UM6SS);
}

static void um6_command_register(const uint8_t address)
{
#ifdef UM6SPIDEBUG
	printf_P(PSTR("%scommand_register:%x\n"), k_um6nm, address);
#endif

	um6_select();
    spi_transfer(UM6_SPI_WRITE);
    spi_transfer(address);
	// no data for a command register
	um6_unselect();
	
#ifdef UM6SPIDEBUG
	printf_P(PSTR("%sdone command_register\n"), k_um6nm);
#endif
}

static void um6_write_register(const uint8_t address, const uint8_t data[4] )
{
#ifdef UM6SPIDEBUG
	printf_P(PSTR("%swrite_register:%x"), k_um6nm, address);
	for(int i=0; i<4; ++i)
		printf_P(PSTR("%x "), data[i]);
	puts("");
#endif

	um6_select();
    spi_transfer(UM6_SPI_WRITE);
    spi_transfer(address);
	for(int i=0; i<4; ++i)
		spi_transfer(data[i]);
	um6_unselect();
	
#ifdef UM6SPIDEBUG
	printf_P(PSTR("%sdone write_register\n"), k_um6nm);
#endif
}

static void um6_read_register(const uint8_t address, uint8_t data[4])
{
#ifdef UM6SPIDEBUG
	printf_P(PSTR("%sread_register:%x\n"), k_um6nm, address);
#endif
	um6_select();
    spi_transfer(UM6_SPI_READ);
    spi_transfer(address);
	for(int i=0; i<4; ++i)
		data[i] = spi_transfer(0); 
	um6_unselect();
	
#ifdef UM6SPIDEBUG
	printf_P(PSTR("%sread_register data:"), k_um6nm);
	for(int i=0; i<4; ++i)
		printf_P(PSTR("%x "), data[i]);
	puts("");
#endif   
}

/*
 * Read UM6 config
 */
void um6_read_config(struct um6_dev_config *config)
{
	um6_read_register(UM6_MISC_CONFIG, config->comm);
	config->misc = config->comm[0];
	um6_read_register(UM6_COMMUNICATION, config->comm);
	
#ifdef UM6DEBUG
	printf_P(PSTR("%sread_config: comm:"), k_um6nm);
	for(int i=0; i<4; ++i)
		printf_P(PSTR("%x "), config->comm[i]);
	printf_P(PSTR(" misc:%x\n"), config->misc);
#endif   
}

/*
 * Write UM6 config
 */
void um6_write_config(const struct um6_dev_config *config)
{
	uint8_t d[4];
	d[0] = config->misc;
	d[1] = d[2] = d[3] = 0;
	um6_write_register(UM6_MISC_CONFIG, d);
	um6_write_register(UM6_COMMUNICATION, config->comm);
	
#ifdef UM6DEBUG
	printf_P(PSTR("%swrite_config\n"), k_um6nm);
#endif   
}

int um6_broadcast_rate(const struct um6_dev_config *config)
{
	return config->comm[3];
}

void um6_set_broadcast_rate(struct um6_dev_config *config, int rate)
{
	config->comm[3] = rate;
}

int um6_baud_rate(const struct um6_dev_config *config)
{
	return config->comm[2] & 0x7;
}

void um6_set_baud_rate(struct um6_dev_config *config, int rate)
{
	config->comm[2] |= (rate & 0x7);
}

int um6_gps_baud_rate(const struct um6_dev_config *config)
{
	return (config->comm[2] & 0x38) >> 3;
}

void um6_set_gps_baud_rate(struct um6_dev_config *config, int rate)
{
	config->comm[2] |= ((rate & 0x7) << 3);
}

void um6_gps_config(const struct um6_dev_config *config,
					int *sat_status_en,
					int *sat_summary_en,
					int *gps_velocity_en,
					int *gps_rel_position_en,
					int *gps_position_en)
{
	*sat_status_en = (config->comm[2] & 0x80) == 0x80;
	*sat_summary_en = (config->comm[1] & 0x1) == 0x1;
	*gps_velocity_en = (config->comm[1] & 0x2) == 0x2;
	*gps_rel_position_en = (config->comm[1] & 0x4) == 0x4;
	*gps_position_en = (config->comm[1] & 0x8) == 0x8;
}

void um6_set_gps_config(struct um6_dev_config *config,
						int sat_status_en,
						int sat_summary_en,
						int gps_velocity_en,
						int gps_rel_position_en,
						int gps_position_en)
{
	config->comm[2] |= sat_status_en << 7;
	config->comm[1] |= (gps_position_en << 3)
		| (gps_rel_position_en << 2)
		| (gps_velocity_en << 1)
		| sat_summary_en;
}

void um6_kalman_config(const struct um6_dev_config *config,
					   int *temperature_en,
					   int *covariance_en,
					   int *euler_en,
					   int *quaternion_en)
{
	*temperature_en = (config->comm[1] & 0x10) == 0x10;
	*covariance_en = (config->comm[1] & 0x20) == 0x20;
	*euler_en = (config->comm[1] & 0x4) == 0x4;
	*quaternion_en = (config->comm[1] & 0x8) == 0x8;
}

void um6_set_kalman_config(struct um6_dev_config *config,
						   int temperature_en,
						   int covariance_en,
						   int euler_en,
						   int quaternion_en)
{
	config->comm[1] |= (quaternion_en << 3)
		| (euler_en << 2)
		| (covariance_en << 1)
		| temperature_en;
}

void um6_output_config(const struct um6_dev_config *config,
					   int *proc_mag_en,
					   int *proc_acc_en,
					   int *proc_gyro_en,
					   int *raw_mag_en,
					   int *raw_acc_en,
					   int *raw_gyro_en,
					   int *broadcast_en)
{
	*proc_mag_en = (config->comm[0] & 0x1) == 0x1;
	*proc_acc_en = (config->comm[0] & 0x2) == 0x2;
	*proc_gyro_en = (config->comm[0] & 0x4) == 0x4;
	*raw_mag_en = (config->comm[0] & 0x8) == 0x8;
	*raw_acc_en = (config->comm[0] & 0x10) == 0x10;
	*raw_gyro_en = (config->comm[0] & 0x20) == 0x20;
	*broadcast_en = (config->comm[0] & 0x40) == 0x40;
}

void um6_set_output_config(struct um6_dev_config *config,
						   int proc_mag_en,
						   int proc_acc_en,
						   int proc_gyro_en,
						   int raw_mag_en,
						   int raw_acc_en,
						   int raw_gyro_en,
						   int broadcast_en)
{
	config->comm[0] |= (broadcast_en << 6)
		| (raw_gyro_en << 5)
		| (raw_acc_en << 4)
		| (raw_mag_en << 3)
		| (proc_gyro_en << 2)
		| (proc_acc_en << 1)
		| proc_mag_en;
}

void um6_misc_config(const struct um6_dev_config *config,
					 int *pps_en,
					 int *quaternion_est_en,
					 int *start_gyro_cal_en,
					 int *ekf_acc_updates_en,
					 int *ekf_mag_updates_en)
{
	*pps_en = (config->misc & 0x4) == 0x4;
	*quaternion_est_en = (config->misc & 0x10) == 0x10;
	*start_gyro_cal_en = (config->misc & 0x20) == 0x20;
	*ekf_acc_updates_en = (config->misc & 0x40) == 0x40;
	*ekf_mag_updates_en = (config->misc & 0x80) == 0x80;
}

void um6_set_misc_config(struct um6_dev_config *config,
						 int pps_en,
						 int quaternion_est_en,
						 int start_gyro_cal_en,
						 int ekf_acc_updates_en,
						 int ekf_mag_updates_en)
{
	config->misc |= (ekf_mag_updates_en << 7)
		| (ekf_acc_updates_en << 6)
		| (start_gyro_cal_en << 5)
		| (quaternion_est_en << 4)
		| (pps_en << 3);
}

/*
 * Initialize the UM6
 */
uint8_t um6_init(struct um6_dev_version *ver,
				 struct um6_dev_config *settings)
{

	// get the firmware version
	um6_read_register(UM6_GET_FW_VERSION, (uint8_t*)ver->fw_version);
	
	// set the configuration
	um6_write_config(settings);

	// now read the configuration
	um6_read_config(settings);
	
	return UM6_OK;
}

/*
 * Returns UM6_OK if ok,
 * UM6_FAIL if failed
 */
uint8_t um6_self_test(void)
{
	uint8_t data[4];
	
	um6_read_register(UM6_STATUS, data);
#ifdef UM6DEBUG
	printf_P(PSTR("%sself_test: "), k_um6nm);
	for (int i=0; i<4; ++i)
		printf_P(PSTR("%x "), data[0]);
	puts("");
#endif
	if(data[0] || data[1] || (data[2] & 0xe) || (data[3] & 0x1))
		return UM6_FAIL;
	
	return UM6_OK;
}

void um6_get_filtered_attitude(struct um6_dev_att_data *d)
{
	uint8_t data[4];

	// x and y axis are reversed, fix later
	um6_read_register(UM6_QUAT_AB, data);
	d->quaternion_est[0] = (data[0] << 8 | data[1]) * K_QUAT_SCALAR;
	d->quaternion_est[1] = (data[2] << 8 | data[3]) * K_QUAT_SCALAR;
	um6_read_register(UM6_QUAT_CD, data);
	d->quaternion_est[2] = (data[0] << 8 | data[1]) * K_QUAT_SCALAR;
	d->quaternion_est[3] = (data[2] << 8 | data[3]) * K_QUAT_SCALAR;
}

void um6_get_filtered_euler_attitude(struct um6_dev_euler_att_data *d)
{
	uint8_t data[4];

	um6_read_register(UM6_EULER_PHI_THETA, data);
	d->att[0] = (data[0] << 8 | data[1]) * K_QUAT_SCALAR;
	d->att[1] = (data[2] << 8 | data[3]) * K_QUAT_SCALAR;
	um6_read_register(UM6_EULER_PSI, data);
	d->att[2] = (data[0] << 8 | data[1]) * K_QUAT_SCALAR;
}

void um6_get_filtered_magnetic(struct um6_dev_mag_data *d)
{
	uint8_t data[4];
	
	// x and y axis are reversed, so fix it
	um6_read_register(UM6_MAG_PROC_XY, data);
	d->mag_vec[1] = (data[0] << 8 | data[1]) * K_MAG_SCALAR;
	d->mag_vec[0] = -(data[2] << 8 | data[3]) * K_MAG_SCALAR;
	um6_read_register(UM6_MAG_PROC_Z, data);
	d->mag_vec[2] = (data[0] << 8 | data[1]) * K_MAG_SCALAR;
	
}

void um6_get_filtered_acceleration(struct um6_dev_acc_data *d)
{
	uint8_t data[4];

	// x and y axis are reversed, so fix it
	um6_read_register(UM6_ACCEL_PROC_XY, data);
	d->acc_vec[1] = (data[0] << 8 | data[1]) * K_ACCEL_SCALAR;
	d->acc_vec[0] = -(data[2] << 8 | data[3]) * K_ACCEL_SCALAR;
	um6_read_register(UM6_ACCEL_PROC_Z, data);
	d->acc_vec[2] = (data[0] << 8 | data[1]) * K_ACCEL_SCALAR;
}

void um6_get_filtered_gyro_rate(struct um6_dev_gyro_data *d)
{
	uint8_t data[4];
	
	// x and y axis are reversed, so fix it
	um6_read_register(UM6_GYRO_PROC_XY, data);
	d->gyro_vec[1] = (data[0] << 8 | data[1]) * K_GYRO_SCALAR;
	d->gyro_vec[0] = -(data[2] << 8 | data[3]) * K_GYRO_SCALAR;
	um6_read_register(UM6_GYRO_PROC_Z, data);
	d->gyro_vec[2] = (data[0] << 8 | data[1]) * K_GYRO_SCALAR;
}

void um6_set_gyro_zero_rate_calibration(void)
{
	um6_command_register(UM6_ZERO_GYROS);
	// delay for 4 seconds, resetting the watchdog every 100ms
	int count = 0;
	while(++count < 40) {
		uint32_t tstart = jiffie();
		while (duration(tstart) < 1000);
		watchdog_reset();
	}
	// now write to flash
	um6_command_register(UM6_FLASH_COMMIT);
}

// um6 should be level, when this is commanded
void um6_set_accel_ref_vector(void)
{
	uint32_t tstart = jiffie();
	um6_command_register(UM6_SET_ACCEL_REF);
	// delay for 100 msecs
	while (duration(tstart) < 1000);
	// now write to flash
	um6_command_register(UM6_FLASH_COMMIT);
}

// um6 should be oriented north, y-axis reading of zero, when this
// is commanded
void um6_set_mag_ref_vector(void)
{
	uint32_t tstart = jiffie();
	um6_command_register(UM6_SET_MAG_REF);
	// delay for 100 msecs
	while (duration(tstart) < 1000);
	// now write to flash
	um6_command_register(UM6_FLASH_COMMIT);
}
