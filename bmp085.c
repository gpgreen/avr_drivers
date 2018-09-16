#include <stdio.h>
#include <util/delay.h>
#include <util/atomic.h>
#include <i2cmaster.h>
#include "bmp085.h"
#include "bmp085_def.h"
#include "driver.h"

/* globals */
#ifdef BMP085DEBUG
static const char* k_name = "bmp085:";
#endif

/*-----------------------------------------------------------------------*/

/*
  write bytes to the BMP085
  
  \param reg the register to write
  \param buf pointer to bytes to write
  \param bufcount number of bytes to write
  \return 0 if ok, -1 if comm failed
  \see bmp085_read()
*/
static int bmp085_write(uint8_t reg, uint8_t* buf, int bufcount)
{
    i2c_start_wait(BMP085_ADDRESS+I2C_WRITE);
	
    if(i2c_write(reg))
        return -1;

    for (int i=0; i<bufcount; ++i)
        if (i2c_write(buf[i]))
            return -1;

    i2c_stop();

    return 0;
}

/*-----------------------------------------------------------------------*/

/*
  read bytes from the BMP085
  
  \param reg the register to start read from
  \param buf pointer to buffer to store data
  \param bufcount number of bytes to read
  \return 0 if ok, -1 if comm failed
  \see bmp085_write()
*/
static int bmp085_read(uint8_t reg, uint8_t* buf, int bufcount)
{
    i2c_start_wait(BMP085_ADDRESS+I2C_WRITE);

    if (i2c_write(reg))
        return -1;

    if (i2c_rep_start(BMP085_ADDRESS+I2C_READ))
        return -1;

    for (int i=0; i<bufcount-1; ++i)
        buf[i] = i2c_readAck();
    buf[bufcount-1] = i2c_readNak();
	
    i2c_stop();

    return 0;
}

/*-----------------------------------------------------------------------*/

// given a user-selected operating mode, get the right op code
static uint8_t get_op_mode_code(enum BMP_OP_MODE omode)
{
    uint8_t code = 0;
    switch (omode) {
    case ULTRA_LOW_POWER:
        code = BMP_OP_ULTRA_LO_PWR_MODE;
        PDEBUG("ultra-low-power mode");
        break;
    case STANDARD:
        code = BMP_OP_STANDARD_MODE;
        PDEBUG("standard mode");
        break;
    case HI_RES:
        code = BMP_OP_HI_RES_MODE;
        PDEBUG("hi-resolution mode");
        break;
    case ULTRA_HI_RES:
        code = BMP_OP_ULTRA_HI_RES_MODE;
        PDEBUG("ultra-high-resolution mode");
        break;
    }
    return code;
}

/*-----------------------------------------------------------------------*/

// Read the calibration data from PROM on device into device structure
static void get_cal_param(struct bmp085_dev_t* device, uint8_t* data)
{
    /*parameters AC1-AC6*/
    device->ac1 =  (data[0] <<8) | data[1];
    device->ac2 =  (data[2] <<8) | data[3];
    device->ac3 =  (data[4] <<8) | data[5];
    device->ac4 =  (data[6] <<8) | data[7];
    device->ac5 =  (data[8] <<8) | data[9];
    device->ac6 = (data[10] <<8) | data[11];
	
    /*parameters B1,B2*/
    device->b1 =  (data[12] <<8) | data[13];
    device->b2 =  (data[14] <<8) | data[15];
	
    /*parameters MB,MC,MD*/
    device->mb =  (data[16] <<8) | data[17];
    device->mc =  (data[18] <<8) | data[19];
    device->md =  (data[20] <<8) | data[21];

    PDEBUG2("%s%d:calibration parameters", k_name, device->num);
    PDEBUG2("\tac1:%hd ac2:%hd ac3:%hd", device->ac1, device->ac2,
             device->ac3);
    PDEBUG2("\tac4:%hu ac5:%hu ac6:%hu", device->ac4, device->ac5,
             device->ac6);
    PDEBUG2("\tb1:%hd b2:%hd"), device->b1, device->b2);
    PDEBUG2("\tmb:%hd mc:%hd md:%hd", device->mb, device->mc,
            device->md);
}

/*-----------------------------------------------------------------------*/

/*
 * Initialize the BMP085
 */
int bmp085_init(enum BMP_OP_MODE omode, struct bmp085_dev_t* device)
{
    PDEBUG2("%d:init", device->num);

    uint8_t data[BMP085_PROM_DATA__LEN];

    // set oversampling
    device->oversampling_setting = get_op_mode_code(omode);

    // read the chip id
    if (bmp085_read(BMP085_CHIP_ID_REG, data, 1)) {
        PDEBUG2("%d:Failed to read chip id", device->num);
        return BMP085_FAIL;
    }
    device->chip_id = data[0];
    PDEBUG2("%d:chip_id:%x", device->num, device->chip_id);
		
    // read version information
    if (bmp085_read(BMP085_VERSION_REG, data, 1)) {
        PDEBUG2("%d:Failed to read version", device->num);
        return BMP085_FAIL;
    }
    device->ml_version = BMP085_GET_BITSLICE(data[0], BMP085_ML_VERSION);
    device->al_version = BMP085_GET_BITSLICE(data[0], BMP085_AL_VERSION);
    PDEBUG2("%d:ml version:%x", device->num, device->ml_version);
    PDEBUG2("%d:al version:%x", device->num, device->al_version);
		
    // read calibration parameter information
    if (bmp085_read(BMP085_PROM_START__ADDR, data, BMP085_PROM_DATA__LEN)) {
        PDEBUG2("%d:Failed to read calibration parameters", device->num);
        return BMP085_FAIL;
    }
    get_cal_param(device, data);
	
    PDEBUG2("%d:init done", device->num);

    return BMP085_OK;
}

/*-----------------------------------------------------------------------*/

/*
 * Returns BMP085_OK if ok,
 * BMP085_FAIL if failed
 */
int bmp085_self_test(struct bmp085_dev_t* device)
{
    PDEBUG2("%d:self test", device->num);

    // verify that all configuration data is not 0 or 0xffff
    // from the datasheet, section 3.4, p12.
    if (device->ac1 == 0 || device->ac1 == 0xffff)
        return BMP085_FAIL;
    if (device->ac2 == 0 || device->ac2 == 0xffff)
        return BMP085_FAIL;
    if (device->ac3 == 0 || device->ac3 == 0xffff)
        return BMP085_FAIL;
    if (device->ac4 == 0 || device->ac4 == 0xffff)
        return BMP085_FAIL;
    if (device->ac5 == 0 || device->ac5 == 0xffff)
        return BMP085_FAIL;
    if (device->ac6 == 0 || device->ac6 == 0xffff)
        return BMP085_FAIL;
    if (device->b1 == 0 || device->b1 == 0xffff)
        return BMP085_FAIL;
    if (device->b2 == 0 || device->b2 == 0xffff)
        return BMP085_FAIL;
    if (device->mb == 0 || device->mb == 0xffff)
        return BMP085_FAIL;
    if (device->mc == 0 || device->mc == 0xffff)
        return BMP085_FAIL;
    if (device->md == 0 || device->md == 0xffff)
        return BMP085_FAIL;
	
    PDEBUG2("%d:self test ok", device->num);

    return BMP085_OK;
}

/*-----------------------------------------------------------------------*/

/*
 * Read the temperature & pressure data from device
 */
void bmp085_read_data(struct bmp085_dev_t* device)
{
    PDEBUG2("%d:read_data", device->num);
    bmp085_read_temperature(device);
    bmp085_get_temperature(device);

    bmp085_read_pressure(device);
    bmp085_get_pressure(device);
}

/*-----------------------------------------------------------------------*/

/*
 * Read the temperature from device
 */
void bmp085_read_temperature(struct bmp085_dev_t* device)
{
    uint8_t data[2];

    PDEBUG2("%d:read_temperature", device->num);
	
    // we want temperature sample
    uint8_t code = BMP085_T_MEASURE;
    bmp085_write(BMP085_CTRL_MEAS_REG, &code, 1);

    // wait
    _delay_ms(BMP085_TEMP_CONVERSION_TIME);
	
    // read temperature sample
    bmp085_read(BMP085_ADC_OUT_MSB_REG, data, 2);

    device->rt = (uint16_t)(data[0] << 8) | data[1];
    device->status = TEMPERATURE;
}

/*-----------------------------------------------------------------------*/

/*
 * Read the pressure data from device
 */
void bmp085_read_pressure(struct bmp085_dev_t* device)
{
    uint8_t data[3];

    PDEBUG2("%d:read_pressure", device->num);
	
    // we want pressure sample
    uint8_t code = BMP085_P_MEASURE + (device->oversampling_setting << 6);
    bmp085_write(BMP085_CTRL_MEAS_REG, &code, 1);

    // wait
    switch (device->oversampling_setting) {
    case 0:
        _delay_ms(2);
        break;
    case 1:
        _delay_ms(10);
        break;
    case 2:
        _delay_ms(18);
        break;
    case 3:
        _delay_ms(26);
        break;
    }
	
    // read pressure sample
    bmp085_read(BMP085_ADC_OUT_MSB_REG, data, 3);

    device->rp = (((uint32_t)data[0] << 16) | ((uint32_t)data[1] << 8)
                  | (uint32_t)data[2]) >> (8 - device->oversampling_setting);
	device->status = PRESSURE;
}

/*-----------------------------------------------------------------------*/

/** calculate temperature from raw data
    device->rt was read from the device via I2C
    gives temperature in steps of 0.1 deg celsius
    \see bmp085_read_temperature()
*/

void bmp085_get_temperature(struct bmp085_dev_t* device) 
{
    int16_t temperature;
    int32_t x1,x2;    

    x1 = (((int32_t) device->rt - (int32_t) device->ac6)
          * (int32_t) device->ac5) >> 15;
    x2 = ((int32_t) device->mc << 11) / (x1 + device->md);
    device->param_b5 = x1 + x2;
    temperature = ((device->param_b5 + 8) >> 4);  // temperature in 0.1°C

    PDEBUG2("%d:temp:%hd", device->num, temperature);

    // get temperature, in degrees kelvin
    device->temp = (float)temperature / 10.0 + 273.15;
}

/*-----------------------------------------------------------------------*/

/** calculate pressure from raw data
    device->rp was read from the device via I2C
    In case of BMP085 averaging is done through oversampling by the sensor IC

    gives pressure in steps of kPa
    \see bmp085_read_pressure()
*/

void bmp085_get_pressure(struct bmp085_dev_t* device)
{
    int32_t pressure,x1,x2,x3,b3,b6;
    uint32_t b4, b7;
   
    b6 = device->param_b5 - 4000;
    //*****calculate B3************
    x1 = (b6*b6) >> 12;	 	 
    x1 *= device->b2;
    x1 >>=11;

    x2 = (device->ac2*b6);
    x2 >>=11;

    x3 = x1 +x2;

    b3 = (((((int32_t)device->ac1 )*4 + x3)
           <<device->oversampling_setting) + 2) >> 2;

    //*****calculate B4************
    x1 = (device->ac3* b6) >> 13;
    x2 = (device->b1 * ((b6*b6) >> 12) ) >> 16;
    x3 = ((x1 + x2) + 2) >> 2;
    b4 = (device->ac4 * (uint32_t) (x3 + 32768)) >> 15;
     
    b7 = ((uint32_t)(device->rp - b3)
          * (50000>>device->oversampling_setting));   
    if (b7 < 0x80000000)
	{
	    pressure = (b7 << 1) / b4;
	}
    else
	{ 
	    pressure = (b7 / b4) << 1;
	}
   
    x1 = pressure >> 8;
    x1 *= x1;
    x1 = (x1 * SMD500_PARAM_MG) >> 16;
    x2 = (pressure * SMD500_PARAM_MH) >> 16;
    pressure += (x1 + x2 + SMD500_PARAM_MI) >> 4;	// pressure in Pa  

    PDEBUG2("%d:press:%ld", device->num, pressure);

    // get pressure, in Pa, convert to hPa
    device->press = (float)pressure / 100.0;
}

/*-----------------------------------------------------------------------*/

/** check for completion of reading
 */

enum bmp_sensor_reading bmp085_async_check_reading(struct bmp085_dev_t* device)
{
    PDEBUG2("%d:bmp_async_check_reading", device->num);
	enum bmp_sensor_reading m = NONE;
    
    ATOMIC_BLOCK(ATOMIC_FORCEON) {
        m = device->status;
    }
    return m;
}

/*-----------------------------------------------------------------------*/

/** clear reading
 */

void bmp085_async_clear_reading(struct bmp085_dev_t* device)
{
    PDEBUG2("%d:bmp_async_clear_reading", device->num);
    ATOMIC_BLOCK(ATOMIC_FORCEON) {
        device->status = NONE;
    }
}

/*-----------------------------------------------------------------------*/

/** start async pressure reading
 */

int bmp085_async_start_pressure(struct bmp085_dev_t* device)
{
    PDEBUG2("%d:async_start_pressure", device->num);
	
    // check and make sure that we haven't got messed up
    ATOMIC_BLOCK(ATOMIC_FORCEON) {
        if(device->op != NOOP)
            device->pending_op = PRESS;
        else {
            // we want pressure sample
            uint8_t code = BMP085_P_MEASURE + (device->oversampling_setting << 6);
            bmp085_write(BMP085_CTRL_MEAS_REG, &code, 1);
            device->op = PRESS;
        }
    }
    return BMP085_OK;
}

/*-----------------------------------------------------------------------*/

/** start async temperature reading
 */

int bmp085_async_start_temperature(struct bmp085_dev_t* device)
{
    PDEBUG2("%d:async_start_temperature", device->num);
	
    // check and make sure that we haven't got messed up
    ATOMIC_BLOCK(ATOMIC_FORCEON) {
        if(device->op != NOOP) {
            device->pending_op = TEMP;
        }
        else {
            // we want temperature sample
            uint8_t code = BMP085_T_MEASURE;
            bmp085_write(BMP085_CTRL_MEAS_REG, &code, 1);
            device->op = TEMP;
        }
    }
    
    return BMP085_OK;
}

/*-----------------------------------------------------------------------*/

/** handle an EOC interrupt, called from interrupt handler
 * 
 */

int bmp085_async_handle_interrupt(struct bmp085_dev_t* device)
{
    uint8_t data[3];

    // check and make sure that we haven't got messed up
    if(device->op == PRESS) {
        // read pressure sample
        bmp085_read(BMP085_ADC_OUT_MSB_REG, data, 3);

        device->rp = (((uint32_t)data[0] << 16) | ((uint32_t)data[1] << 8)
                      | (uint32_t)data[2]) >> (8 - device->oversampling_setting);
        device->status = PRESSURE;
    }
    else if(device->op == TEMP) {
        // read temperature sample
        bmp085_read(BMP085_ADC_OUT_MSB_REG, data, 2);

        device->rt = (uint16_t)(data[0] << 8) | data[1];
        device->status = TEMPERATURE;
    }
    
    // check for pending operations
    device->op = NOOP;
    if(device->pending_op == PRESS || device->pending_op == NOOP) {
        // we want pressure sample
        uint8_t code = BMP085_P_MEASURE + (device->oversampling_setting << 6);
        bmp085_write(BMP085_CTRL_MEAS_REG, &code, 1);
        device->op = PRESS;
    }
    else if(device->pending_op == TEMP) {
        // we want temperature sample
        uint8_t code = BMP085_T_MEASURE;
        bmp085_write(BMP085_CTRL_MEAS_REG, &code, 1);
        device->op = TEMP;
    }
    device->pending_op = NOOP;
	
    return BMP085_OK;
}

#if 0
/*
 * bmp085_check_code
 * this function checks the get_temperature and get_pressure code
 * by setting the calibration parameters and using fake pressure
 * and temperature readings as shown in the datasheet. The answers
 * should match the datasheet
 */
void bmp085_check_code(void)
{
    // fake calibration data
    struct bmp085_dev_t tst;
    tst.ac1 = 408;
    tst.ac2 = -72;
    tst.ac3 = -14383;
    tst.ac4 = 32741;
    tst.ac5 = 32757;
    tst.ac6 = 23153;
    tst.b1 = 6190;
    tst.b2 = 4;
    tst.mb = -32768;
    tst.mc = -8711;
    tst.md = 2868;
    // oversampling setting s/b 0
    tst.oversampling_setting = 0;
	
    int16_t temp = bmp085_get_temperature(27898, &tst);
    int32_t press = bmp085_get_pressure(23843, &tst);

    PDEBUG2("temp:%hd sb 150", temp);
    PDEBUG2("press:%ld sb 69964", press);
}

#endif
