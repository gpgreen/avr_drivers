#include "defs.h"

#ifdef ADCDEBUG
#include <stdio.h>
#include <avr/pgmspace.h>
#endif

#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <util/atomic.h>
#include "adc.h"

// define which adc channel goes to which sensor
// 0 - gyro x, 1 - gyro y, 2 - gyro z, 6 - vref1, 7 - vref2
const int k_channel_map[8] = {1, 2, 0, 3, 4, 5, 7, 6};

// define which vref applies to axis
const int k_vref_map[3] = {0, 0, 1};

// defines which analog channel is being sampled
static volatile uint8_t s_muxsel;

// defines which voltage reference is being used
// this is or'd with s_muxsel to choose
// and initiate the sample
static volatile uint8_t s_reference;

// the data sampled at each pin
static volatile uint16_t s_channel[8];

// the number of samples at each pin
static volatile uint8_t s_count[8];

// the raw data read from device
static int16_t s_gyro_data[3];

// the bias of the device on each axis
static int16_t s_bias[3];

// the raw data for the vref lines
static int16_t s_vref[2];

// device structure
static struct gyro_device* s_device;

void
adc_init(struct gyro_device* device)
{
	s_device = device;

#ifdef ADCDEBUG
	puts_P(PSTR("gyros initializing"));

	// print out self-test
	printf_P(PSTR("gyro axis map: %d,%d,%d\n"),
			 s_device->axis_map[0],s_device->axis_map[1],
			 s_device->axis_map[2]);
	printf_P(PSTR("gyro sign map: %d,%d,%d\n"),
			 s_device->sign_map[0],s_device->sign_map[1],
			 s_device->sign_map[2]);
#endif
	
	s_reference = _BV(REFS0);
	
	// reset the gyros
	PORT_HP |= _BV(P_HP);
	_delay_ms(20);

	PORT_HP &= ~(_BV(P_HP));

    // turn it on, enable interrupt, scale clock by 8
    ADCSRA |= _BV(ADIE) | _BV(ADEN) | _BV(ADPS1) | _BV(ADPS0);
    // start conversions
    ADCSRA |= _BV(ADSC);

#ifdef ADCDEBUG
	puts_P(PSTR("gyros setup"));
#endif
}

void
adc_read(void)
{
    uint16_t temp1;
    uint8_t temp2;
    
    for (int i=0; i<8; ++i) {
		if (i >= 3 && i <= 5)
			continue;
		int j = k_channel_map[i];
		// we need to copy s_channel, s_count with interrupts off, so the
		// isr doesn't corrupt the data
		ATOMIC_BLOCK(ATOMIC_RESTORESTATE)
		{
			temp1 = s_channel[j];
			temp2 = s_count[j];
		}
        if (temp2 > 0)
		{
			if (i < 3) {
				s_gyro_data[i] = temp1 / temp2;
			}
			else {
				s_vref[i-6] = temp1 / temp2;
			}
		}
    }
    for (int i=0; i<8; ++i) {
		if (i >= 3 && i <= 5)
			continue;
        do {
            s_channel[i] = 0;
            s_count[i] = 0;
        } while (s_channel[i] != 0);
    }
}

// get gyro data
float
get_gyro_data(int axis)
{
	int corr_axis = s_device->axis_map[axis];
	return s_device->sign_map[axis] * (s_gyro_data[corr_axis]
									   - s_vref[k_vref_map[corr_axis]]
									   - s_bias[corr_axis]);
}

int16_t
get_gyro_bias(int axis)
{
	return s_bias[s_device->axis_map[axis]];
}

// gyro's self-test
void 
gyro_self_test(void)
{
	uint16_t testvb[3] = {0,0,0};
	uint16_t testva[3] = {0,0,0};
	
	// read and average 16 values without self-test applied
	for(int i=0; i<16; ++i)    // We take some readings...
	{
		adc_read();
		for(int y=0; y<3; y++)   // Cumulate values
			testvb[y] += s_gyro_data[y];
		_delay_ms(20);
	}
    
	// apply self-test
	PORT_ST |= _BV(P_ST);
	_delay_ms(100);
  
	// read and average 16 values with self-test applied
	for(int i=0; i<16; i++)    // We take some readings...
	{
		adc_read();
		for(int y=0; y<3; y++)   // Cumulate values
			testva[y] += s_gyro_data[y];
		_delay_ms(20);
	}
    
	// remove self-test
	PORT_ST &= ~(_BV(P_ST));

	// now compute the averages
	for(int y=0; y<3; y++) {
		testvb[y] /= 16;
		testva[y] /= 16;
	}

#ifdef ADCDEBUG
	// print out self-test
	puts_P(PSTR("Gyros before self-test:"));
	printf_P(PSTR("%hd,%hd,%hd\n"), testvb[0], testvb[1], testvb[2]);
	puts_P(PSTR("Gyros self-test:"));
	printf_P(PSTR("%hd,%hd,%hd\n"), testva[0], testva[1], testva[2]);
#endif
}  
  
// gyro's set bias
void 
gyro_set_bias(void)
{
	int16_t testv[3] = {0,0,0};
	
	// read and average 16 values
	for(int i=0; i<16; ++i)    // We take some readings...
	{
		adc_read();
		for(int y=0; y<3; y++)   // Cumulate values
			testv[y] += (s_gyro_data[y] - s_vref[k_vref_map[y]]);
		_delay_ms(20);
	}
    
	// now compute the averages
	for(int y=0; y<3; y++) {
		s_bias[y] = testv[y] / 16;
	}
#ifdef ADCDEBUG
	// print out bias values
	puts_P(PSTR("Gyro bias values:"));
	printf_P(PSTR("%hd,%hd,%hd\n"), s_bias[0], s_bias[1], s_bias[2]);
#endif
}  
  
ISR(ADC_vect)
{
    volatile uint8_t low, high;

    low = ADCL;
    high = ADCH;
    
    if(s_count[s_muxsel] < 63) {
        s_channel[s_muxsel] += (high << 8) | low;   // cumulate analog values
        ++s_count[s_muxsel];
    }
    ++s_muxsel;
	if (s_muxsel == 3)
		s_muxsel = 6;
	else if (s_muxsel == 8)
		s_muxsel = 0;
    ADMUX = s_reference | s_muxsel;
    // start the conversion
    ADCSRA |= _BV(ADSC);
}
