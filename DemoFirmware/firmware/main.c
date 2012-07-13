/* Name: sine_fader.c
 * Author: Icy Labs Pty. Ltd.
 * Copyright: (c) 2012 Icy Labs Pty. Ltd.
 * License: Internal Use Only. Open Source Licensing TBA
 */

#include "thinklet.h"
#include <avr/pgmspace.h>

#define TRUE 1
#define FALSE 0

#define LOOP_FLAG 1

/*
#define WAVEFORM_STORAGE_SIZE 0

uint8_t channelData[WAVEFORM_STORAGE_SIZE] PROGMEM = {

};
*/

// Example Data
uint8_t channelData[] PROGMEM = {
	20, // waiting time (us)
	2, // Number of waveforms

	// waveform:
	0, 10, // length, high low
	1, // flags (to loop or not)
	// waypoints:
	0, 12,
	10, 10,
	15, 0,
	0, 15,
	255, 1,

	// waveform 1:
	0, 6, // length
	0, // flags
	// waypoints:
	5, 12,
	255, 10,
	0, 20
};


/*
 * Interpolation and waveform functions
 */

typedef struct channelData {
	// id/index for this channel
	uint8_t id;

	// the x (time) and y (brightness) distances
	// of the current interpolation
	uint8_t dx, dy;

	
	// the sign (direction) of the
	// interpolation slope
	//int8_t sx; // not needed: time only goes in one direction
	int8_t sy;

	// x (time) and y (brightness) coordinates
	// of the line (x0, y0), (x1, y1) that is
	// yet to be interpolated
	uint8_t x0, x1;
	uint8_t y0, y1;
	
	// an error measure for drawing the
	// interpolated line
	int16_t err;

	// the x and y value at the beginning
	// of the interpolation
	uint8_t x_prev;
	uint8_t y_prev;

	// the current output result (brightness)
	// of this channel
	uint8_t result;

	// flag for whether the interpolation
	// has completed
	uint8_t done;

	// flag for whether we're at the start of
	// the interpolation
	uint8_t first;

	// the current index of the waveform lookup
	uint16_t waveIndex;

	// flags, e.g. looping
	uint8_t flags;

	// length of the whole waveform
	uint16_t length;

	// program memory where the waveform is stored
	uint8_t *wave;

} channelData_t;



// output the current value of the channel
void channelOut( channelData_t *channel );

// brentham's line algorithm for 
// discrete linear interpolation
void brenthamInit( channelData_t *channel );
void brenthamIteration( channelData_t *channel );

// start interpolating a channel, and move to the next waypoint
void startChannelInterpolation( channelData_t *channel );

// move along an interpolation line until
// the next timestep is reached
uint8_t interpolateChannel( channelData_t *channel );

// do an interpolation step on a channel,
// taking into account moving on to the next waypoint
uint8_t stepChannel( channelData_t *channel );

// do an interpolation step over all channels
uint8_t stepAllChannels( channelData_t *channels, uint8_t numChannels );

// run the waveforming loop until all waveforms have finished
// (or forever, if one loops)
void startWaveforming( void );




void brenthamInit( channelData_t *channel ) {
	// initialise a channel to start a new interpolation
	// using brentham's algorithm

	if ( channel->x0 < channel->x1 ) {
		channel->dx = channel->x1 - channel->x0;
		//channel->sx = 1; // our lines only go left to right
	} else {
		channel->dx = channel->x0 - channel->x1;
		//channel->sx = -1;
	}

	if ( channel->y0 < channel->y1 ) {
		channel->dy = channel->y1 - channel->y0;
		channel->sy = 1; // going up
	} else {
		channel->dy = channel->y0 - channel->y1;
		channel->sy = -1; // going down
	}

	channel->err = channel->dx - channel->dy;
}

void brenthamIteration( channelData_t *channel ) {
	// perform an iteration of brentham's line algorithm

	int16_t e2 = channel->err << 1;

	if ( e2 > -channel->dy ) {
		channel->err -= channel->dy;
		channel->x0 += 1;//channel->sx;
	}

	if ( e2 < channel->dx ) {
		channel->err += channel->dx;
		channel->y0 += channel->sy;
	}
}

void startChannelInterpolation( channelData_t *channel ) {
	// load the next interpolation's
	// start, end and duration from program memory

	uint8_t index = channel->waveIndex;

	// look up the next waveform coordinates:
	//   - brightness start
	//   - time to interpolate for
	//   - brightness target
	channel->x0 = 0;
	channel->y0 = pgm_read_byte(&channel->wave[index]);

	channel->x1 = pgm_read_byte(&channel->wave[index+1]);
	channel->y1 = pgm_read_byte(&channel->wave[index+2]);

	// initialise the new interpolation
	brenthamInit( channel );

	// reset the 'first' flag
	channel->first = TRUE;

	// skip over the values already loaded into the interpolation
	// to make the brightness target the next starting brightness
	channel->waveIndex = index + 2;

	// reset the 'done' flag
	channel->done = FALSE;
}


uint8_t interpolateChannel( channelData_t *channel ) {
	// step the interpolation of a single channel

	// save the previous x and y
	channel->x_prev = channel->x0;
	channel->y_prev = channel->y0;

	// skip over all interpolated values which are in the same time step
	while ( channel->x_prev == channel->x0 ) {
		brenthamIteration( channel );

		if ( channel->x0 == channel->x1 && channel->y0 == channel->y1 ) {
			// detect if we've landed on the endpoint, and escape if so
			channel->done = TRUE;
			break;
		}
	}

	// if we're done on this interpolation, generate a result
	if ( !channel->done ) {
		if ( channel->first ) {
			channel->first = FALSE;

			// for the the first point, we take its starting value
			channel->result = channel->y_prev;
		} else {
			// for subsequent points we take the mid-point of the 
			// interpolated line
			channel->result = (channel->y0+channel->y_prev)>>1;
		}

		channelOut( channel );
	}

	// return if successfully generated a result
	return !channel->done;
}


uint8_t stepChannel( channelData_t *channel ) {
	// step the interpolation of a channel until a result is found

	if ( channel->flags & LOOP_FLAG && channel->waveIndex >= channel->length ) {
		// reset the waveform if it's flagged as looping

		channel->waveIndex = 0;
		startChannelInterpolation( channel );
	}


	if ( channel->waveIndex < channel->length ) {
		// still have more waypoints to interpolate over

		// move along the interpolation line by one step
		while ( !interpolateChannel( channel ) ) {
			// move on to next waypoint until one step is made
			startChannelInterpolation( channel );
		}

		// possibly still more waypoints
		return 0;
	} else {

		// run out of waypoints
		return 1;
	}
}

uint8_t stepAllChannels( channelData_t *channels, uint8_t numChannels ) {
	uint8_t i;
	uint8_t numFinished;

	numFinished = 0;
	for ( i = 0; i < numChannels; i++ ) {
		// count how many channels have finished stepping
		// through their waveforms
		numFinished += stepChannel( &channels[i] );
	}

	return (numFinished == numChannels);
}


// globally store the number of channels being used
// so the timer interrupt can access it
global uint8_t numChannels;

void startWaveforming( void ) {
	uint8_t i;
	uint16_t offset;

	// first two values in the waveform data
	uint8_t waitTime = pgm_read_byte(&channelData[0]);
	
	numChannels = pgm_read_byte(&channelData[1]);

	channelData_t channels[numChannels];

	// already read two values
	offset = 2;
	for ( i = 0; i < numChannels; i++ ) {
		channels[i].id = i;
		channels[i].waveIndex = 0;

		// high byte
		channels[i].length = (pgm_read_byte(&channelData[offset]) << 8);
		// low byte
		channels[i].length |= pgm_read_byte(&channelData[offset+1]);

		// flags (e.g. looping)
		channels[i].flags  =  pgm_read_byte(&channelData[offset+2]);

		// program memory pointer
		channels[i].wave = &(channelData[offset+3]);

		// initialise this waveform's channel
		startChannelInterpolation( &channels[i] );

		// skip over the 3 values read, and the length of the waveform
		offset += channels[i].length + 3;
	}

	// each iteration of the waveforming
	while ( !stepAllChannels( channels, numChannels ) ) {
		// still more steps to go, for all waveforms

		// wait the time resolution between updates
		wait_us( waitTime );
	}
}



/* 
 * PWM, gamma correction and pin setup functions
 */

#define MAX_CHANNELS 6

#define CHANNEL_PIN_0 PB0
#define CHANNEL_PIN_1 PB1
#define CHANNEL_PIN_2 PB2
#define CHANNEL_PIN_3 PB3
#define CHANNEL_PIN_4 PB4
#define CHANNEL_PIN_5 PB5

#define BRIGHTNESS_LIMIT 256

// Gamma correction lookup table
uint8_t gammaTable[BRIGHTNESS_LIMIT] PROGMEM = {
	  0,   1,   1,   1,   1,   1,   1,   1,   1,   1,   1,   1,   1,   1,   1,   1,
	  1,   1,   1,   1,   1,   1,   1,   1,   1,   1,   1,   1,   2,   2,   2,   2,
	  2,   2,   2,   2,   2,   3,   3,   3,   3,   3,   3,   3,   4,   4,   4,   4,
	  4,   5,   5,   5,   5,   6,   6,   6,   6,   7,   7,   7,   7,   8,   8,   8,
	  9,   9,   9,  10,  10,  10,  11,  11,  11,  12,  12,  12,  13,  13,  14,  14,
	 15,  15,  15,  16,  16,  17,  17,  18,  18,  19,  19,  20,  20,  21,  22,  22,
	 23,  23,  24,  24,  25,  26,  26,  27,  28,  28,  29,  30,  30,  31,  32,  32,
	 33,  34,  35,  35,  36,  37,  38,  38,  39,  40,  41,  42,  43,  43,  44,  45,
	 46,  47,  48,  49,  50,  51,  52,  53,  53,  54,  55,  56,  57,  58,  60,  61,
	 62,  63,  64,  65,  66,  67,  68,  69,  70,  72,  73,  74,  75,  76,  78,  79,
	 80,  81,  83,  84,  85,  86,  88,  89,  90,  92,  93,  94,  96,  97,  99, 100,
	101, 103, 104, 106, 107, 109, 110, 112, 113, 115, 116, 118, 120, 121, 123, 124,
	126, 128, 129, 131, 133, 134, 136, 138, 139, 141, 143, 145, 146, 148, 150, 152,
	154, 156, 157, 159, 161, 163, 165, 167, 169, 171, 173, 175, 177, 179, 181, 183,
	185, 187, 189, 191, 193, 195, 198, 200, 202, 204, 206, 208, 211, 213, 215, 217,
	220, 222, 224, 227, 229, 231, 234, 236, 238, 241, 243, 246, 248, 251, 253, 255
};

// channel id to pin lookup
const uint8_t channelPins[MAX_CHANNELS] = {
	CHANNEL_PIN_0,
	CHANNEL_PIN_1,
	CHANNEL_PIN_2,
	CHANNEL_PIN_3,
	CHANNEL_PIN_4,
	CHANNEL_PIN_5
};

// channel brightnesses
global uint8_t channelValues[MAX_CHANNELS] = {0, 0, 0, 0, 0, 0};

// save the current channel value to the output array
void channelOut( channelData_t *channel ) {
	channelValues[channel->id]	= channel->result;
}

/*
 * Built-in Macros
 */

setup( ) {
	/*
	make_output_pin( CHANNEL_PIN_0 );
	make_output_pin( CHANNEL_PIN_1 );
	make_output_pin( CHANNEL_PIN_2 );
	make_output_pin( CHANNEL_PIN_3 );
	make_output_pin( CHANNEL_PIN_4 );
	make_output_pin( CHANNEL_PIN_5 );
	// equivalent to DDRB = 0xff;
	*/
	DDRB = 0xff; // set everything to be an output
	
	// run the internal timer interrupt as fast as it can
	setup_timer( TIMER_Clock );
	
	// start interrupting
	start_interrupts( );
	
	// run the waveforming loop
	startWaveforming( );
}

on_timer( ) {
	// each timer interrupt, update PWM for each channel

	// where in the duty cycle we're up to
	static uint8_t windowCounter = 0;
	
	uint8_t i;

	// turn each channel on or off according to PWM
	for ( i = 0; i < numChannels; i++ ) {

		// compare against a gamma corrected brightness (via lookup table)
		if ( windowCounter >= pgm_read_byte(gammaTable + channelValues[i]) ) {
			// turn pin off when the duty cycle timing is past the brightness
			pin_off( channelPins[i] );
		} else {
			// turn the pin on for a duration proportional to the brightness
			pin_on( channelPins[i] );
		}

	}

	windowCounter = (windowCounter + 1) % BRIGHTNESS_LIMIT;
}

loop( ) {
	// shut down microcontroller when finished
	sleep( );
}

