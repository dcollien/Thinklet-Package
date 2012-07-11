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


typedef struct channelLerp {
	uint8_t id;

	uint8_t dx, dy;
	//int8_t sx;
	
	int8_t sy;

	uint8_t x0, x1;
	uint8_t y0, y1;
	
	int16_t err;

	uint8_t x_prev;
	uint8_t y_prev;

	uint8_t result;

	uint8_t done;
	uint8_t first;

	uint16_t waveIndex;

	uint8_t flags;

	uint16_t length;
	uint8_t *wave;

} channelLerp_t;

static inline void channelOut( channelLerp_t *channel );

static inline void brenthamInit( channelLerp_t *channel ) {
	// initialise a channel to start a new interpolation
	// using brentham's algorithm

	if ( channel->x0 < channel->x1 ) {
		channel->dx = channel->x1 - channel->x0;
		//channel->sx = 1;
	} else {
		channel->dx = channel->x0 - channel->x1;
		//channel->sx = -1;
	}

	if ( channel->y0 < channel->y1 ) {
		channel->dy = channel->y1 - channel->y0;
		channel->sy = 1;
	} else {
		channel->dy = channel->y0 - channel->y1;
		channel->sy = -1;
	}

	channel->err = channel->dx - channel->dy;
}

static inline void brenthamIteration( channelLerp_t *channel ) {
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

void startChannelLerp( channelLerp_t *channel ) {
	// load the next interpolation's
	// start, end and duration from program memory

	channel->x0 = 0;
	channel->y0 = pgm_read_byte(&channel->wave[channel->waveIndex]);

	channel->x1 = pgm_read_byte(&channel->wave[channel->waveIndex+1]);
	channel->y1 = pgm_read_byte(&channel->wave[channel->waveIndex+2]);

	// initialise the new interpolation
	brenthamInit( channel );

	// reset the 'first' flag
	channel->first = TRUE;

	// skip over the values already loaded into the interpolation
	channel->waveIndex += 2;

	// reset the 'done' flag
	channel->done = FALSE;
}


uint8_t lerpStepChannel( channelLerp_t *channel ) {
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


uint8_t stepChannel( channelLerp_t *channel ) {
	// step the interpolation of a channel until a result is found

	if ( channel->flags & LOOP_FLAG && channel->waveIndex >= channel->length ) {
		// reset the waveform if it's flagged as looping

		channel->waveIndex = 0;
		startChannelLerp( channel );
	}


	if ( channel->waveIndex < channel->length ) {
		// still have more waypoints to interpolate over

		// move along the interpolation line by one step
		while ( !lerpStepChannel( channel ) ) {
			// move on to next waypoint until one step is made
			startChannelLerp( channel );
		}

		// possibly still more waypoints
		return 0;
	} else {

		// run out of waypoints
		return 1;
	}
}

uint8_t stepAllChannels( channelLerp_t *channels, uint8_t numChannels ) {
	uint8_t i;
	uint8_t numFinished;

	numFinished = 0;
	for ( i = 0; i < numChannels; i++ ) {
		numFinished += stepChannel( &channels[i] );
	}

	return (numFinished == numChannels);
}


void start_waveforming( void ) {
	uint8_t i;
	uint16_t offset;

	// first two values in the waveform data
	uint8_t waitTime = pgm_read_byte(&channelData[0]);
	uint8_t numChannels = pgm_read_byte(&channelData[1]);

	channelLerp_t channels[numChannels];

	// already read two values
	offset = 2;
	for ( i = 0; i < numChannels; i++ ) {
		channels[i].id = i;
		channels[i].waveIndex = 0;

		channels[i].length = (pgm_read_byte(&channelData[offset]) << 8);
		channels[i].length |= pgm_read_byte(&channelData[offset+1]);
		channels[i].flags  =  pgm_read_byte(&channelData[offset+2]);

		// program memory pointer
		channels[i].wave = &(channelData[offset+3]);

		// initialise this waveform's channel
		startChannelLerp( &channels[i] );

		// skip over the 3 values read, and the length of the waveform
		offset += channels[i].length + 3;
	}

	// each iteration of the waveforming
	while ( !stepAllChannels( channels, numChannels ) ) {
		// wait the time resolution between updates
		wait_us( waitTime );
	}
}



/* 
 * PWM and pin setup
 */

#define NUM_CHANNELS 6

#define CHANNEL_PIN_0 PB0
#define CHANNEL_PIN_1 PB1
#define CHANNEL_PIN_2 PB2
#define CHANNEL_PIN_3 PB3
#define CHANNEL_PIN_4 PB4
#define CHANNEL_PIN_5 PB5

#define PWM_WINDOW 255
#define BRIGHTNESS_LIMIT 256

uint8_t gammaTable[BRIGHTNESS_LIMIT] PROGMEM = {
	  0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,
	  0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   1,   1,   1,   1,
	  1,   1,   1,   1,   1,   2,   2,   2,   2,   2,   2,   2,   3,   3,   3,   3,
	  3,   4,   4,   4,   4,   5,   5,   5,   5,   6,   6,   6,   6,   7,   7,   7,
	  8,   8,   8,   9,   9,   9,  10,  10,  10,  11,  11,  11,  12,  12,  13,  13,
	 14,  14,  14,  15,  15,  16,  16,  17,  17,  18,  18,  19,  19,  20,  21,  21,
	 22,  22,  23,  23,  24,  25,  25,  26,  27,  27,  28,  29,  29,  30,  31,  31,
	 32,  33,  34,  34,  35,  36,  37,  37,  38,  39,  40,  41,  42,  42,  43,  44,
	 45,  46,  47,  48,  49,  50,  51,  52,  52,  53,  54,  55,  56,  57,  59,  60,
	 61,  62,  63,  64,  65,  66,  67,  68,  69,  71,  72,  73,  74,  75,  77,  78,
	 79,  80,  82,  83,  84,  85,  87,  88,  89,  91,  92,  93,  95,  96,  98,  99,
	100, 102, 103, 105, 106, 108, 109, 111, 112, 114, 115, 117, 119, 120, 122, 123,
	125, 127, 128, 130, 132, 133, 135, 137, 138, 140, 142, 144, 145, 147, 149, 151,
	153, 155, 156, 158, 160, 162, 164, 166, 168, 170, 172, 174, 176, 178, 180, 182,
	184, 186, 188, 190, 192, 194, 197, 199, 201, 203, 205, 207, 210, 212, 214, 216,
	219, 221, 223, 226, 228, 230, 233, 235, 237, 240, 242, 245, 247, 250, 252, 255
};

const uint8_t channelPins[NUM_CHANNELS] = {
	CHANNEL_PIN_0,
	CHANNEL_PIN_1,
	CHANNEL_PIN_2,
	CHANNEL_PIN_3,
	CHANNEL_PIN_4,
	CHANNEL_PIN_5
};

global uint8_t channelValues[NUM_CHANNELS] = {0, 0, 0, 0, 0, 0};

static inline void channelOut( channelLerp_t *channel ) {
	channelValues[channel->id]	= channel->result;
}

setup( ) {
	make_output_pin( CHANNEL_PIN_0 );
	make_output_pin( CHANNEL_PIN_1 );
	make_output_pin( CHANNEL_PIN_2 );
	make_output_pin( CHANNEL_PIN_3 );
	make_output_pin( CHANNEL_PIN_4 );
	make_output_pin( CHANNEL_PIN_5 );
	
	setup_timer( TIMER_Clock );
	
	start_interrupts( );

	start_waveforming( );
}

on_timer( ) {
	static uint8_t windowCounter = 0;
	
	uint8_t i;

	for ( i = 0; i < NUM_CHANNELS; i++ ) {
		if ( windowCounter >= pgm_read_byte(gammaTable + channelValues[i]) ) {
			pin_off( channelPins[i] );
		} else {
			pin_on( channelPins[i] );
		}		
	}

	windowCounter = (windowCounter + 1) % PWM_WINDOW;
}

loop( ) {
	sleep( );
}

