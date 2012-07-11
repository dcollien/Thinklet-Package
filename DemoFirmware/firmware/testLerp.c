#include <stdio.h>
#include <stdint.h>

#define TRUE 1
#define FALSE 0

#define LOOP_FLAG 1
#ifndef PROGMEM
#define PROGMEM
#endif

uint8_t waitTime;

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


void brenthamInit( channelLerp_t *channel ) {

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

void brenthamIteration( channelLerp_t *channel ) {
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

void channelOut( channelLerp_t *channel ) {
	printf( "channel %d set to: %d\n", channel->id, channel->result );
}

void wait( ) {
	printf( "wait %d\n", waitTime );
}

void startChannelLerp( channelLerp_t *channel ) {
	channel->x0 = 0;
	channel->y0 = channel->wave[channel->waveIndex];

	channel->x1 = channel->wave[channel->waveIndex+1];
	channel->y1 = channel->wave[channel->waveIndex+2];

	brenthamInit( channel );

	channel->first = TRUE;


	channel->waveIndex += 2;
	channel->done = FALSE;
}


uint8_t lerpStepChannel( channelLerp_t *channel ) {
	channel->x_prev = channel->x0;
	channel->y_prev = channel->y0;

	// skip over all brightnesses which are in the same time step
	while ( channel->x_prev == channel->x0 ) {
		brenthamIteration( channel );

		if ( channel->x0 == channel->x1 && channel->y0 == channel->y1 ) {
			channel->done = TRUE;
			break;
		}
	}

	// if we're done on this interpolation, generate a result
	if ( !channel->done ) {
		if ( channel->first ) {
			channel->first = FALSE;
			channel->result = channel->y_prev;
		} else {
			channel->result = (channel->y0+channel->y_prev)>>1;
		}

		channelOut( channel );
	}

	// return if successfully set an output
	return !channel->done;
}


uint8_t stepChannel( channelLerp_t *channel ) {

	if ( channel->flags & LOOP_FLAG && channel->waveIndex >= channel->length ) {
		// reset on looping
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

	wait( );

	return (numFinished == numChannels);
}


int main( int argc, char *argv[] ) {
	uint8_t numChannels = channelData[1];
	uint8_t i;
	uint16_t offset;

	uint8_t debug_i;

	waitTime = channelData[0];
	printf( "Number of channels: %d\n", numChannels );

	channelLerp_t channels[numChannels];

	offset = 2;
	for ( i = 0; i < numChannels; i++ ) {
		channels[i].id = i;
		channels[i].length = (channelData[offset] << 8) | channelData[offset+1];
		channels[i].waveIndex = 0;
		channels[i].flags = channelData[offset+2];
		channels[i].wave = &(channelData[offset+3]);
		offset += channels[i].length + 3;


		printf( "Waveform %d\n", i );
		printf( "  length: %d\n", channels[i].length );

		for ( debug_i = 0; debug_i < channels[i].length; debug_i++ ) {
			printf( "   %d\n", channels[i].wave[debug_i] );
		}

		startChannelLerp( &channels[i] );
	}

	while ( !stepAllChannels( channels, numChannels ) );

	return 0;
}

