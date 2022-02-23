// sdrplayalsa.c
// 202110 - Created by Gary W.
// 20211028 - Working AGC block - Gary C. and Clint T.
// 20211029 - Added gain step size parameters -s and -S (decrease and increase attenuation, respectively)
// 20211107 - Added -e <gainfile> argument to output gain reduction value to a file for use in WebSDR
// 20211109 - Modified gain reduction value in file to be relative based on minimum gain (e.g. "gain_reduction-min_gain_reduction")
// 20220222 - Added decimation, shift values and validation to properly configure for rates 96, 192, 384 and 768k.  A rate of 1536k is validated, but is not currently supported by ALSA, but included if someone wanted to take a crack at recompiling ALSA to allow it.  [Clint]
// 20220223 - Modified so that when a gainfile is specified, a value of "0" (zero) is written to it at start-up.

#define _GNU_SOURCE
#include <alloca.h>
#include <alsa/asoundlib.h>
#include <sdrplay_api.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

static int gain_reduction = 30; // gain reduction
static int min_gain_reduction = 30;  // this version used to hold onto command-line specified gain values for AGC control
static int max_gain_reduction = 59;  // used to set maximum amount of gain reduction on command line
static snd_pcm_t *pcm;
static int AGCEnable = 0;
static int AGC1increaseThreshold = 16384;
static int AGC2decreaseThreshold = 8192;
static int AGC3minTimeMs = 500;
static int AGC4A = 4096;
static int AGC5B = 1000;
static int AGC6C = 5000;
static int agc_timer_scaling;
static int max_adc = 0;
static int agc_increase_timer = 0;
static int agc_decrease_timer = 0;
static int agc_timer = 0;
static sdrplay_api_DeviceParamsT *dp;
static sdrplay_api_CallbackFnsT callbacks;
static sdrplay_api_DeviceT devices[ 8 ];
static unsigned numdevices;
static int devind = 0;
static int verbose = 0;
static sdrplay_api_TunerSelectT tuner = sdrplay_api_Tuner_Both; // defined in /usr/local/include/sdrplay_api_tuner.h
static int debugPeriod = 0;
static int counter_ms = 0;
static int debug_counter_ms = 0;
static int counter_samples = 0;
static int adc_high_count = 0;
static int wbs = 0;
static int bwtype = 1536;
static int gainstep_inc = 1;	// step size to increase attenuation
static int gainstep_dec = 1;    // step size to decrease attenuation
static FILE *gainfp = NULL;


void update_sdrplay_gain_reduction() {
    int ret;
    sdrplay_api_ReasonForUpdateT reasonForUpdate = sdrplay_api_Update_Tuner_Gr;
    sdrplay_api_ReasonForUpdateExtension1T reasonForUpdateExt1 = sdrplay_api_Update_Ext1_None;

    if (verbose) {
	fprintf(stderr, "updating gain_reduction to %d, agc_timer is %d\n", gain_reduction, agc_timer);
    }

    dp->rxChannelA->tunerParams.gain.gRdB = gain_reduction;
    ret = sdrplay_api_Update( devices[ devind ].dev, tuner, reasonForUpdate, reasonForUpdateExt1);
    if(ret) {
	fprintf( stderr, "Error response from sdr_api_Update: %s\n", sdrplay_api_GetErrorString( ret ) );
    } else {
	if (verbose) {
	    fprintf(stderr, "Successful response from sdr_api_Update\n");
	}
    }
    if (gainfp) {
	fseek(gainfp, 0, SEEK_SET);
	fprintf(gainfp, "%d\n", gain_reduction-min_gain_reduction);
	fflush(gainfp);
    }
}


void agc(short *buf, sdrplay_api_StreamCbParamsT *params, unsigned numSamples) {
    int adc_result, abs_adc, i;
    int gain_changed = 0;
	
    for (i = 0; i < numSamples; i++) {
	counter_samples ++;
	if (counter_samples > agc_timer_scaling) {
	    counter_ms++;
	    debug_counter_ms++;
	    counter_samples = 0;
	    agc_timer++; // this is the timer for the AGC loop;
	    agc_increase_timer++;
	    agc_decrease_timer++;
	}
	adc_result = buf[i];
	abs_adc = abs(adc_result);

	if(abs_adc > max_adc) max_adc = abs_adc;	// get high water mark
	
	if(abs_adc > AGC1increaseThreshold)		// is A/D value above high signal level?
	    if(adc_high_count < 65530) adc_high_count++;	// yes - bump count, prevent overflow

	if(agc_timer >= AGC3minTimeMs) { // we can look at the AGC timing again since it has been long enough
	    // (do increase threshold count timing, etc. and decrease gain as necessary)
	    // if it has been long enough since we did an increase - AND is there a minimum number of A/D conversions above the last
	    if((agc_increase_timer > AGC5B) && (adc_high_count > AGC4A)) {
		// is the current "gain reduction" value below the limit?
		if(gain_reduction < AGC1increaseThreshold) {
		    // yes - decrease the gain by one step
		    gain_reduction+=gainstep_inc;
		    if(gain_reduction > 59) gain_reduction = max_gain_reduction;	// limit maximum amount of gain reduction
		    gain_changed = 1;
		    agc_increase_timer = 0;  // reset timer for gain increase
		    agc_decrease_timer = 0;  // also reset AGC decrease timer because we just did an increase
		}
	    } 
	    else if (max_adc < AGC2decreaseThreshold) {
		// (do decrease threshold count timing, etc. and increase gain as necessary)
		if( (agc_decrease_timer > AGC6C) ) {
		    // yes - is the current "gain reduction" above the limit?
		    if(gain_reduction > min_gain_reduction) {	// prevent gain reduction from being set lower than explicitly specified in command line
			// yes - increase the gain by one step
			gain_reduction-=gainstep_dec;
			gain_changed = 1;
			agc_increase_timer = 0; // reset the gain adjustment timers
			agc_decrease_timer = 0;
			adc_high_count = 0;
		    }
		}
	    }
	    max_adc = 0; // reset the ADC high-water value before the next AGC sampling window starts
	    agc_timer = 0;
	    adc_high_count = 0;	// end of timing window - reset high count for next time.
	}

	if(debugPeriod > 0) {
	    if (debug_counter_ms > debugPeriod) {
		debug_counter_ms = 0;
		fprintf(stderr, "DEBUG: agc_timer=%d, gain_reduction=%d, abs_adc=%d, max_adc=%d, gain_changed=%d, adc_high_count=%u\n", agc_timer, gain_reduction, abs_adc, max_adc, gain_changed, adc_high_count);
	    }
	}

	if (gain_changed) {
	    update_sdrplay_gain_reduction();
	    gain_changed = 0;
	}
    }
}


void rx( short *xi, short *xq, sdrplay_api_StreamCbParamsT *params,
	 unsigned numSamples, unsigned reset, void *cbContext ) {

    short *buf = alloca( numSamples * 2 * sizeof (short) );
    short *p;
    int i;
    int ret;
    ssize_t write_return_value;

#if 1
    // already decimated
    for( i = 0, p = buf; i < numSamples; i++, p += 2 ) {
	p[ 0 ] = *xi++;
	p[ 1 ] = *xq++;
    }
#else
    // we do the decimation
    // FIXME antialias first!
    numSamples >>= 2;
    for( i = 0, p = buf; i < numSamples; i++, p += 2 ) {
	p[ 0 ] = *xi;
	p[ 1 ] = *xq;
	xi += 4;
	xq += 4;
    }
#endif

    if(AGCEnable) {
	agc( buf, params, numSamples );
    }

    if( pcm ) {
	if( ( ret = snd_pcm_writei( pcm, buf, numSamples ) ) < 0 ) {
	    if( ret == -EAGAIN )
		return;

	    if( ret != -EPIPE )
		fprintf( stderr, "snd_pcm_writei: %s\n", snd_strerror( ret ) );
	
	    if( ( ret = snd_pcm_prepare( pcm ) < 0 ) )
		fprintf( stderr, "snd_pcm_prepare: %s\n", snd_strerror( ret ) );

	    // prime the pump
	    for( i = 0; i < 4; i++ )
		if( ( ret = snd_pcm_writei( pcm, buf, numSamples ) ) < 0 )
		    fprintf( stderr, " snd_pcm_writei: %s\n",
			     snd_strerror( ret ) );
	}
    } else {
	write_return_value = write( 1, buf, numSamples << 2 );
	if (!write_return_value) {
	    fprintf( stderr, "write returned 0\n");
	}
    }
}

void event() {
}

static void usage( char *argv0 ) {

    fprintf( stderr, "usage: %s [options...]\n"
	     "options:\n"
	     "    -a inc   AGC \"increase\" threshold, default 16384\n"
	     "    -B bwType baseband low-pass filter type (200, 300, 600, 1536, 5000)\n"
	     "    -b dec   AGC \"decrease\" threshold, default 8192\n"
	     "    -c min   AGC sample period (ms), default 500\n"
	     "    -d       list available input/output devices\n"
	     "    -e gainfile  write gain_reduction value to file\n"
	     "    -f freq  set tuner frequency (in Hz)\n"
	     "    -g gain  set min gain reduction during AGC operation or fixed gain w/AGC disabled, default 30\n"
	     "    -G gain  set max gain reduction during AGC operation, default 59\n"
	     "    -h       show usage\n"
	     "    -i ser   specify input device (serial number)\n"
	     "    -l val   set LNA state, default 3.  See SDRPlay API gain reduction tables for more info\n"
	     "    -n       AGC enable, uses parameters a,b,c,g,s,S,x,y,z\n"
	     "    -o dev   specify output device\n"
	     "    -r rate  set sampling rate (in Hz) [Must be 96000, 192000, 384000 or 768000]\n"
	     "    -S step_inc  set gain AGC attenuation increase (gain reduction) step size in dB, default = 1 (1-10)\n"
	     "    -s step_dec  set gain AGC attenuation decrease (gain gain increase) step size in dB, default = 1 (1-10)\n"
	     "    -t taps  set number of antialias FIR taps, default = 9\n"
	     "    -v       enable verbose output\n"
	     "    -W       enable wideband signal mode (e.g. half-band filtering). Warning: High CPU useage!\n"
	     "    -w debugPeriodMs    warning/debug output period (ms)\n"
	     "    -x A     num conversions for overload, default 4096\n"
	     "    -y B     gain decrease event time (ms), default 1000\n"
	     "    -z C     gain increase event time (ms), default 5000\n", argv0 );
}

static void setopt( int *p, char *optarg, char *argv0 ) {

    char *endp;

    *p = strtol( optarg, &endp, 0 );

    if( p < 0 || !*optarg || *endp ) {
	usage( argv0 );
	exit( 1 );
    }
}

extern int main( int argc, char *argv[] ) {

    int opt;
    static int devlist, bwbad = 0;
    static int freq = 0;
    static char *in;
    static int lna = 3;
    static char *out;
    static char *gainfile;
    static int rate = 0;
    static int taps = 9;
    int ret;
    int i;
    static int decimation = 2;
    static int rateshift = 2;


    while( ( opt = getopt( argc, argv, "a:b:c:de:f:g:hi:l:no:r:s:t:vw:x:y:z:B:W:G:S:" ) ) >= 0 )
	switch( opt ) {
	case 'a':
	    setopt( &AGC1increaseThreshold, optarg, argv[ 0 ] );
	    break;

	case 'b':
	    setopt( &AGC2decreaseThreshold, optarg, argv[ 0 ] );
	    break;

        case 'B':
	    setopt (&bwtype, optarg, argv[ 0 ] );
	    if((bwtype != 200) && (bwtype != 300) && (bwtype != 600) && (bwtype != 1536) && (bwtype != 5000)) bwbad = 1;
	    break;

	case 'c':
	    setopt( &AGC3minTimeMs, optarg, argv[ 0 ] );
	    break;

	case 'd': // list devices
	    devlist = 1;
	    break;
	    
	case 'e': // write gain_reduction to file
	    gainfile = optarg;
	    break;
	    
	case 'f': // frequency
	    setopt( &freq, optarg, argv[ 0 ] );
	    break;
	    
	case 'g': // gain (reduction) - fixed gain, or minimum gain reduction level during AGC operation
	    setopt( &min_gain_reduction, optarg, argv[ 0 ] );
	    //
	    if(min_gain_reduction < 19) min_gain_reduction = 19;
	    else if(min_gain_reduction > max_gain_reduction) min_gain_reduction = max_gain_reduction;
	    gain_reduction = min_gain_reduction;
	    break;
		
	case 'G':  // maximum amount of gain reduction during AGC operation
	    setopt( &max_gain_reduction, optarg, argv[ 0 ] );
	    if(max_gain_reduction < gain_reduction) max_gain_reduction = gain_reduction;	// don't allow setting lower than minimum gain reduction
	    else if(max_gain_reduction > 59) max_gain_reduction = 59;
	    break;
	    
	case 'h': // help
	    usage( argv[ 0 ] );
	    return 0;
	    
	case 'i': // input device (serial number)
	    in = optarg;
	    break;
	    
	case 'l': // lna
	    setopt( &lna, optarg, argv[ 0 ] );
	    break;

	case 'n': // new AGC enable
	    AGCEnable = 1;
	    break;
    	    
	case 'o': // output device
	    out = optarg;
	    break;
	    
	case 'r': // sample rate
	    setopt( &rate, optarg, argv[ 0 ] );
	    break;
	
	case 'S':  // AGC step size for INCREASE of attenuation
	    setopt( &gainstep_inc, optarg, argv[ 0 ] );
            if(gainstep_inc < 1) gainstep_inc = 1;
            else if(gainstep_inc > 10) gainstep_inc = 10;
	    break;

	case 's':  // AGC step size for DECREASE of attenuation
	    setopt( &gainstep_dec, optarg, argv[ 0 ] );
            if(gainstep_dec < 1) gainstep_dec = 1;
            else if(gainstep_dec > 10) gainstep_dec = 10;
	    break;

    	case 'W': // Wideband Signal mode
	    wbs = 1;
            break;
	    
	case 't': // FIR taps
	    setopt( &taps, optarg, argv[ 0 ] );
	    break;
	    
	case 'v': // verbose
	    verbose = 1;
	    break;

	case 'w':
	    setopt( &debugPeriod, optarg, argv[ 0 ] );
	    break;

	case 'x':
	    setopt( &AGC4A, optarg, argv[ 0 ] );
	    break;

	case 'y':
	    setopt( &AGC5B, optarg, argv[ 0 ] );
	    break;

	case 'z':
	    setopt( &AGC6C, optarg, argv[ 0 ] );
	    break;

	default:
	    usage( argv[ 0 ] );
	    return 1;
	}

    if( ( ret = sdrplay_api_Open() ) ) {
	fprintf( stderr, "sdr_api_Open: %s\n", sdrplay_api_GetErrorString( ret ) );
	return 1;
    }

    agc_timer_scaling = rate / 1000;

    if (verbose && AGCEnable) {
	fprintf(stderr, "enabled AGC with\n  AGC1increaseThreshold=%d,\n  AGC2decreaseThreshold=%d,\n  AGC3minTimeMs=%d,\n"
		"  AGC4A=%d,\n  AGC5B=%d,\n  AGC6C=%d\n", AGC1increaseThreshold, AGC2decreaseThreshold, AGC3minTimeMs, AGC4A, AGC5B, AGC6C);
	fprintf(stderr, "agc_timer_scaling = %d\n", agc_timer_scaling);
    }
	
    sdrplay_api_DebugEnable( NULL, verbose );
    sdrplay_api_LockDeviceApi();
    sdrplay_api_GetDevices( devices, &numdevices, 8 );

    if( devlist ) {
	fputs( "Available input devices:\n", stderr );
	fprintf( stderr, "    %d devices available:\n", numdevices );
	for( i = 0; i < numdevices; i++ )
	    fprintf( stderr, "    %s (%d)\n", devices[ i ].SerNo, devices[ i ].hwVer );

	// FIXME also show ALSA devices

	return 0;
    }
    
    if( !numdevices ) {
	fprintf( stderr, "%s: no suitable input devices found\n", argv[ 0 ] );
	return 1;
    }

    if( bwbad )	{
	fprintf( stderr, "%s: Invalid bandwidth specified\n", argv[ 0 ] );
	return 1;
    }

    if( !freq ) {
	fprintf( stderr, "%s: No frequency specified\n", argv[ 0 ] );
	return 1;
    }
    
    if( !rate ) {
	fprintf( stderr, "%s: No sample rate specified\n", argv[ 0 ] );
	return 1;
    }

    if((rate != 96000) && (rate != 192000) && (rate != 384000) && (rate != 768000))  {
	fprintf( stderr, "%s: Invalid sample rate specified\n", argv[ 0 ] );
	return 1;
    }
    
    for( i = 0; i < numdevices; i++ )
	if( in && strcasestr( devices[ i ].SerNo, in ) )
	    devind = i;

    if( in && !strcasestr( devices[ devind ].SerNo, in ) ) {
	fprintf( stderr, "%s: device %s not found\n", argv[ 0 ], in );
	return 1;
    }

    if( ( ret = sdrplay_api_SelectDevice( devices + devind ) ) ) {
	fprintf( stderr, "sdr_api_SelectDevice: %s\n",
		 sdrplay_api_GetErrorString( ret ) );
	return 1;
    }

    sdrplay_api_UnlockDeviceApi();
    
    if( out ) {
	if( ( ret = snd_pcm_open( &pcm, out, SND_PCM_STREAM_PLAYBACK,
				  0 ) ) < 0 ) {
	    fprintf( stderr, "snd_pcm_open: %s\n", snd_strerror( ret ) );
	    return 1;
	}

	snd_pcm_nonblock( pcm, SND_PCM_NONBLOCK );
    
	if( ( ret = snd_pcm_set_params( pcm, SND_PCM_FORMAT_S16_LE,
					SND_PCM_ACCESS_RW_INTERLEAVED, 2,
					rate, 0, 30000 ) ) < 0 ) {
	    fprintf( stderr, "snd_pcm_set_params: %s\n", snd_strerror( ret ) );
	    return 1;
	}

	if( ( ret = snd_pcm_prepare( pcm ) ) < 0 ) {
	    fprintf( stderr, "snd_pcm_prepare: %s\n", snd_strerror( ret ) );
	    return 1;
	}
    }
    
    if( ( ret = sdrplay_api_GetDeviceParams( devices[ devind ].dev, &dp ) ) ) {
	fprintf( stderr, "sdr_api_GetDeviceParams: %s\n",
		 sdrplay_api_GetErrorString( ret ) );
	return 1;
    }
    
    if( gainfile ) {
	if( 0 == ( gainfp = fopen( gainfile, "w" ) )  ) {
	    fprintf( stderr, "gainfile fopen: %s\n", snd_strerror( ret ) );
	    return 1;
	}
        else {	// Init gain file with zero value
	    fseek(gainfp, 0, SEEK_SET);
	    fprintf(gainfp, "0\n");
	    fflush(gainfp);
        }
    }

    // Determine appropriate decimation rate

    if(rate == 96000)	{
	decimation = 32;
	rateshift = 5;
    }
    else if(rate == 192000)	{
	decimation = 16;
	rateshift = 4;
    }
    else if(rate == 384000)	{
	decimation = 8;
	rateshift = 3;
    }
    else if(rate == 768000)	{
        decimation = 4;
	rateshift = 2;
    }
    else if(rate == 1536000)	{
	decimation = 2;
	rateshift = 1;
    }

    dp->devParams->fsFreq.fsHz = rate << rateshift;
    dp->rxChannelA->tunerParams.rfFreq.rfHz = freq;
    dp->rxChannelA->tunerParams.bwType = bwtype;
    dp->rxChannelA->tunerParams.ifType = 0;
    dp->rxChannelA->tunerParams.gain.gRdB = gain_reduction;
    dp->rxChannelA->tunerParams.gain.LNAstate = lna;
#if 1
    dp->rxChannelA->ctrlParams.decimation.enable = 1;
    dp->rxChannelA->ctrlParams.decimation.decimationFactor = decimation;
    dp->rxChannelA->ctrlParams.decimation.wideBandSignal = wbs;
#endif
    dp->rxChannelA->ctrlParams.agc.enable = 0;

    callbacks.StreamACbFn = rx;
    callbacks.EventCbFn = event;

    fprintf( stderr, "   BWType value:  %u\n", bwtype );
    fprintf( stderr, "   WBS value:  %u (0=off, 1=0n) \n", wbs );
    fprintf( stderr, "   AGC gain reduction step size:  %u dB (0=off, 1=0n) \n", gainstep_inc );
    fprintf( stderr, "   AGC gain increase step size:  %u dB (0=off, 1=0n) \n", gainstep_dec );
    fprintf( stderr, "   Sample rate:  %u  (Decimation: %u  Shift: %u) \n", rate, decimation, rateshift );
    
    if( ( ret = sdrplay_api_Init( devices[ devind ].dev, &callbacks, NULL ) ) ) {
	fprintf( stderr, "sdr_api_Init: %s\n", sdrplay_api_GetErrorString( ret ) );
	return 1;
    }
    
    update_sdrplay_gain_reduction();

    for(;;)
	pause();
}
