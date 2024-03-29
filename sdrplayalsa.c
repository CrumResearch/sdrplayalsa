// sdrplayalsa.c
// 202110 - Created by Gary W.
// 20211028 - Working AGC block - Gary C. and Clint T.
// 20211029 - Added gain step size parameters -s and -S (decrease and increase attenuation, respectively)
// 20211107 - Added -e <gainfile> argument to output gain reduction value to a file for use in WebSDR
// 20211109 - Modified gain reduction value in file to be relative based on minimum gain (e.g. "gain_reduction-min_gain_reduction")
// 20220222 - Added decimation, shift values and validation to properly configure for rates 96, 192, 384 and 768k.  A rate of 1536k is validated, but is not currently supported by ALSA, but included if someone wanted to take a crack at recompiling ALSA to allow it.  [Clint]
// 20220223 - Modified so that when a gainfile is specified, a value of "0" (zero) is written to it at start-up.  [Clint]
// 20220304 - Commented out FIR taps option;  Added -R parameter to allow more explicit specification of the raw ADC sample rate and added to the STDOUT the calculated (raw) rate;  Added error trapping.  [Clint]
// 20230210 - Added "lockout" of the AGC (gain) adjustment based on the value of "params->grChanged".  Its use is undocumented in the API but its use was noted in an email by Frank, K4VZ based on correspondence with Andy Carpenter, one of the authors of the API.  Also fixed issue where blank command line was not causing "usage" to be displayed.  Added "-L" parameter to set latency (in uSec) when used with the "-o" parameter to use a sound device rather than STDIO.  These changes were made to allow testing to reduce the "stutter" issue that can occur on the WebSDRs.  Also added SIGNINT function to allow the API to be shut down gracefully, hopefully reducing the need to do a "sudo system ctl restart sdrplay" to restart it when it was simply killed.
// 20220214 - Added more graceful shutdown of all SDRPLay API processes;  Moved gain control (API) to end of RX callback so that it occurs AFTER all buffer copying;  Configured timed callback (100 msec) to poll to see if a new value is to be written to the gain file:  This moves the file write outside of the time-critical RX callback function in the event that a file-write blocks the process and upsets the callback timing and interfacing with the API.

#define _GNU_SOURCE
#include <alloca.h>
#include <alsa/asoundlib.h>
#include <sdrplay_api.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <signal.h>
#include <time.h>
#include <sys/time.h>

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
static int bulkmode = 0;
static int bwtype = 1536;
static int gainstep_inc = 1;	// step size to increase attenuation
static int gainstep_dec = 1;    // step size to decrease attenuation
static FILE *gainfp = NULL;
static int gchange_lockout=0;	// used to lock-out gain changes when API is busy
static char *in_dev;
static int gain_changed = 0;
static char sernum[64];
static int gainfile_flag = 0;

// Do gain update for SDRPlay device
void update_sdrplay_gain_reduction() {
	int ret;
//	char s[16];
//	int l=0;

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

	gainfile_flag = 1;	// Signal to callback that a new gain value is ready to be written

/*
    if (gainfp) {	// Write new gain value to file
		sprintf(s, "%d", gain_reduction-min_gain_reduction);
		l=strlen(s);
		fseek(gainfp, 0, SEEK_SET);
//		fprintf(gainfp, "%d\n", gain_reduction-min_gain_reduction);
		fwrite(s,1,l,gainfp);
		fflush(gainfp);
    }
*/
}

// Process AGC based in samples
void agc(short *buf, sdrplay_api_StreamCbParamsT *params, unsigned numSamples) {
    int adc_result, abs_adc, i;
	
    for (i = 0; i < numSamples; i++) {
		counter_samples ++;
		if (counter_samples > agc_timer_scaling) {	// update AGC "window" timers
	    	counter_ms++;
		    debug_counter_ms++;
		    counter_samples = 0;
	    	agc_timer++; // this is the timer for the AGC loop;
		    agc_increase_timer++;
		    agc_decrease_timer++;
		}

		// determine absolute amplitude of current sample
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

			    	if(gain_reduction > 59)	// Gain reduction at maximum?
						gain_reduction = max_gain_reduction;	// limit maximum amount of gain reduction
					else	// Do API gain change only if valid value
					    gain_changed = 1;

				    agc_increase_timer = 0;  // reset timer for gain increase
				    agc_decrease_timer = 0;  // also reset AGC decrease timer because we just did an increase
				}
		    } 
		    else if (max_adc < AGC2decreaseThreshold) {  // (do decrease threshold count timing, etc. and increase gain as necessary)
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

/*
		if (gain_changed) {		// are we to change gain?
			if(!gchange_lockout)	{	// bail out if gain change is in process - we can wait until later
		    	gain_changed = 0;		// indicate that we (will) have changed gain
				gchange_lockout=1;		// set lockout to prevent another gain change until we know API is ready
			    update_sdrplay_gain_reduction();	// update SDRPlay device
			}
		}
*/
    }
}


void rx( short *xi, short *xq, sdrplay_api_StreamCbParamsT *params, unsigned numSamples, unsigned reset, void *cbContext ) {

    short *buf = alloca( numSamples * 2 * sizeof (short) );
    short *p;
    int i;
    int ret;
    ssize_t write_return_value;
    static int grChanged_flag = 0;
	static unsigned reset_flag = 99;

	// Set lock-outs for AGC gain changes

	if(params->grChanged)	{	// has "grChanged" gone non-zero indicating an operation in process?
		grChanged_flag = 1;		// yes
		gchange_lockout = 1;	// unconditionally set lockout to prevent gain reduction call
	}
	else if(grChanged_flag)	{	// has grChanged gone BACK to zero after being nonzero?
		grChanged_flag = 0;		// yes - clear detect flag
		gchange_lockout = 0;	// clear gain change lockout
	}

	if(reset != reset_flag)	{	// Indicate change in status of the "reset" flag from the API
		fprintf( stderr, "API reset Flag is now %u\n", reset);
		reset_flag = reset;
	}

#if 1
    // already decimated
    for( i = 0, p = buf; i < numSamples; i++, p += 2 ) {	// Copy samples to local buffer
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

    if(AGCEnable) {		// send samples to our own AGC function if enabled
		agc( buf, params, numSamples );
    }


    if( pcm ) {		// Send samples to audio (ALSA) device
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
    } 
	else {		// Send samples to STDOUT
		write_return_value = write( 1, buf, numSamples << 2 );
		if (!write_return_value) {
		    fprintf( stderr, "write returned 0\n");
		}
    }
	//

	if (gain_changed) {		// are we to change gain?
		if(!gchange_lockout)	{	// bail out if gain change is in process - we can wait until later
	    	gain_changed = 0;		// indicate that we (will) have changed gain
			gchange_lockout=1;		// set lockout to prevent another gain change until we know API is ready
		    update_sdrplay_gain_reduction();	// update SDRPlay device
		}
	}

}

void event() {
}

static void usage( char *argv0 ) {

    fprintf( stderr, "usage: %s [options...]\n"
	     "options:\n"
	     "    -a inc   AGC \"increase\" threshold, default 16384\n"
	     "    -B bwType baseband low-pass filter bandwidth (200, 300, 600, 1536, 5000 kHz)\n"
	     "    -b dec   AGC \"decrease\" threshold, default 8192\n"
	     "    -c min   AGC sample period (ms), default 500, minimum 50\n"
	     "    -d       list available input/output devices\n"
	     "    -e gainfile  write gain_reduction value to file\n"
	     "    -f freq  set tuner frequency (in Hz)\n"
	     "    -g gain  set min gain reduction during AGC operation or fixed gain w/AGC disabled, default 30\n"
	     "    -G gain  set max gain reduction during AGC operation, default 59\n"
	     "    -h       show usage\n"
	     "    -i ser   specify input SDRPlay device by serial number (full or partial)\n"
	     "    -l val   set LNA state, default 3.  See SDRPlay API gain reduction tables for more info\n"
         "    -L latency in microseconds - Used only with '-o' parameter - must be >=30000, default 50000\n"
	     "    -n       AGC enable, uses parameters a,b,c,g,s,S,x,y,z\n"
	     "    -o dev   specify output device (Use with '-L' parameter) \n"
	     "    -r rate  set sampling rate (in Hz) [Must be 96000, 192000, 384000 or 768000 unless '-R' is specified]\n"
         "    -R rexp  If specified, use with '-r' to set decimation and sample rate:  Choose 'rexp' so that 'rate * 2^rexp' is >=2.048 and <8.064 Msamples/sec:  Decimation is 2^rexp (Must be 0-5)\n"
	     "    -S step_inc  set gain AGC attenuation increase (gain reduction) step size in dB, default = 1 (1-10)\n"
	     "    -s step_dec  set gain AGC attenuation decrease (gain increase) step size in dB, default = 1 (1-10)\n"
//	     "    -t taps  set number of antialias FIR taps, default = 9\n"
	     "    -v       enable verbose output\n"
	     "    -W       enable wideband signal mode (e.g. half-band filtering). Warning: High CPU useage! (May not work)\n"
	     "    -w debugPeriodMs    warning/debug output period (ms)\n"
		 "    -X       Set to USB Xfer mode to BULK rather than Isochronous \n"
	     "    -x A     num of A/D samples above threshold (-a parameter) before detection, default 4096\n"
	     "    -y B     gain decrease event time (ms), default 1000, minimum 50\n"
	     "    -z C     gain increase event time (ms), default 5000, minimum 50\n\n", argv0 );
}

static void setopt( int *p, char *optarg, char *argv0 ) {

    char *endp;

    *p = strtol( optarg, &endp, 0 );

    if( p < 0 || !*optarg || *endp ) {
	usage( argv0 );
	exit( 1 );
    }
}

void term(int signum)	{		// termination signal handler
int ret;
int err = 0;

	ret = sdrplay_api_Uninit((devices+devind)->dev);

	if(ret != sdrplay_api_Success)	{
		fprintf(stderr, "SDRPlay uninit failed");
		err = 1;
	}
	else
		fprintf(stderr, "SDRPlay uninit successful");

	fprintf(stderr, " for device %s\n", devices[devind].SerNo);

	ret = sdrplay_api_ReleaseDevice( devices + devind );

	if(ret != sdrplay_api_Success)	{
		fprintf(stderr, "SDRPlay Device release failed");
		err = 1;
	}
	else
		fprintf(stderr, "SDRPlay Device release successful");

	fprintf(stderr, " for device %s\n", devices[devind].SerNo);

	ret = sdrplay_api_UnlockDeviceApi();

	if(ret != sdrplay_api_Success)	{
		fprintf(stderr, "SDRPlay Device unlock failed");
		err = 1;
	}
	else
		fprintf(stderr, "SDRPlay Device unlock successful");

	fprintf(stderr, " for device %s\n", devices[devind].SerNo);

	ret = sdrplay_api_Close();	// close sdrplay API as gracefully as possible

	if(ret != sdrplay_api_Success)	{
		fprintf(stderr, "SDRPlay Device close failed");
		err = 1;
	}
	else
		fprintf(stderr, "SDRPlay Device close successful for");

	fprintf(stderr, " for device %s\n", devices[devind].SerNo);

	if(!err)	// no error
		exit (0);
	else	// error in closing
		exit(1);
}



// Polling to write gain values when updated, independent of other callback processes

void timer_callback(int signum)
{
	if(gainfile_flag)	{	// is there new gain file data to write?
    	if (gainfp) {	// Write new gain value to file
//			sprintf(s, "%d", gain_reduction-min_gain_reduction);
//			l=strlen(s);
			fseek(gainfp, 0, SEEK_SET);
			fprintf(gainfp, "%d\n", gain_reduction-min_gain_reduction);
//			fwrite(s,1,l,gainfp);
			fflush(gainfp);
	    }
		gainfile_flag = 0;
	}
}



extern int main( int argc, char *argv[] ) {

    struct sigaction action;
    memset(&action, 0, sizeof(action));
    action.sa_handler = term;
    sigaction(SIGTERM, &action, NULL);

    struct itimerval new_timer;
    struct itimerval old_timer;
  
    new_timer.it_value.tv_sec = 0;
    new_timer.it_value.tv_usec = 100;
    new_timer.it_interval.tv_sec = 0;
    new_timer.it_interval.tv_usec = 100 * 1000; // Gain file update is polled every 100 msec
 
    setitimer(ITIMER_REAL, &new_timer, &old_timer);
    signal(SIGALRM, timer_callback); 


    int opt;
    static int devlist, bwbad = 0;
    static int freq = 0;
    static int lna = 3;
    static char *out;
    static char *gainfile;
    static int rate = 0;
    static int taps = 9;
    int ret;
    int i;
    static int decimation = 1;
    static int rateshift = 2;
    static int rateval = -1;
    static int latency_us = 50000;

	if(argc <2)	{			// do usage if no arguments
		usage( argv[ 0 ] );
		return 1;
	}
	

    while( ( opt = getopt( argc, argv, "a:b:c:de:f:g:hi:l:no:r:s:t:vw:x:y:z:B:L:WG:S:R:X" ) ) >= 0 )

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
	    if(min_gain_reduction < 20) min_gain_reduction = 20;  // trap invalid value
	    else if(min_gain_reduction > max_gain_reduction) min_gain_reduction = max_gain_reduction;
	    gain_reduction = min_gain_reduction;
	    break;
		
	case 'G':  // maximum amount of gain reduction during AGC operation
	    setopt( &max_gain_reduction, optarg, argv[ 0 ] );
	    if(max_gain_reduction < gain_reduction) max_gain_reduction = gain_reduction;	// don't allow setting lower than minimum gain reduction
	    else if(max_gain_reduction > 59) max_gain_reduction = 59;  // trap invalid value
	    break;
	    
	case 'h': // help
	    usage( argv[ 0 ] );
	    return 0;
	    
	case 'i': // input device (serial number)
	    in_dev = optarg;
	    break;
	    
	case 'l': // lna
	    setopt( &lna, optarg, argv[ 0 ] );
	    break;

	case 'L':
		setopt( &latency_us, optarg, argv[ 0 ] );
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

        case 'R': // sample rate decimation exponent
	    setopt( &rateval, optarg, argv [ 0 ] );
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

	case 'X':  // Xfermode = BULK
		bulkmode = 1;
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
		fprintf(stderr, "enabled AGC with\n  AGC1increaseThreshold=%d,\n  AGC2decreaseThreshold=%d,\n  AGC3minTimeMs=%d,\n  AGC4A=%d,\n  AGC5B=%d,\n  AGC6C=%d\n", AGC1increaseThreshold, AGC2decreaseThreshold, AGC3minTimeMs, AGC4A, AGC5B, AGC6C);
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
		fprintf( stderr, "\n%s: no suitable input devices found\n\n", argv[ 0 ] );
		return 1;
    }

    if( bwbad )	{
		fprintf( stderr, "%s: Invalid bandwidth specified - must be 200, 300, 600 or 1536.\n", argv[ 0 ] );
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

    if( ((AGC5B <50) || (AGC6C < 50) || (AGC3minTimeMs < 50)) && (AGCEnable == 1))   {
        fprintf( stderr, "AGC Timing value setting <50 msec - recheck values! \n" );
        return 1;
    }

    if((rateval == -1) && (rate != 96000) && (rate != 192000) && (rate != 384000) && (rate != 768000))  {
		fprintf( stderr, "%s: Invalid sample rate specified\n", argv[ 0 ] );
		return 1;
    }
    
    for( i = 0; i < numdevices; i++ )	{
		if( in_dev && strcasestr( devices[ i ].SerNo, in_dev ) ) {
	   		devind = i;
		}
	}

    if( in_dev && !strcasestr( devices[ devind ].SerNo, in_dev ) ) {
		fprintf( stderr, "%s: device %s not found\n", argv[ 0 ], in_dev );
		return 1;
    }

    if( ( ret = sdrplay_api_SelectDevice( devices + devind ) ) ) {
		fprintf( stderr, "sdr_api_SelectDevice: %s\n", sdrplay_api_GetErrorString( ret ) );
		return 1;
    }

	sprintf(sernum, "%s",devices[devind].SerNo);	// get serial number

    sdrplay_api_UnlockDeviceApi();
    
    if( out ) {	// PCM (ALSA) device specified?
		if( ( ret = snd_pcm_open( &pcm, out, SND_PCM_STREAM_PLAYBACK, 0 ) ) < 0 ) {
		    fprintf( stderr, "snd_pcm_open: %s\n", snd_strerror( ret ) );
		    return 1;
		}

		if(latency_us < 30000) {	// Trap invalid latency setting
			fprintf( stderr,"Specified latency in usec is %u - must be >=30000!\n", latency_us);
			return 1;
		}
		snd_pcm_nonblock( pcm, SND_PCM_NONBLOCK );
    
		if( ( ret = snd_pcm_set_params( pcm, SND_PCM_FORMAT_S16_LE, SND_PCM_ACCESS_RW_INTERLEAVED, 2, rate, 0, latency_us ) ) < 0 ) {
		    fprintf( stderr, "snd_pcm_set_params: %s\n", snd_strerror( ret ) );
		    return 1;
		}

		if( ( ret = snd_pcm_prepare( pcm ) ) < 0 ) {
	    	fprintf( stderr, "snd_pcm_prepare: %s\n", snd_strerror( ret ) );
		    return 1;
		}
    }
    
    if( ( ret = sdrplay_api_GetDeviceParams( devices[ devind ].dev, &dp ) ) ) {
		fprintf( stderr, "sdr_api_GetDeviceParams: %s\n", sdrplay_api_GetErrorString( ret ) );
		return 1;
    }
    
    if( gainfile ) {   // make sure that we can open gain file
		if( 0 == ( gainfp = fopen( gainfile, "w" ) )  ) {   // Cannot open gainfile - error
		    fprintf( stderr, "Cannot open gainfile:  %s\n", snd_strerror( ret ) );
	    	return 1;
		}
        else {	// Init successful - load gain file with zero value to indicate active AGC
		    fseek(gainfp, 0, SEEK_SET);
	    	fprintf(gainfp, "0\n");
		    fflush(gainfp); 
        }
    }


    // Determine appropriate decimation rate if "-R" parameter not specified

    if(rateval == -1)	{	// If no "-R" parameter specified
       if(rate == 96000)	{
	   rateshift = 5;	// 96000 * (2^5) = 3072000 sps ADC rate
       }
       else if(rate == 192000)	{
	   rateshift = 4;	// 192000 * (2^4) = 3072000 sps ADC rate
       }
       else if(rate == 384000)	{
	   rateshift = 3;	// 384000 * (2^3) = 3072000 sps ADC rate
       }
       else if(rate == 768000)	{
	   rateshift = 2;       // 768000 * (2^2) = 3072000 sps ADC rate
       }
    }
    else   {
		rateshift = rateval;
    }

    // Calculate "longhand" so we don't need math.h's "pow()" function just for this

    for(i = 1; i <= rateshift; i++) {
       decimation *= 2;
    }

    if(((rate << rateshift) < 2048000) || ((rate << rateshift) >= 8064000))   {
		fprintf( stderr, "ADC sample rate of [%u*(2^%u)]=%lu out of range! \n", rate, rateshift, (long int)(rate << rateshift));
		return 1;
    }

    dp->devParams->fsFreq.fsHz = rate << rateshift;
	if(bulkmode)
		dp->devParams->mode = sdrplay_api_BULK;

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

    if(strlen(in_dev))
		fprintf( stderr, "For device %s:\n", devices[devind].SerNo);
	//
	fprintf( stderr, "   BWType value:  %u\n", bwtype );
    fprintf( stderr, "   WBS value:  %u (0=off, 1=0n) \n", wbs );
    fprintf( stderr, "   AGC gain reduction step size:  %u dB\n", gainstep_inc );
    fprintf( stderr, "   AGC gain increase step size:  %u dB\n", gainstep_dec );
    fprintf( stderr, "   Sample rate:  %u  (Decimation: %u  Shift: %u) \n", rate, decimation, rateshift );
    fprintf( stderr, "   ADC sample rate:  %lu sps \n",(long int)(rate << rateshift));
	fprintf( stderr, "   USB Transfer is in %s mode \n",(bulkmode ? "Bulk" : "Isochronous") );

	if(out)
		fprintf( stderr, "   Output device: '%s'  Configured latency = %u uSec\n", out, latency_us);
	else
		fprintf( stderr, "   Output using STDIO:  Use '-o' and '-L' parameters to specify audio device and latency in uSec\n");
    
    if( ( ret = sdrplay_api_Init( devices[ devind ].dev, &callbacks, NULL ) ) ) {
		fprintf( stderr, "sdr_api_Init: %s\n", sdrplay_api_GetErrorString( ret ) );
		return 1;
    }
    
//    update_sdrplay_gain_reduction();	

    for(;;)
	pause();
}
