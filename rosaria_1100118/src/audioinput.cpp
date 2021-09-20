#include <cstdio>
#include <cerrno>
#include <cstdlib>
#include <cstring>
#include <cmath>
#include <cinttypes>

#ifdef _WIN32
#include <Windows.h>

void sleep(int n){
    Sleep(-1);
    return;
}

//g++ -I "C:\Program Files (x86)\JACK2\include" audioinput.c "C:\Users\user\Desktop\project\libs\*" -o audioinput

#else
#include <unistd.h>
#endif

#include <jack/jack.h>

#include <ros/ros.h>
#include <std_msgs/Empty.h >

#define WINDOWSIZE 16
#define MAXINDEX 15
#define HIGHTHRES 0.0002
#define LOWTHRES 0.00005
#define TIMEINC 0.001
#define PEAKDIFF 0.01
#define TIMEDIFF 0.02

#define SIMILAR(a, b, d) (fabs((a) - (b)) < (d))

jack_port_t *input_port;
jack_client_t *client;

jack_default_audio_sample_t windowarr[WINDOWSIZE];
jack_default_audio_sample_t *current_in;
jack_default_audio_sample_t last_input;
jack_default_audio_sample_t current_diff;
jack_default_audio_sample_t last_diff;
jack_default_audio_sample_t current_dd;
int windowindex;
float current_time;
float current_ms;


typedef struct peak{
    float start_time;
    float end_time;
    float maximum;
} Peak;

typedef struct gap{
    float start_diff;
    float end_diff;
} Gap;

void initialize_peak(Peak* a){
    a->start_time = 0;
    a->end_time = 0;
    a->maximum = 0;
    return;
}

void initialize_gap(Gap* a){
    a->start_diff = 0;
    a->end_diff = 0;
    return;
}

Peak last_peak;
Peak current_peak;
Gap last_gap;
Gap current_gap;
int gap_validity;
int current_state;

/*
int similar(float* a, float* b, float diff){
    return abs((*a) - (*b)) < diff;
}
*/

int
process (jack_nframes_t nframes, void *arg)
{
	current_in = (jack_default_audio_sample_t*) jack_port_get_buffer (input_port, nframes);
	current_ms -= windowarr[windowindex];
	current_diff = (*current_in) - last_input;
	current_dd = current_diff - last_diff;
	windowarr[windowindex] = current_dd * current_dd;
	last_input = (*current_in);
	last_diff = current_diff;
	current_ms += windowarr[windowindex];
	windowindex = (windowindex == MAXINDEX)? 0:(windowindex+1);
	current_time += TIMEINC;

	//printf("%f\n", current_ms);



	if(current_state == 0){
        if(current_ms > HIGHTHRES){
            current_state = 1;
            current_peak.start_time = current_time;
        }
	}else{
        if(current_ms < LOWTHRES){
            current_state = 0;
            current_peak.end_time = current_time;
            if(SIMILAR(last_peak.maximum, current_peak.maximum, PEAKDIFF)){
                if(gap_validity){
                    current_gap.start_diff = current_peak.start_time - last_peak.start_time;
                    current_gap.end_diff = current_peak.start_time - last_peak.start_time;
                    if(SIMILAR(current_gap.start_diff, last_gap.start_diff, TIMEDIFF) && SIMILAR(current_gap.end_diff, last_gap.end_diff, TIMEDIFF)){
                        printf("valid ");
                        //output true;
                    }
                    last_gap.start_diff = current_gap.start_diff;
                    last_gap.end_diff = current_gap.end_diff;
                }else{
                    last_gap.start_diff = current_peak.start_time - last_peak.start_time;
                    last_gap.end_diff = current_peak.start_time - last_peak.start_time;
                }
                gap_validity = 1;
            }else{
                gap_validity = 0;
            }
            last_peak.start_time = current_peak.start_time;
            last_peak.maximum = current_peak.maximum;
            last_peak.end_time = current_peak.end_time;
            printf("peak: %f, time: %f\n", current_peak.maximum, current_time);
            initialize_peak(&current_peak);
        }else{
            if(current_ms > current_peak.maximum){
                current_peak.maximum = current_ms;
            }
        }
	}

	return 0;
}

void
jack_shutdown (void *arg)
{
	exit (1);
}


int
main (int argc, char *argv[])
{
	const char **ports;
	const char *client_name = "simple";
	const char *server_name = NULL;
	jack_options_t options = JackNullOption;
	jack_status_t status;

	/* open a client connection to the JACK server */

	client = jack_client_open (client_name, options, &status, server_name);
	if (client == NULL) {
		fprintf (stderr, "jack_client_open() failed, "
			 "status = 0x%2.0x\n", status);
		if (status & JackServerFailed) {
			fprintf (stderr, "Unable to connect to JACK server\n");
		}
		exit (1);
	}
	if (status & JackServerStarted) {
		fprintf (stderr, "JACK server started\n");
	}
	if (status & JackNameNotUnique) {
		client_name = jack_get_client_name(client);
		fprintf (stderr, "unique name `%s' assigned\n", client_name);
	}

	/* tell the JACK server to call `process()' whenever
	   there is work to be done.
	*/

	jack_set_process_callback (client, process, 0);

	/* tell the JACK server to call `jack_shutdown()' if
	   it ever shuts down, either entirely, or if it
	   just decides to stop calling us.
	*/

	jack_on_shutdown (client, jack_shutdown, 0);

	/* display the current sample rate.
	 */

	printf ("engine sample rate: %" PRIu32 "\n",
		jack_get_sample_rate (client));

	/* create two ports */

	input_port = jack_port_register (client, "input",
					 JACK_DEFAULT_AUDIO_TYPE,
					 JackPortIsInput, 0);

	if (input_port == NULL) {
		fprintf(stderr, "no more JACK ports available\n");
		exit (1);
	}

	windowindex = 0;
	last_input = 0;
	last_diff = 0;
	current_ms = 0;
	current_time = 0;
	initialize_peak(&last_peak);
	initialize_peak(&current_peak);
	initialize_gap(&last_gap);
	initialize_gap(&current_gap);
	gap_validity = 0;
	current_state = 0;

	/* Tell the JACK server that we are ready to roll.  Our
	 * process() callback will start running now. */

	if (jack_activate (client)) {
		fprintf (stderr, "cannot activate client");
		exit (1);
	}

	ports = jack_get_ports (client, NULL, NULL,
				JackPortIsPhysical|JackPortIsOutput);
	if (ports == NULL) {
		fprintf(stderr, "no physical capture ports\n");
		exit (1);
	}

	if (jack_connect (client, ports[0], jack_port_name (input_port))) {
		fprintf (stderr, "cannot connect input ports\n");
	}

	free (ports);

	/* keep running until stopped by the user */

	sleep (-1);

	/* this is never reached but if the program
	   had some other way to exit besides being killed,
	   they would be important to call.
	*/
	ros::init(argc, argv, "microphone_node");
	ros::NodeHandle nh;
	ros::Publisher Micro_ac_pub = nh.advertise<std_msgs::Empty>("/MP_received",10)

	jack_client_close (client);
	exit (0);
}


