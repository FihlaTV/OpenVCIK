/*                                   LICENSE 
*                                ----------------
*   Copyright (c) 2015,  Raman Research Institute, Bangalore, India.
*  
*    
*   This program is free software; you can redistribute it and/or modify
*   it under the terms of the GNU General Public License as published by
*   the Free Software Foundation.
*
*   This program is distributed in the hope that it will be useful,
*   but WITHOUT ANY WARRANTY; without even the implied warranty of
*   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
*   GNU General Public License for more details.

*   You should have received a copy of the GNU General Public License along
*   with this program; if not, write to the Free Software Foundation, Inc.,
*   51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA.

*   This work was supported by the Department of Science and Technology (DST), India. 
* 
*   RRI hereby disclaims all copyright interest in the project OpenVCIK.
*   
* -------------------------------------------------------------------------------------
*  Note: This copyright is only on this program and not on Pocketsphinx of which we
*        are using the APIs.
* ------------------------------------------------------------------------------------
*/


/*                                                DESCRIPTOPN
 *
 * openVCIKSpeechRecognition.cpp is a module which is used in the OpenVCIKServices.cpp i.e. the Service Layer for speech recogntion
 * using the Pocketsphinx Library. This module defines APIs so that it is not only used in the OpenVCIKServices.cpp, but that can be 
 * used in any other program by just including this file and calling such APIs which internally call the Poccketsphinx APIs. 
 *
 * Note: This program is a modified version of the continuous.c example available in the pocketsphinx's Github page.
 *
 *        Author: Junaid Ahmed Ansari
 *        emain: ansariahmedjunaid@gmail.com
 *        github: junaidcs032
*/


/*
COMPILE : g++ -o speechRecog openVCIKSpeechRecognition.cpp `pkg-config --cflags --libs pocketsphinx sphinxbase gtk+-2.0`

RUN : ./speechRecog -hmm <hmm_directory_path> -lm <.lm file> -dict <.dic file>

Note: Add a main before testing it. Main for testing to be included...
*/



#include <stdio.h>
#include <string.h>
#include <string>
#include <sphinxbase/err.h>
#include <sphinxbase/ad.h>
#include <sphinxbase/cont_ad.h>
#include <iostream>

#ifndef SPEECH_RECOGNITION
#define SPEECH_RECOGNITION

#include "pocketsphinx.h"

namespace OpenVCIK{

using namespace std;


static ps_decoder_t *ps;
static cmd_ln_t *config;
static FILE* rawfd;
int i=0;
ad_rec_t *ad;
cont_ad_t *cont;
int wordCount=0;


static const arg_t cont_args_def[] = {
    POCKETSPHINX_OPTIONS,
    /* Argument file. */
    { "-argfile",
      ARG_STRING,
      NULL,
      "Argument file giving extra arguments." },
    { "-adcdev",
      ARG_STRING,
      NULL,
      "Name of audio device to use for input." },
    { "-infile",
      ARG_STRING,
      NULL,
      "Audio file to transcribe." },
    { "-time",
      ARG_BOOLEAN,
      "no",
      "Print word times in file transcription." },
    CMDLN_EMPTY_OPTION
};


/* Sleep for specified msec */
static void
sleep_msec(int32 ms)
{
#if (defined(WIN32) && !defined(GNUWINCE)) || defined(_WIN32_WCE)
    Sleep(ms);
#else
    /* ------------------- Unix ------------------ */
    struct timeval tmo;

    tmo.tv_sec = 0;
    tmo.tv_usec = ms * 1000;

    select(0, NULL, NULL, NULL, &tmo);
#endif
}



// setups the speech recognition by looking for MIC, then Initiallizing VAD and then CALIBRATING the VAD
// returns -1 - MIC NOF FOUND
//         -2 - VAD NOT INITIALLIZED
//         -3 - FAILED RECORDING            
//         -4 - VAD COULD NOT BE CALIBRARTED      
int setupMic(){


    if ((ad = ad_open_dev(cmd_ln_str_r(config, "-adcdev"),
                          (int)cmd_ln_float32_r(config, "-samprate"))) == NULL)
        return -1;

    /* Initialize continuous listening module */
    if ((cont = cont_ad_init(ad, ad_read)) == NULL)
        return -2;
    if (ad_start_rec(ad) < 0)
        return -3;
    if (cont_ad_calib(cont) < 0)
        return -4;

    return 0;

}

/*
* Main utterance processing loop:
* for (;;) {
* wait for start of next utterance;
* decode utterance until silence of at least 1 sec observed;
* print utterance result;
* }
*/
static void
recognize_from_microphone(string* s)
{
    int iter=0;
    unsigned long int ccurrTime = 0, cprevTime = 0;
   // ad_rec_t *ad;
    int16 adbuf[4096];
    int32 k, ts, rem;
    char const *hyp;
    char const *uttid;
   // cont_ad_t *cont;
    char word[256];
   int sampleSize = 0;     // this variable is included oly for the sake of analysis of the time taken by individual words
    /*

            call setupSpeechRecog()
    
    */
	// if( setupMic() <0){
	// printf("MIC ISSUE");        
   	 
	// }

    //for (;;) {
        /* Indicate listening for next utterance */
//        printf("READY....\n");
//        fflush(stdout);
//        fflush(stderr);
       // cont_ad_set_thresh(cont,1,4);
        do{     
                cprevTime = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch()).count() ;           
                k = cont_ad_read(cont, adbuf, 2850);   //2850          
                if(k==0) sleep_msec(130);  //130
                   
        }while(k==0);
	sampleSize+=k;

        /* Wait data for next utterance */
      //  while ((k = cont_ad_read(cont, adbuf, 256)) == 0)
      //      sleep_msec(10);

        if (k < 0)
            E_FATAL("Failed to read audio\n");

        /*
* Non-zero amount of data received; start recognition of new utterance.
* NULL argument to uttproc_begin_utt => automatic generation of utterance-id.
*/
        if (ps_start_utt(ps, NULL) < 0)
            E_FATAL("Failed to start utterance\n");
        ps_process_raw(ps, adbuf, k, FALSE, FALSE);
//        printf("Listening...\n");
//        fflush(stdout);

        /* Note timestamp for this first block of data */
        ts = cont->read_ts;

        /* Decode utterance until end (marked by a "long" silence, >1sec) */
        for (;;) {
	    
            /* Read non-silence audio data, if any, from continuous listening module */
            if ((k = cont_ad_read(cont, adbuf, 2850)) < 0) //2500,>3
                E_FATAL("Failed to read audio\n");
            if (k == 0) {
                /*
* No speech data available; check current timestamp with most recent
* speech to see if more than 1 sec elapsed. If so, end of utterance.
*/
                if ((cont->read_ts - ts) > 500) // 3200 for 200ms ,  DEFAULT_SAMPLES_PER_SEC)
                    break;
            }
            else {
                /* New speech data received; note current timestamp */
                ts = cont->read_ts;
            }

            /*
* Decode whatever data was read above.
*/
            rem = ps_process_raw(ps, adbuf, k, FALSE, FALSE);

	    iter ++;

            sampleSize+=k;
	    if(sampleSize>10000)	    // speech sample based Full Hyp triggering..as discussed in paper
               break;
            /* If no work to be done, sleep a bit 
            if ((rem == 0) && (k == 0))
                sleep_msec(10);*/
        }

        /*
* Utterance ended; flush any accumulated, unprocessed A/D data and stop
* listening until current utterance completely decoded
*/
        ad_stop_rec(ad);
        while (ad_read(ad, adbuf, 4096) >= 0);
        cont_ad_reset(cont);

//        printf("Stopped listening, please wait...\n");
//        fflush(stdout);
        /* Finish decoding, obtain and print result */
        ps_end_utt(ps);
        hyp = ps_get_hyp(ps, NULL, &uttid);
        ccurrTime = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch()).count();        
	
	/* Below commented segmented is used for debugging.. it displays the recognized word and time taken to do so */	
	/*
	if(hyp!=NULL || hyp!=""){
	        printf("%d,%d,%s,%ld\n",wordCount, sampleSize, hyp, ccurrTime-cprevTime-31); 
		wordCount++;
		
	}
	*/
	
	//cout<< " In speechRecog.c.cpp : In recog_from_mic(): moving hyp \n";
	*s = move(hyp);
       
        // push it in the QUEUE

        /* Resume A/D recording for next utterance */
        if (ad_start_rec(ad) < 0)
            E_FATAL("Failed to start recording\n");
   //}


}

/* closes the speech sample reading session*/
void closeRecording(){

    cont_ad_close(cont);
    ad_close(ad);
    ps_free(ps);
    printf("freed ps");
    fflush(stdout);
}

static jmp_buf jbuf;
static void
sighandler(int signo)
{
    longjmp(jbuf, 1);
}

// //initiallizes pocketsphinx..
//returns -1 - CONFIG ERROR
//        -2 - INIT ERROR
int initSpeechRecog(string hmm, string lm, string dict){
     
   char* params[] = {" ", "-hmm", (char*)hmm.c_str(), "-lm" , (char*)lm.c_str(), "-dict" , (char*)dict.c_str(), "-pl_window","8","-kdmaxdepth","5","-maxwpf","5","-bestpath", "false", "-maxhmmpf", "3000", "-ds" ,"2"};
	
    char const *cfg;
    err_set_logfile("/dev/null");
    config = cmd_ln_parse_r(NULL, cont_args_def, 7, (char**)params, FALSE);
    if (config == NULL)
        return -1;

    ps = ps_init(config);
    if (ps == NULL)
        return -2;

    return 0;
}


} // end of namspace

#endif

