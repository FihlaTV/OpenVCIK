/*                                   LICENSE 
 *                                ----------------
 *   Copyright (c) 2015,  Raman Research Institute, Bangalore, India.
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
 */


/*                                                DESCRIPTOPN
 *
 * Header file for the main program which implements the Service Layer of the OpenVCIK framework.This class as of now It integrates 
 * all other classes such as the dataQueue, serialComm, etc., and provides APIs for different functions
 *
 *      Author: Junaid Ahmed Ansari
 *      email: ansariahmedjunaid@gmail.com 	
 *      github: junaidcs032
 *
 */

#ifndef OPENVCIKSERVICES_H
#define OPENVCIKSERVICES_H

namespace OpenVCIK{

using namespace std;
using namespace chrono;

/**
 * below are the structures to hold data related to specific sensors: 
 * sonarReading - ultrasonic proximity sensors (sonars)
 * compassReading - digital I2C based compass sensor
 * chargeReading - sensing the battery's charge  
 */
 
typedef struct sonarReading{

      int reading[8];         // in CM
      int timestamp[4];       // in hh : mm : ss : ms format from 0 - 3 respectively

};

typedef struct compassReading{

      int reading;            // in degrees  
      int timestamp[4];       // in hh : mm : ss : ms format from 0 - 3 respectively

};

typedef struct chargeReading{

      int reading;            // in percentage           
      int timestamp[4];       // in hh : mm : ss : ms format from 0 - 3 respectively

};


/* timestamp structure - currently not used in sensor data */

typedef struct timestamp{

  int hr;
  int min;
  int sec;
  int ms;

  // prints the timestamp - no end line in the print
  void print(){
      cerr<< hr <<":" << min << ":" << sec << ":" <<ms;
  }

  // returns timestamp as HH:MM:SS:MS 
  string getAsString(){
      string ts;
      ts = "";
      ts = ts + to_string(hr)  +":";
      ts = ts + to_string(min) +":";
      ts = ts + to_string(sec) +":";
      ts = ts + to_string(ms); 
      return ts;
  }
};

/**
 * structure with two byte mask characters for masking sensors and other functionalities such as PID control or sensor data sending, etc
 * Note: Change: byteMask was previously called sensorMask
 */

typedef struct byteMask{

      unsigned char mask1;
      unsigned char mask2; 
};

enum sensorType{ SONAR, COMPASS, CHARGE};


/* class which for now completely defines the Service Layer of the Framework */

class Services{
	
   	string cmdString;		  /* cmd string*/
   	mutex _cmdStringLock;	/* mutex for cmd string */
   	int _numOfBytesSent;

   	/* the strucuters for holding the parsed data from sensors */
    sonarReading   _sonarData;
    compassReading _compassData;
    chargeReading  _chargeData;

    byteMask _mask;

    /* queues for storing sensor readings in raw (String) format */
    dataQueue *sonarQueue;
    dataQueue *compassQueue;
    dataQueue *chargeQueue;
    dataQueue *speechQueue;

    SerialComm *serial;               /* object for serial communication */
    mutex _busLock;		                /* lock for serial comm. */	

    bool _commServiceContFlag;        /* this flag decides whether the commServiceThread should continue or not */
    mutex _commServiceContFlagMutex;  /* mutex which control mutual exclusion of _commServiceContinueFlag */    
    string _data;            		      /* this is used as a buffer for storing data from queues before parsing them */
    std::thread commService;   		    /* thread object for the comm. service thread containing DAQ and command sending function */
    
    mutex _speechRecogContFlagMutex;
    bool _speechRecogContFlag;
    std::thread speechRecognitionService;   /* thread object for speech recognition service thread */      
	
    // below six are used for speaking
    mutex speakFlagLock;
    mutex speakDoneFlagLock;
    string speakText;
    bool speakFlag; 
    bool speakDoneFlag;
    std:: thread tts;   /* thread variable which keeps the speaking thread  -- we are not using it in the new ARCHITECTURE */

	/* for timestamping */
    unsigned long long int _startTime;
    unsigned long long int _elapsedTime;
   
    private:

        void commServiceThread();     			        /* commServiceThread Function. For more info check the definition */
        void speechRecognitionServiceThread();   		/* Speech recogition service thread funciton. */ 
        void speakThreadDetachable(string text);  	/* On demand TTS thread function */
        //void speakThread();                       /* for speaking (TTS)  -- not in this architecture

        /* to set up pocketsphinx speech recongnition with dictionary, hmm, and language model files */      
       
        int setupSpeechRecognition(string hmm, string lm, string dict,int);        
        void startSpeechRecognitionThread();                                  /* starts the recognition thread */
      

    public:

      /* constructor */	
      Services(string port, 
      	   speed_t speed, 
      	   int sonarQueueSize, 
      	   int compassQueueSize, 
      	   int chargeQueueSize, 
      	   int speechQueueSize, 
      	   string hmm, 
      	   string lm, 
      	   string dict
      	   );

      /*destructor - it release all the allocated memory and resources */
      ~Services();
      

      /* All the get funciton below mentioned will load the structures with most recent values. But if values not present in the queue
       * then will not affect the structures, that is the struucture will retain their previous data.
       */

      const compassReading* getCompassData(int);
      const chargeReading* getChargeData(int);
      const sonarReading* getSonarData(int);
      string getSpeechCommand(int);

      /* All the below get size functions return the number of enteries in the corresponding queue at the time of calling */
      int getSonarQueueSize();
      int getCompassQueueSize();
      int getChargeQueueSize();
      int getSpeechQueueSize();

      /* clear corresponding queue */
      void clearSonarQueue();
      void clearCompassQueue();
      void clearChargeQueue();
      void clearSpeechQueue(); 
      
      void sendCommand(string);                                             /* send cmd via Serial Comm */
      
      timestamp computeTimeStamp(int print = 0);                            /* returns current timestamp and if passed 1 then prinst on stderr */	  	      
      timestamp getSystemTime();                                            /* returns the system time */
      void timestampSync(bool findSyncTime = true);                         /* syncs the timestamp */      

      void speakDetachable(string s);                                       /* speak a text using on demand TTS serivice */      

      void startServices();		/* starts all the services i.e. the Service Layer of the framework will be setup */
      
}; 

}

#endif

