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
 * Implementation of the openVCIKServices.h. As stated in the openVCIKServices.h, this forms the main library for the app
 * application developer to build voice based control application on the OpenVCIK Framework.
 *
 *      Author: Junaid Ahmed Ansari
 *      email: ansariahmedjunaid@gmail.com
 *      github: junaidcs032
 *
 * to run any example : g++ <filename> -o <output> -pthread -std=c++0x -Wl,--no-as-needed `pkg-config --cflags --libs pocketsphinx sphinxbase gtk+-2.0` 
 * Make sure you have Main() or include it in another file and then run it if you want to compile then run this file with -c option
 */



#include <iostream>
#include <fstream>
#include <cstdlib>
#include <string>
#include <queue>
#include <string.h>
#include <stdio.h>
#include <string>
#include <exception>
#include <utility>                  
#include <mutex>
#include <thread>
#include <chrono>
#include <cmath>

#include "openVCIKDataQueue.cpp"
#include "openVCIKSerialComm.cpp"
#include "openVCIKSpeechRecognition.cpp"

#include "openVCIKServices.h"


using namespace std;

namespace OpenVCIK{


/* CONSTRUCTOR 
 * initiallizes the strucutures 
 * all time stamps are inits by -1, all reading are init by -1 except for sonar reading which is 0
 * throws 1 - if port not available
 * throws 2 - problem getting initial port parameters
 * throws 3 - cant setup the parameters of port
 * throws 4-6 - (sonar, compass,charge) queue bad allocation exception'
 * throws 7-13   - problem with speech recog setup - see the setupSpeechRecognition() function for details of errorcode
*/

Services::Services(string port, speed_t speed,int sonarQueueSize, int compassQueueSize, int chargeQueueSize, int speechQueueSize, string hmm, string lm, string dict): serial(NULL), _speechRecogContFlag(true),_commServiceContFlag(true), cmdString(""),_numOfBytesSent(-1){


      int errorNumber=0;
      speakFlag = false;
      speakDoneFlag = false;
      serial = new SerialComm(port);                // init the serial port 
      int speechSetupStatus = 0; 
 
      errorNumber = serial->openPort();                 

      // if something failed
      if(errorNumber == -1){
        throw (1);                  // could not open port
        
      }
      else if(errorNumber == -2)
        throw (2);                  // problem getting initial port parameters

      errorNumber = serial->setupPortParameters(speed, speed, 0, 2); 
      
      // if something failed
      if(errorNumber <0){
        throw (3);                  // COULD NOT SETUP PARAMETERS FOR PORT
      }

      // wait for some time for connection to be established - 3 second is not an exact figure can certainly be less
      std::this_thread::sleep_for (std::chrono::seconds(3));

      serial->flushPortIO();        // flush

      try{
            sonarQueue = new dataQueue(sonarQueueSize);
      }
      catch(int a){
            throw (4);  // bad alloc for sonar queue
      }

      try{      
        compassQueue = new dataQueue(compassQueueSize);     
      }
      catch(int a){  
        throw (5);      // bad alloc for compass    
      }
      try{   
            chargeQueue = new dataQueue(chargeQueueSize);
      }
      catch(int a){
        throw (6);      // bad alloc for charge Queue
      }
      

      speechSetupStatus = setupSpeechRecognition(hmm, lm, dict, speechQueueSize);

      if(speechSetupStatus<0)
        throw((speechSetupStatus*-1) + 6);


    // intiallize the data strucuture holding parsed data from the queues
    for(int i = 0; i<4 ; ++i){
        _sonarData.timestamp[i] = -1;
        _compassData.timestamp[i] = -1;
        _chargeData.timestamp[i] = -1;
      
    }
    for(int i = 0; i<8 ; ++i){
        _sonarData.reading[i] = 0;
    }
    
    _compassData.reading = -1;
    _chargeData.reading = -1;

    // initiallize the mask
    _mask.mask1 = 0;
    _mask.mask2 = 0;      
 
    _startTime = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch()).count();  
 

    //startServices();
}


/* Starts Comm and Speech services
 * Note: Make sure you try the constructor and catch the exceptions before calling this function else it might creat problems
 * Also later we can have individual function for starting the services independently... 
 */

void Services::startServices(){

    timestampSync();							//sync timestamp before starting anything
    commService = move(thread(&Services::commServiceThread, this));	// starts the commService thread
    startSpeechRecognitionThread(); 					// start the speech recognition  service thread    
    // tts = move(thread(&VCIW::speakThread, this));   			// start the TTS thread
}


/* timestampSync - it syncs the Host timestamp with the Mega Timstamp with very minor lags
 * Arguement : findSyncTime : True (default): then sends a set of sync time find character and calculates the send time
			      which is then used as the offset from the current time;

			    : False: does not calculate the send time; it justs inits the timestamp to 0

  Note: This function sleeps for 100 ms just to create a separation between this comm send and the Comm Service's comm activities
        i.e. the Comm Service will start after 100ms 
*/
void Services::timestampSync(bool findSyncTime){

	int timeForComm = 0;
	
	/*first calculate the avg character sendTime i.e. timeForComm */
	if(findSyncTime){
	
		int dataCount = 0;
		unsigned long long int startTime = 0;
		unsigned long long int endTime = 0;
		unsigned int sumOfTimeDiff = 0;
	 
		string syncData;
		while(dataCount < 1000){
			startTime = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch()).count();
			serial->writePort("%");
		
			do{
				if(serial->readPort(&syncData) > 0)	;
					//cerr<< "got it" <<endl;
			
			}while(syncData == "%");

			endTime = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch()).count();		
			sumOfTimeDiff += (endTime - startTime) ;		
			dataCount++;
		}
	
		/* calculating the average character send time */	
		timeForComm = round(float(float(sumOfTimeDiff)/1000)/2);

		cerr <<"time for Comm : "<< sumOfTimeDiff<<"   "<< timeForComm<<":";  		
	}

	/* now lets send start SCA loop character and sync the timestamps */	
	serial->writePort("@");
	_startTime = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch()).count() + timeForComm; 

	/* sleep for some time just to create a separation between this sending and the comm thread */
	std::this_thread::sleep_for (std::chrono::milliseconds(100)); 
}

/*
//initiallizes the timestamp
inline void Services::initTimestamp(){

	_startTime = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch()).count();
}
// pass a init time 
inline void Services::initTimestamp(unsigned long long int t){

	_startTime = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch()).count();
	_startTime += t;
}

*/


/* all the get<sensor>QueueSize functions below will return the queue size not the assigned size.
 *if the queue is not initiallized then they wil return -1;
 */
inline int Services::getSonarQueueSize(){

  if(sonarQueue!=NULL)
    return sonarQueue->getQueueSize();
  else
    return -1;
}

inline int Services::getCompassQueueSize(){
  
  if(compassQueue!=NULL) 
    return compassQueue->getQueueSize();
  else
    return -1;
}

inline int Services::getChargeQueueSize(){
  if(chargeQueue!=NULL)
    return chargeQueue->getQueueSize();
  else
    return -1;
}

inline int Services::getSpeechQueueSize(){

  if(speechQueue!=NULL)
    return speechQueue->getQueueSize();
  else
    return -1;
}

/*

//** This segment was used for having a dedicated thread for TTS 
//** which now we donot use as we go on-demand thread generation

// to speak...(toggles the flag for speaking and sets the speakTest)
void Services::speak(string s){
    
    {
        lock_guard<mutex> lock(speakFlagLock);
        speakFlag = true;
    }
    
    speakText = s;
}

// function which runs in a thread for speaking
void Services::speakThread(){
    bool isDone = false;
    bool isSpeakReq = false;
    string s;
    do{
        {
            lock_guard<mutex> lock(speakFlagLock);
            isSpeakReq = speakFlag;
            speakFlag = false;
        }
        
        if(isSpeakReq){
            // speaking code
            //cout<< "spoke : " << speakText;
	    s = "flite -t \"" + speakText;
	    s += "\"";	
	    system(s.c_str());
	    //cout.flush();
        }
        
        {
            lock_guard<mutex> lock(speakDoneFlagLock);
            isDone = speakDoneFlag;
        }
       // std::this_thread::sleep_for (std::chrono::seconds(5));
    }while(!isDone);
    
   
}
*/

/* function which runs in a detached thread for speaking */
void Services::speakThreadDetachable(string text){
    string s;
    s = "flite -t \"" + text;
    s += "\" &";  
    system(s.c_str());   
}

/* this will allow u to send a string to speak in a detached thread.. that is the thread is spawned when speak is requested
 and immediatly released */

void Services::speakDetachable(string text){

		speakThreadDetachable(text);
           // thread t(&Services::speakThreadDetachable,this, text);          // spawn a thread
            //t.detach();                                     // detach it ... Note that we dont have any ctrl over it any more  
}

//sets up speech recogntion:
//Note that the below error codes are appended above 6 when they are used in construcutor
//returns  -1 - CONFIG ERROR
//         -2 - INIT ERROR
//         -3 - MIC NOF FOUND
//         -4 - VAD NOT INITIALLIZED
//         -5 - FAILED RECORDING            
//         -6 - VAD COULD NOT BE CALIBRARTED 
//         -7 - BAD ALLOC FOR QUEUE ALLOCATION RELATED TO SPEECH DATA
int Services::setupSpeechRecognition(string hmm, string lm, string dict, int speechQueueSize){

  int initStatus = 0, initMicStatus = 0;
  initStatus = initSpeechRecog(hmm, lm, dict);         // intiallizes the pocketsphinx by providing hmm, lm, dict
  
  if(initStatus<0)
    return initStatus;

  initMicStatus = setupMic();

  if(initMicStatus<0)
    return (initMicStatus-2);

  try{
        speechQueue = new dataQueue (speechQueueSize);
  }
  catch(int a){
        return -7;  //bad alloc for speechqueue
  }


  return 0; // successful
}

/*Starts the speech recognition service thread */

void Services::startSpeechRecognitionThread(){   
  
  speechRecognitionService = move(thread(&Services::speechRecognitionServiceThread, this));
}

/*
            thread t(speakThreadDetachable, textRecognized);          // spawn a thread
            t.detach();                                               // detach it ... Note that we dont have any ctrl over it any more  

*/

 
/*the function runs in an indpendent thread and does speech recogntion - this is termed as the Speech Recogntion Service in the Architecture*/            
void Services::speechRecognitionServiceThread(){

    bool flag = true;
    string textRecognized="";
    while(flag){
//        cerr << printTimestampOnCerr()<<"VC::ST(): ^R -> ";    // start recog
        recognize_from_microphone(&textRecognized);
        //cerr << printTimestampOnCerr()<< "In Services::speechRecThr: end recog"<<endl;
//        cerr << printTimestampOnCerr()<< " ~R"<<endl;
        if(textRecognized.data()!=""){
//            cerr << printTimestampOnCerr()<< "VC::ST(): ^P ->";
            speechQueue->put(textRecognized);
//            cerr << printTimestampOnCerr()<< " ~P"<<endl;

        }
        {
           lock_guard<mutex> guard(_speechRecogContFlagMutex) ;
           flag = _speechRecogContFlag;
   		
        }

      };
	cerr<< getSystemTime().getAsString()<<"INFO MSG: OUT OF SPEECH THREAD";
}


// destructor
Services::~Services(){



//        cout << "in destructor now";

        {

            lock_guard<mutex> guard(_commServiceContFlagMutex);
            _commServiceContFlag = false;
        }
//        cout<< "lock released";
        if(commService.joinable()){
          commService.join();
        }
         cout<< "speech lock releasing";
        {

            lock_guard<mutex> guard(_speechRecogContFlagMutex);
            _speechRecogContFlag = false;
             cout<< "speech lock released";
        }
//          cout<< "speech recog lock released";
        if(speechRecognitionService.joinable()){
          speechRecognitionService.join();
        }

        closeRecording();     // global functions related to pocketsphinx
  
        /* for TTS thread  
        {
            lock_guard<mutex> lock(speakDoneFlagLock);
            speakDoneFlag = true;
        }
        if(tts.joinable())
          tts.join();*/

        cerr<< getSystemTime().getAsString() << "out of the threads deleting stuffs";
        delete sonarQueue;
        delete compassQueue;
        delete chargeQueue;
        delete speechQueue;

}


/* a function which would run in a SEPERATE THREAD readding data from the FIRMWARE */
void Services::commServiceThread(){

    string byte;
    string data(100,'\0');  
    bool flag;
    byte = "";
    int i=0;
    do{
          

    
          // sending serial data.... As it takes very little time lets give it priority
          {
            lock_guard<mutex> guard(_cmdStringLock);
	    	
            if(cmdString!="\0" || cmdString != ""){//cerr <<"------"<<cmdString<<endl;
              _numOfBytesSent = serial->writePort(cmdString.data());           // we have to find a way how to expose this _numOfButes.. variable
/*              if(_numOfBytesSent == cmdString.length())
//               cerr<< printTimestampOnCerr()<<"VC : DT(): ^S" <<endl;  
              else
               cerr << printTimestampOnCerr()<<"VC:DT(): ! ^S "<<endl;  */
            }
  	        cmdString =""; 
	   
          }
       // cout<<"after sending"<<endl;
         {           
          lock_guard<mutex> guard(_busLock);

          // from here is the DAQ part   
          data[0] = '\0';
          
          if(serial->readPort(&byte, 1)> 0){
            //cout<< "Befor IF";
            //cout<<byte[0];
            if(byte[0] == '$'){
//                cerr << printTimestampOnCerr()<<"VC:DT(): ^G \'$\' -> "<<endl;  
                while(1){ 
                  if(serial->readPort(&byte, 1) > 0 ){ //cout<<byte[0];  // chech if data is there?
                   if(byte[0] != '.')
                      data[i++] = byte[0];
                    else{
                      data[i] = '\0';   // put a null terminator
                      break;
                    }
                  }
                  else{
                    data="";   // purge the previously put data
                    break;
                  }
                } 
                
//                cerr << printTimestampOnCerr()<<"VC:DT(): ~G \'.\'"<<endl;                
            }
          }

    
        //cout<<"\n data :" << data<<endl;//<< endl ;

        // we can have functions sepcified for creating QUEUES and then also associated parsing chracter with it...
        // once the DAQ looks for header it will automatically put data in to the specified queuue....
       
        switch(data[0]){
            
            case 'S': 
                      //check if queue is Initialized
                       if(sonarQueue != NULL){//cout<<"sonar\n";

//                          cerr << printTimestampOnCerr()<< "VC:DT(): ^P S_U ->";       
                          sonarQueue->put(data);
//                          cerr << "~P "<<endl;       
                       }
                      break;
            case 'C':
                      //check if queue is Initialized
                      if(compassQueue != NULL){//cout<<"comp\n";
//                          cerr << printTimestampOnCerr()<< "VC:DT():^P S_C -> ";       
                          compassQueue->put(data);
//                          cerr << "~P "<<endl;
                      }//cout<<"after C"<<endl;
                      break;
            case 'V':
                      //check if queue is Initialized
                      if(chargeQueue != NULL){
//                         cerr << printTimestampOnCerr()<< "VC:DT():^P S_V -> ";       
                         chargeQueue->put(data);
//                         cerr << "~P " << endl;
                      }
                      break;
        }

        } // _busLOck -- lock released here  
        
        i=0;     
            
 //cout<<"after daq";cout.flush();

      // scope it
      { 
          lock_guard<mutex> guard(_commServiceContFlagMutex);
          flag = _commServiceContFlag;  // this will change in happen in the destrucutor of the Services class where it will be joined also

      } 
     
      i=0;


    //  std::this_thread::sleep_for (std::chrono::milliseconds(25));   // sleep for 25 ms as the datarate is 30 ms

    }while(flag);
    
    cerr<< getSystemTime().getAsString()<<"OUT of DAQThread()";
}



/* currently it is return by COPY.... we will change it in a better way */
string Services::getSpeechCommand(int current = 0){

      _data = speechQueue->get(current);    // remember get() exits() if out of bound occurs

      return _data; 
}

// function to get the compass data from the queue and parse it and return in the form of compassReading structure
const compassReading* Services::getCompassData(int current = 0){

//      cerr << printTimestampOnCerr()<< "VC:GC(): ^G "<<endl;       
      _data = compassQueue->get(current);

      if(_data != "\0"){
         _data.erase(_data.begin());    
          char * pch;
          int numTokens=0;
          
          pch = strtok ((char*)(_data.c_str()),",:");

          while (pch != NULL)
          {
            
            if( numTokens < 4){
                // parse the time stamp ------------FOR this testing We will not parse the TIMESTAMP

                _compassData.timestamp[numTokens] = atoi(pch);
                
                numTokens ++;               
            }   
            // parse the data
            else
                _compassData.reading = atoi(pch);

            pch = strtok (NULL, ",:");

          }
//          cerr << printTimestampOnCerr()<< "VC:GC():~G > "<<endl;       

          return &_compassData;
      }
      else{
//	  cerr << printTimestampOnCerr()<< "VC:GC():!"<<endl; 
          return NULL;
      }
}

// function to get the charge data from the queue and parse it and return in the form of chargeReading structure
const chargeReading* Services::getChargeData(int current = 0){

//      cerr << printTimestampOnCerr()<< "VC:GV():^G"<<endl;  
      _data = chargeQueue->get(current);

      if(_data != "\0"){
          _data.erase(_data.begin());
          char * pch;
          int numTokens=0;
          
          pch = strtok ((char*)(_data.c_str()),",:");

          while (pch != NULL)
          {
            // parse the time stamp
            if( numTokens < 4){

                
                _chargeData.timestamp[numTokens] = atoi(pch);
                
                
            } 
            // parse the data       
            else
                _chargeData.reading = atoi(pch);

            numTokens ++;
            pch = strtok (NULL, ",:");

          }
//          cerr << printTimestampOnCerr()<< "VC:GV():~G > "<<endl; 
          return (&_chargeData);
      }
      else{
//        cerr << printTimestampOnCerr()<< "VC:GV():!"<<endl; 
        return NULL;
     }
}

// function to get the sonar data from the queue and parse it and return in the form of sonarReading structure
// returns SonarReading structure or NULL if data not avaialable i.e. queue empty
const sonarReading* Services::getSonarData(int current = 0){

    //  cout<<"in get sonar";
//      cerr << printTimestampOnCerr()<< "VC:GS():^G "<<endl;  
      _data = sonarQueue->get(current);
       //std::this_thread::sleep_for (std::chrono::seconds(3));
      if(_data != "\0"){    // if data available
          _data.erase(_data.begin());	
          char * pch;
          int numTokens=0;

          pch = strtok ((char*)(_data.c_str()),",:");

          while (pch != NULL)
          {
            // parse the time stamp
            if( numTokens < 4){
                
                _sonarData.timestamp[numTokens] = atoi(pch);
                 
             }//   numTokens ++;                                       // this was not there earlier. put it on 4-9-14

            // parse the data
            else
                _sonarData.reading[numTokens-4] = atoi(pch);

            numTokens ++;  
      
            pch = strtok (NULL, ",:");

          }
//          cerr << printTimestampOnCerr()<< "VC:GS():~G > "<<endl;            
          return &(_sonarData);
      }
      else{
//	   cerr << printTimestampOnCerr()<< "VC:GS():!"<<endl;
           return NULL;  
      }
      
}

inline void Services::clearSonarQueue(){

    if(sonarQueue!= NULL){
        sonarQueue->clear();
    }
}

inline void Services::clearCompassQueue(){

    if(compassQueue!= NULL){
        compassQueue->clear();
    }
}

inline void Services::clearChargeQueue() {
    if(chargeQueue!= NULL){
        chargeQueue->clear();
    }
}

inline void Services::clearSpeechQueue() {
    if(speechQueue!= NULL){
        speechQueue->clear();
    }
}

// returns -1 if Serial Failed or Serial not initiallized. Else it returns the number of bytes WRITTEN
void Services::sendCommand(string s){
  int bytesWritten = -1;
  if(serial != NULL){


			{
				lock_guard<mutex> guard(_cmdStringLock);                        
//				cout<<s<<endl;cout.flush();
				cmdString = s;
			}
    
  }

//  we have to think of a mechanis to returspeechCommandn the no. of bytes sent which serves as a check for COMM sometimes... good practice
}



// Computes timestamp and writes the stamp to the provided struc
// returns pointer to integer containing 4 elements 
timestamp Services::computeTimeStamp(int print){
      timestamp ts;
      
      auto currentTime = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch()).count();  

      int rh, rm;                                           // variables to store remaineders for Time Stamp computing
       
      if(_startTime > currentTime)
	      _elapsedTime = _startTime - currentTime;// - _startTime;              // gather cuurent in time in millis and offeseted by the init time
      
      else
		_elapsedTime = currentTime - _startTime ;		

      ts.hr  = ((_elapsedTime)/(1000*60*60))%24;     // get hours
      rh     = (_elapsedTime)%(1000*60*60);               // get remainder of hours
      ts.min = ((_elapsedTime)/(1000*60)%60);        // get mins
      rm     = (_elapsedTime)%(1000*60);                  // get remainder of Mins      
      ts.sec = ((_elapsedTime)/(1000))%60;           // get seconds
      ts.ms  = (_elapsedTime)%(1000);    
      
      if(print){
	  //cerr <<"ts - "<<ts.hr<<":"<<ts.min<<":"<<ts.sec<<":"<<ts.ms<<endl;		
          cerr<<_elapsedTime<<" ";
      }
      return ts;

}

/* retunrs the current system time */
inline timestamp Services::getSystemTime(){
      timestamp ts;
      auto currentTime = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch()).count();  
      
      int rh, rm;                                           // variables to store remaineders for Time Stamp computing      
      
      ts.hr  = (int(((currentTime)/(1000*60*60))%24));      // get hours
      rh     = (currentTime)%(1000*60*60);                  // get remainder of hours
      ts.min =(int((currentTime)/(1000*60)%60));            // get mins
      rm     = (currentTime)%(1000*60);                     // get remainder of Mins      
      ts.sec = (int(((currentTime)/(1000))%60));            // get seconds
      ts.ms  = (int((currentTime)%(1000)));    

	return ts;
}

}// end of namespace



