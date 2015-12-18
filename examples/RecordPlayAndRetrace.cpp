#include <iostream>
#include <string>
#include <vector>
#include <chrono>

#include "openVCIKServices.hpp" 


using namespace OpenVCIK;			/* openVCIK namespace where all the APIs are grouped*/
using namespace std;
using namespace chrono;

vector<string> recordedCmd;			/* contains the recorded commands*/
vector<unsigned int> cmdExecutionTime;		/* contains the time of execution of the commands played*/
vector<int> cmdAngles;				/* conatins the opposite angles of the commands to be used in RETRACE */	

int currentAngle = 0;				/* store the current angle read by the compass */

bool isWheelchairMoving = false;

/* variables for storing sensor and speech data */
const sonarReading *sonar; 
const compassReading *compass;
string speechCommand;

/* just naming the error codes - easy for debugging */
string errorCodeStrings[12]={"SERIAL PORT NOT AVAILABLE\0",
			      "PROBLEM ACQUIRING PORT PARAMETERS\0",
			      "PROBLEM SETTING UP PORT PARAMETERS\0",
			      "SONAR BAD ALLOCATION\0",
			      "COMPASS QUEUEU BAD ALLOCATION\0",
			      "CHARGE QUEUE BAD ALLOCATION\0",
			      "SPHINX CONFIG ERROR\0",
			      "SPHINX INIT ERROR\0",
			      "MIC NOT FOUND\0",
           		      "VAD NOT INITIALLIZED\0",
			      "FAILED RECORDING\0",
			      "VAD COULD NOT BE CALIBERATED\0" };



/* Generates the wheelchair motion commands based on some basic commands depemneding upon currentAngle 
 * Return : Returns the motion command that can be sent to the HAL via the sendCommand() API
 * Note: We can club such Motion commands and obtain complex motion paths for the wheelchair
 */
 
string generateCommand(string speechCmd, int currAngle = 0){    
   
    string s;
    s=""; 

    // send f<theta>. at some angle 
    if (speechCmd == "FORWARD"){
          s =  "f" + to_string(currAngle);
          s = s + '.';
	  isWheelchairMoving = true;
    }
    
    // send r. for reverse
    else if(speechCmd == "DOWN"){

          s = s + "r.";
       	  isWheelchairMoving = true;
    }	
    
    //turn 90 at the spot in the left direction
    else if(speechCmd == "LEFTO"){
    
          currAngle = ((currAngle-90+360)%360);
          s = "t" + to_string(currAngle);
          s = s + '.';
	  isWheelchairMoving = true;
    }
    
    //turn 90 degrees at the spot in the right direction
    else if(speechCmd == "RIGHT"){
          currAngle = ((currAngle + 90)%360);
          s = "t" + to_string(currAngle);
          s = s + '.';
	  isWheelchairMoving = true;
    }
    
    // increase the speed
    else if(speechCmd == "SPEED UP"){

          s = "su.";
	  isWheelchairMoving = true;
    }    

    // speed down
    else if( speechCmd == "SPEED DOWN"){

          s = "sd.";
	  isWheelchairMoving = true;
    }            

    // stop the wheelchair i.e. send b. command
    else if (speechCmd == "STOP"){
          s = "b.";
	  isWheelchairMoving = false;
    }

    // NOP		
    else
    	s = "\0";

    /*debug line */
    cerr<<speechCmd<<" - " <<s<<" "<<endl;	
	
    return(s);    
}


/* generates an exact opposite angle for the passed angle */
int findOppositeAngle(int ang){

	int newAng = ang + 180;

	if(newAng == 360)
		newAng = 0;
	else if(newAng >360)
		newAng = newAng - 360;

	return newAng;
}


/* main : here we implement all the wheelchair control mechanism
 *        
 *       - Simple command and move 
 *       - "Record-and-Play" using "Next"
 *       - Retrace the sequence using "Retrace" command
 */

int main(int argc, char const *argv[])
{
	unsigned int timeOfExec = 0;
	unsigned long long int startTime = 0;
	unsigned long long int endTime = 0;
		
	int cmdPos = 0;
	int execTimePos = 0;
	string speechCmd;		
	string cmd;
	int cmdType = 0;
	/*
  		0 - normal
  		1 - recording
  		2 - playing
	*/

  	int ang;

	/* instantiate the  openVCIK ; Dont forget to put it in try {} */	
        try{
        	Services service("/dev/ttyACM0", B9600,  100,100,100,10,"/home/linaro/pocketsphinx/model/hmm/en_US/hub4wsj_sc_8k/","/home/linaro/Desktop/currentWorking_CubieboardPrograms_19_8_14/OpenVCIK_complete_working_program_21_11_14/newModel9795/9795.lm","/home/linaro/Desktop/currentWorking_CubieboardPrograms_19_8_14/OpenVCIK_complete_working_program_21_11_14/newModel9795/9795.dic");	
     
     		/*start services*/
     		service.startServices();
	
		/* go in an infinite command reception loop ; Breaks when uttered "DISABLE" */
		do{
			
			/* lets get the sensor data from there respective queues using sensor data get APIs */
			sonar = service.getSonarData();
			compass = service.getCompassData();

			/* lets check for any obstacle to stop the wheelchair incase the wheelchair is moving */
			if(isWheelchairMoving){
			
				// check for null very important to avoid segfaults....
				if(sonar!=NULL){
					
					/* in our wheelchair case we have only 2 sonars connected in the front which are 
					 * connected to the SONAR PORTS 3 and 4. 
					 * Check if any of the two sonars reads 50 cm for obstacles; if yes stop the chair
					 */
					 
					 if((sonar->reading[3] > 0 && sonar->reading[3] <= 50)||(sonar->reading[4] > 0 && sonar->reading[4] <= 50)){
					 
					 	string cmd = generateCommand("STOP");
					 	service.sendCommand(cmd);
					 	isWheelchairMoving = false;
					 }	
				}
			}
			
			/* get compass data to send with motion commands */			
			if(compass != NULL)
				currentAngle = compass->reading;
 
			/* debug lines */
			currentAngle = 0;
			
			/* get the current speech command from the Speech Recogntion Service */
			speechCmd = service.getSpeechCommand();
			
			/*check if speech command = DISABLE ; if yes get out of the PROGRAM*/
			if(speechCmd == "DISABLE")
				break;
			
			/* if some speech cmd available */					
			if(speechCmd != "\0"){
						
				if(speechCmd == "RECORD"){
					cmdType = 1;
					cerr << " Going in RECORD mode "<<endl;
				}

				else if(speechCmd == "HALT"){	
				
					// if in RECORD mode
					if(cmdType == 1){
						//cerr<<"clearing"<<endl;
						cmdExecutionTime.resize(recordedCmd.size());
						cmdAngles.resize(recordedCmd.size());
					}
			
					cmdType = 0;
					if(!recordedCmd.empty()){				
			
						/*
						 * compute the time taken;
						 * and save it
						 */
						 if(!cmdAngles.empty()){
							 
							 timeOfExec = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch()).count() - startTime;
							 cmdExecutionTime[execTimePos] = timeOfExec;
							 //cout<<"te"<<execTimePos<<" "<<timeOfExec<<endl;
						 }
						 execTimePos = 0;
						 cmdPos = 0;
						 // reinit the start time
						 startTime = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch()).count();
					}
					cerr << " End of mode "<<endl;
				}

				// go in play mode when alrcmdAngleseady record and in general command mode.. and play the first cmd
				else if(speechCmd == "BEGIN" && cmdType == 0 && !recordedCmd.empty()){

					cmdType = 2; 								
					
					/*execute the first command */					
					service.sendCommand ( generateCommand(recordedCmd[cmdPos], currentAngle) );				
					cmdAngles[cmdPos] = findOppositeAngle(currentAngle);			
					cmdPos = (cmdPos+1) % recordedCmd.size();			
					/*
					 * record the start time 	
					 */
					startTime = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch()).count();

				}

				/* remove the recorded commands only when in general command mode */
				else if(speechCmd == "REMOVE" && cmdType == 0 && !recordedCmd.empty()){
			
					recordedCmd.clear();

					cmdExecutionTime.clear();
					cmdAngles.clear();	
				}	

				// execute from the recorded cmds
				else if(speechCmd == "NEXT" && cmdType == 2){

						/*
						 * compute the time taken;
						 * and save it in the -1 of the current cmdPos location
						 */
						
						timeOfExec = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch()).count() - startTime;						
						cmdExecutionTime[execTimePos] = timeOfExec;
				
						/* reinit the time */	
						startTime = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch()).count();
				
						cmdAngles[cmdPos] = findOppositeAngle(currentAngle);
				
						/* execute the Command */

						service.sendCommand ( generateCommand(recordedCmd[cmdPos], currentAngle) );
				
						cmdPos = (cmdPos+1) % recordedCmd.size();
						execTimePos = (execTimePos+1) % recordedCmd.size();
				}
				else{

					if(cmdType == 1){
						cerr <<" saving speechCmd " << speechCmd <<endl;
						recordedCmd.push_back(speechCmd);
					}
					else{
						if(speechCmd == "ENABLE"){
							if(!cmdExecutionTime.empty() && !cmdAngles.empty() && !recordedCmd.empty()){
								//int recordCmdSize = recordedCmd.size() - 1;
								for(int i=recordedCmd.size()-1; i>=0 ; --i){
									/*debug lines */
									cerr<<" Retracing "<<recordedCmd[i]<< " "<< cmdAngles[i]<<" for "<<cmdExecutionTime[i]<<endl; 
									
									/* execute the commands in an Opposite fashion with the stored opposite angles for retrace */
									service.sendCommand( generateCommand(recordedCmd[i], cmdAngles[i]) );	
									
									/*let the command execute for the time it was played in record-and-play sequence */								
									std::this_thread::sleep_for (std::chrono::milliseconds(cmdExecutionTime[i]));
								}
							}
							
						}
						
						/*if a generat motion command then execute */
						service.sendCommand ( generateCommand(speechCmd, currentAngle) );
				
					}
				}
			
			}

		}while(speechCmd != "DISABLE");
		
		
	}// end of try {}
	
	catch(int error){
	
		/* catch the exception and print the error type and the error string*/
		cerr <<" \n Whoa! Some Error : "<<error<<" ("<< errorCodeStrings[error-1] <<")"<<endl;
	}
	
	return 0;
}



/* parse the command into individual tokens and stores in commandVector - vector*/
/*
void parseCommand(string cmd){


   
    char * pch;
    int numTokens=0;
   // cerr<<"In Global: In parseCommand : start of parse -> ";
     if(cmd != ""){
        if(!commandVector.empty())   
            commandVector.clear();
        pch = strtok ((char*)(cmd.c_str())," ");
        if(pch!=NULL)
          commandVector.push_back(pch);
        
        while (pch != NULL)
        {
        
            pch = strtok (NULL, " ");
            if(pch != NULL)
               commandVector.push_back(pch);
            //cout << pch << endl;
            //cout << " hello"; cout.flush();
        }
    }
   // cerr<<"end of parse"<<endl;
   
}

*/
