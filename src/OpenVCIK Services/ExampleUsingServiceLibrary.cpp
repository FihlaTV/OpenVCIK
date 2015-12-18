#include "openVCIKServices.hpp" 


using namespace OpenVCIK;








//___________________________________________________END OF Services CLASS______________________________________________//


// GLOBAL VARIABLES 
    vector<string> plan, temp;
    vector<string> reversed;
    string command;
    bool planAvailable = false;
    bool isExecuting = false;
    int mode = 0;  // 0- normal, 1-execture, 2- record
    int planCmdPointer = 1;
    bool isWheelchairMoving = false;	

    // for storing the data from sensors
     const sonarReading *s; 
     const compassReading *c;
     const chargeReading *ch;

     string speechCommand;



vector<string> commandVector;       // stores individual commands
vector<string> planVector;          // stores the plan. i.e. commands in the plan
int planVectorPointer = 1;

// parse the command into individual tokens and stores in commandVector - vector
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



// executes the basic commands depemneding upon currentAngle
string executeCommand(string cmd, int currAngle){    
    string s;
    s=""; 

    cerr << "cmd , angle : " << cmd.data()<<" "  << currAngle;
    
    if (cmd == "FORWARD"){
          s =  "f" + to_string(currAngle);
          s = s + '.';
	  isWheelchairMoving = true;
    }
    else if(cmd == "DOWN"){
          //currAngle = ((currAngle-90+360)%360);
          //s = "f" + to_string(currAngle);
          s = s + "r.";
	  isWheelchairMoving = true;

    }	
    else if(cmd == "LEFTO"){
          currAngle = ((currAngle-90+360)%360);
          s = "t" + to_string(currAngle);
          s = s + '.';
	  cerr << "left"<<s;
	  isWheelchairMoving = true;

    }
    else if(cmd == "RIGHT"){
          currAngle = ((currAngle + 90)%360);
          s = "t" + to_string(currAngle);
          s = s + '.';
	  isWheelchairMoving = true;
    }
    else if(cmd == "SPEED"){
//	  cout << "speedup"<<endl;cout.flush();
          s = "su.";
	  isWheelchairMoving = true;
    }    
     else if( cmd == "SPEED DOWN"){
//	  cout << "speeddown"<<endl;cout.flush();
          s = "sd.";
	  isWheelchairMoving = true;
    }
    else if( cmd == "LEFTO"){
          currAngle = ((currAngle-90+360)%360);
          s = "t" + to_string(currAngle);
          s = s + '.';
	  isWheelchairMoving = true;
    }
    else if(cmd == "RIGHT RIGHT"){
          currAngle = ((currAngle + 90)%360);
          s = "t" + to_string(currAngle);
          s = s + '.';
	  isWheelchairMoving = true;
    }

    else if (cmd == "STOP"){
          s = "b.";
	  isWheelchairMoving = false;
    }
	
    else
    	s = "\0";

//    cout <<"EXECUTING : "<<s;cout.flush();	
    return(s);    
}



int main(){

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

   string spkTxt;
   bool isDisabled = false;
   int compReading = 0;
   bool stopListenning = false;
   string genCmd;
   genCmd = "";
   //freopen ("/dev/null","w",stdout);
    try{
      Services v("/dev/ttyACM0", B9600,  100,100,100,10,"/home/linaro/pocketsphinx/model/hmm/en_US/hub4wsj_sc_8k/","/home/linaro/Desktop/currentWorking_CubieboardPrograms_19_8_14/OpenVCIK_complete_working_program_21_11_14/newModel9795/9795.lm","/home/linaro/Desktop/currentWorking_CubieboardPrograms_19_8_14/OpenVCIK_complete_working_program_21_11_14/newModel9795/9795.dic");	

      cerr << v.getSystemTime().getAsString()<< "MP:M():C()"<<endl;  

      v.startServices();

int syncFlag = 0;
do{
       

/*  -- below codes are being written to test the KIT */
         
   
	/*c = v.getCompassData();
	if(c!=NULL )
            cerr <<c->timestamp[0]<<":"<<c->timestamp[1]<<":"<<c->timestamp[2]<<":"<<c->timestamp[3] <<" C "<<" "<<c->reading<<endl;
	
	ch = v.getChargeData();
        if(ch!=NULL)
	    cerr <<ch->timestamp[0]<<":"<<ch->timestamp[1]<<":"<<ch->timestamp[2]<<":"<<ch->timestamp[3]<<"CH "<< ch->reading<<endl;
	*/
	s = v.getSonarData(0);
        if(s!=NULL){
		

	     unsigned long long int el = (s->timestamp[3] + s->timestamp[2]*1000+ s->timestamp[1]*60000+s->timestamp[0]*3600000);	
	    if(syncFlag==0){
		//v.initTimestamp(el);
		syncFlag++;
	    }
             v.computeTimeStamp(1);
	     cerr << s->timestamp[0]<< s->timestamp[1]<<":"<<s->timestamp[2]<<":"<<s->timestamp[3]<<endl;
                       //<< s->reading[1] << ","
                    //   << s->reading[2] << ","
                     //  << s->reading[3] << ","
                     //  << s->reading[4] << ","
                     //  << s->reading[5] << ","
                     //  << s->reading[6] << ","
                     //  << s->reading[7] << ","
		       //<< endl;
	    
	}
		
/* -- TILL HERE --------*/


    speechCommand = v.getSpeechCommand(0);      
    if(speechCommand!= "\0")
	    cerr << speechCommand <<endl;
    if(speechCommand == "ENABLE ENABLE") {

        do{  

      	s = v.getSonarData(0);
        if(s!=NULL){
	       	if(((s->reading[4]>0 &&  s->reading[4]< 50)||(s->reading[0]>0 &&  s->reading[0]< 50)) && isWheelchairMoving == true){// || s->reading[3] <20){
			         v.sendCommand("b.");
			         isWheelchairMoving = false;      
				 cout<<"S:"<<s->reading[0]<<","<<s->reading[1]<<","<<s->reading[2]<<","<<s->reading[3]<<","<<s->reading[4]<<","<<s->reading[5]<<","<<s->reading[6]<<","<<s->reading[7]<<endl;         
                }
		                 // cerr<<"S:"<<s->reading[0]<<","<<s->reading[1]<<","<<s->reading[2]<<","<<s->reading[3]<<","<<s->reading[4]<<","<<s->reading[5]<<","<<s->reading[6]<<","<<s->reading[7]<<endl;//}//","<<s->reading[0]<<","<<}
	      }
	     c = v.getCompassData(0);

       if(c!=NULL){

         compReading = c->reading;
	//cerr<< compReading<<endl;
       }
 	    
	    ch = v.getChargeData(0);
      if(ch!= NULL){
        int x = ch->reading;
      }
	    speechCommand = v.getSpeechCommand(0);
	    if(speechCommand!=""){

	           if(speechCommand == "DISABLE"){
    		               isDisabled = true;
                       break;
    	       }
/*
      			if(speechCommand != "")
      	         	v.sendCommand(executeCommand(speechCommand, compReading));	                	

*/ 		

             parseCommand(speechCommand);
//	           cout<<"Speech:"<<speechCommand<<endl;
                 
                    
                    

                    if(commandVector.size() > 1){
                        //cout<<"if";
                        if(commandVector[0] == "RECORD" ){
                            for(int i = 1; i<commandVector.size(); ++i){       
                                planVector.push_back(commandVector[i]);
                            }
                        }
                        // v.speak("else")  ;
                        else if(commandVector[0] == "EXECUTE" && commandVector[1] == "PLAN"){

                             // v.speak("executing command" + planVector[0]);
                             if(!planVector.empty()){

                                v.sendCommand(executeCommand(planVector[0], compReading));     
                                v.speakDetachable("EXECUTING "+planVector[0]);
                             }
                             else
                                v.speakDetachable("EITHER WRONG COMMAND OR PLAN EXECUTED");
                        } 
            		        else if(commandVector[0] == "SPEED"){
                    			 	cout<<">2"<<speechCommand<<endl;
                    				v.sendCommand(executeCommand(speechCommand,0));
            		      	}		

                    }
                    else{

                        if(commandVector[0] == "NEXT" && !planVector.empty()){cout<<"next";cout.flush();
                            if(!planVector.empty() && planVectorPointer <planVector.size()){                                   
                                v.sendCommand(executeCommand(planVector[planVectorPointer], compReading));
                                v.speakDetachable("EXECUTING "+planVector[planVectorPointer++]);
			                       }
                            else{
                                v.speakDetachable("EITHER COMMANDS EXHAUSTED OR SOME ERROR");
                                planVectorPointer = 1;
                            }
                                
                        }
                        
                        else if(commandVector[0] == "CLEAR" && !planVector.empty()){
                            planVector.clear();
                            v.speakDetachable("PLAN CLEARED");                            
                        }
                        
                        else{

          			             cout<<commandVector[0];
		                         v.speakDetachable(("EXECUTING " + speechCommand));
			                       v.sendCommand(executeCommand(commandVector[0], compReading));                        
//			     }
    	                  } 		
                            
                        
                  }
                    
            } //cout<<"end of inner do"<<endl;cout.flush();
        }while(1);      
	}
	if(isDisabled)
	   break;
     // std::this_thread::sleep_for (std::chrono::milliseconds(125));  /////////////debuig line
  }while(speechCommand != "DISABLE DISABLE"); 
   v.speakDetachable("I AM GOING OFFLINE NOW");
   //delete(s);delete(ch);delete(c);
  }
  catch(int a){
           cerr <<" \n *** Some thing wrong. ***\n ***ERROR CODE : "<<a<<" ErrorType : "<< errorCodeStrings[a-1] <<"***"<<endl;
  }
	
	cout<<" out of main();";
	cout.flush();

}


 /*else if(commandVector[0] == "ENABLE"){
                            stopListenning = false;                          
                             v.speakDetachable("NOW LISTENNING");
                        }*/

/*

ERROR AND THEIR MEANING

1 - SERIAL PORT NOT AVAILABLE
2 - PROBLEM ACQUIRING PORT PARAMETERS
3 - PROBLEM SETTING UP PORT PARAMETERS
4 - SONAR BAD ALLOCATION
5 - COMPASS QUEUEU BAD ALLOCATION
6 - CHARGE QUEUE BAD ALLOCATION
7 - SPHINX CONFIG ERROR 
8 - SPHINX INIT ERROR
9 - MIC NOT FOUND
10 - FAILED RECORDING
11 - VAD COULD NOT BE CALIBERATED
12 - BAD ALLOCATION FOR SPEECH QUEUE


*/

/* Notes regarding improvement of this program

  1. We have to write generic parse routines
  2. We have to have better constructor
  3. and most of the init stuffs should be brought out of the constructor which other wise as we see in the program 
    the whole stuff requires to in the try block.
  4. the class should provide ways to define no. of queues, their IDs, size and also mechanism to access them and parse them
  5. the class will impose few rules on the application programmer, e.g. to adhere to the basic sensor data format and command format
  6. Just like Pocketsphinx we can have debugging info which will be controlled by a flag
  7. we should some way for the recognizer to stop taking samples when we want to speak out loud for feedback
*/












