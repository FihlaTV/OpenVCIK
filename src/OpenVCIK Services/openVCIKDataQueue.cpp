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
 *  This class represents a CIRCULAR QUEUE for holding sensor data and member functions to
 *  read and write to the queue. It will have its own locking strucuture for multi-threaded
 *  programming.
 * 
 *  Limitations : For simplicity reasons, we dont have QUEUE RESIZE fascility as of now
 *                 
 *      Author : Junaid Ahmed Ansari
 *      email: ansariahmedjunaid@gmail.com
 *      github: junaidcs032
 * 
 */


#include <iostream>
#include <string>
#include <utility>

#ifndef DATA_QUEUE
#define DATA_QUEUE

namespace OpenVCIK{

using namespace std;

class dataQueue{
      
      string *_data;               // pointer for holding the array for QUEUE
      int _front;                  // this is used to put as well as to get the most recent queue data
      int _rear;                  // this is used by the get functn
      int _size;  
      string *_buff;
      
      mutex _queueLock;            // lock for mutual exclusion when accessing the queue

      public:
	
        // contsructor
        dataQueue(int);

        //destructor
        ~dataQueue(){
	 
          delete [] _data;
      	  //cout<<" deleted _data\n";
      	  //cout.flush();
        }  


        // puts the data on to the Front of queue. for details see definition.. below after class returns -1 for unsuccess
        int put(string data);

        // returns the data of the queue; For more detail check the definition
        string get(int  current);

        //clear the queue
        inline void clear();  

        // returns the current Front 
        inline int getFront(){
          int front;
          {
            lock_guard<mutex> guard(_queueLock);
            front = _front;
          }
          return front;
        }

        // returns the current Front 
        inline int getRear(){
          int rear;
          {
            lock_guard<mutex> guard(_queueLock);
            rear = _rear;
          }
          return rear;

        }

        //returns allocated size of the queue
        inline int getSize(){
          int size;
          {
            lock_guard<mutex> guard(_queueLock);
            size = _size;
          }
          return size;
        }

        //returns actual size of the queue at the point of this function call
        inline int getQueueSize(){
          int size;
          {
            lock_guard<mutex> guard(_queueLock);
            size = (_size - _front + _rear)%_size;
          }
          return size;

        }

};


// --------------------definitions--------------------------

/* contsructor
 * Arguement size - size of the queue. Defualt 10;
 * throws 1 if bad alloc
 */
dataQueue::dataQueue(int size=10): _front(-1), _rear(-1), _buff(NULL), _data(NULL){
    

      _size = (size >0)?(size):(10);        // making sure that the user doesnt pass queue size less than 0
      
      try{
            _data = new string [_size];
         }
        catch(bad_alloc){
            cerr << " In dataQueue.cpp: in constructor : BAD ALLOCATION EXCEPTION CAUGHT";
            throw (1);
        }
   
}


/* puts the data on in the circular queue.
 * if queue full it automatically clears it and starts a fresh
 * returns 0 if sucess , -1 if outOfBound proble. -1 will never happen it is just for safety
 */
int dataQueue::put(string data){
     
     int returnValue = 0;     

     {
          lock_guard<mutex> guard(_queueLock);

          if((_rear == _size-1 && _front == 0) || _rear+1 == _front ){   // If queue full then clear the queue

          	   _rear = -1;
                  _front = -1;
          }
          //else{
           

            if(_rear == -1){
              _front =0;
              _rear =0;
            }
            else if(_rear == _size -1){
              _rear = 0;
            }
            else
              _rear++;
            if(_rear >=0 && _rear <_size)    // just for safety
                _data[_rear] = data.data();  // making it null terminated... or rather for efficiency we can let the caller take care of it .. but not now
                                             // We can also check wethe ther is null in the end or not and then do a .data() call. by 
                                             // checking data[data.size+1] == NULL. but this might cause segFault
            else{
		returnValue = -1;
                /* debug msg*/
		//cerr << "In dataQueue.cpp: In put() : putting in outofbound - no operation performed" <<endl; 
            }
     }// unlocked here as it goes out of scope

     return (0);

}


/* returns the data from the queue 
 * returns null - if queue empty
 * Arguements : 0 - return data in a circular queue get fashion
 *              1 - returns the current data; doesn't effect the queue
 */
string dataQueue::get(int  current = 0){

      
      if(_front == -1){

        //cout << " Q queue empty";  
        return "\0";
      }
      else{

           
          { // just for scope
            // lock here
            lock_guard<mutex> guard(_queueLock);

            if(current>0){

              // return the current 
              if(_rear>=0 && _rear <_size)      // just recheck
                    _buff = &(_data[_rear]);    // no need to worry of null terminator its taken care in put()
              else{
		    /* debug msg */	
                    //cerr << "In dataQueue.cpp: In get() : in current action : accessing outofbound - exitting" <<endl;                    
              }
            }
            else{

                    if(_front>=0 && _front<_size)    // just recheck
                        _buff = &(_data[_front]);
                    else{
                         
	    	       /* debug msg */	
                       //cerr << "In dataQueue.cpp: In get() : in queue action: accessing outofbound - exitting" <<endl;                               
                    }


                    if(_front == _rear){
                        _front = -1;
                        _rear = -1;
                    }
                    else if(_front == _size-1)
                        _front = 0;
                    else
                        _front ++;
            }
          }// unlocked here
      
          // not using MOVE() here any furthur so whether current data or queue action its always a copy

          return *_buff;    // we will explore move once doen fixing the segFualt and double-free or corruption bug... HOPEFULLY


        /*if(current > 0)  
            return *_buff;              // by copy        
        else
            return (*_buff);        // move*/
      }
}

/* clear the queue */
void dataQueue::clear(){

    // lock here
    lock_guard<mutex> guard(_queueLock);
    _rear = -1;
    _front = -1;

    // unlocked here
}


} // end of namespace

#endif
