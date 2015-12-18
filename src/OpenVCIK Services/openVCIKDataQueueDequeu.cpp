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
 * Same as the openVCIKDataQueue just that it uses STL dequeue for Queue and not an array. 
 *
 * Note: Remember to use .data() or .c_str() with the returned data from the queue to be safe from null-termination issues
 *        as, to what ever extent I have read and heared it seems STL string doesnot guarantee null-terminated sequence.
 * 	
 * The dataQueue is EXCEPTION SAFE.
 * 
 * 		    Author: Junaid Ahmed Ansari
 *          email : ansariahmedjunaid@gmail.com
 *          github: junaidcs032
 *
 */

#include <iostream>       
#include <deque>          
#include <mutex> 	  
#include <string>
using namespace std;	

namespace OpenVCIK{

class dataQueue{

	std::deque<string> _myClassQueue;					// deque container to hold the actual queue
	int _size;
	mutex _queueLock;
	public:
		enum accessType{ CURRENT, NORMAL };				// defines enumeration for current or normal queue access
	public:
		// default constructor
		dataQueue():_size(10){}						// make the queue size 10 by default

		void setSize(int size){	_size = size; }

		int getSize(){	return _size; }					// return the defined size 
	
		int getQueueSize() {return _myClassQueue.size(); }		// return the Queue size

		void put(string data);						// to put in queue
		string get(accessType);						// to get from queue
};

void dataQueue::put(string data){
	
	if(_myClassQueue.size() < _size){

											// LOCK HERE
		lock_guard<mutex> lock(_queueLock);
		_myClassQueue.push_back(data);
				
	}										// UNLOCKED as it goes out of scope
	else{
											// LOCK HERE
		lock_guard<mutex> lock(_queueLock);
		_myClassQueue.clear();
		_myClassQueue.push_back(data);
								
	}										// UNLOCK as it goes out of scope
}
		
string dataQueue::get(accessType current){
	if(!_myClassQueue.empty()){
		if(current == NORMAL){
			string curr(_myClassQueue.front()); 
			{								// LOCK HERE
				lock_guard<mutex> lock(_queueLock);	
				_myClassQueue.pop_front();	
			}								// UNLOCK HERE					
			
			return curr;							// return the most updated data by copy
		}	
		else	
			return _myClassQueue.back();					// return via queue action by copy
	}	
	else{	
		return "";								// else return null
	}
}

} //end of namespace

/* The main program just to test the above class

int main ()
{
	dataQueue x;
	x.setSize(4);
	string y;
	x.put("1");
	x.put("2");
	x.put("3");
	x.put("4");
	x.put("5");
	
	cout<<x.get(dataQueue::NORMAL)<<endl;
	cout<<x.get(dataQueue::NORMAL)<<endl;
	cout<<x.get(dataQueue::NORMAL)<<endl;
	
	cout<<x.get(dataQueue::NORMAL)<<endl;
	cout<<x.get(dataQueue::NORMAL)<<endl;
	
	//string g(x.get(dataQueue::NORMAL));
	//g.erase(g.begin());
	//cout << g << g.size();	

}


*/