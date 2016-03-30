/*                                   LICENSE 
*                             --------------------
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

*   This work was done at the Raman Research Institute (RRI), Bangalore, India, and was supported by the
*   Department of Science and Technology (DST), India.

*   RRI hereby disclaims all copyright interest in the project OpenVCIK.
*/


/*                                                DESCRIPTOPN
 *                                          -----------------------
 * Firmware Program - This Arduino program contains all the required Classes for the HAL i.e. Hardware Abstraction Layer of the OpenVCIK framework. 
 * This program also uses fome AVR functions and port registers to speed the pin manipulation process.
 *
 * Author: Junaid Ahmed Ansari
 * email: ansariahmedjunaid@gmail.com 
 * github: junaidcs032
 *
 */


#include <avr/io.h>
#include <util/delay.h>    // for efficient busy wait activities delays are accurate refer the "protostack.com/..." link below
#include <Wire.h>
#define F_CPU 16000000UL    //This says to the compiler at what frequency our Atmega is running, in this case its 16Mhz
  
/* below are the variables and constants definition related to SENSOR class*/

/* look at this link for selecting appropriate PWM pins for motors : http://forum.arduino.cc/index.php/topic,72092.0.html */
/* below sonar pins are assigned according to the seconds version of the SENSOR_AGGREGATION BOARD */

#define VCIW_SONAR1_TRIG _BV(0)          // Port A
#define VCIW_SONAR1_READ (PINA & _BV(2))

#define VCIW_SONAR2_TRIG _BV(4)          // portA
#define VCIW_SONAR2_READ (PINA & _BV(6))

#define VCIW_SONAR3_TRIG _BV(7)          // portC
#define VCIW_SONAR3_READ (PINC & _BV(5))

#define VCIW_SONAR4_TRIG _BV(3)          // portC
#define VCIW_SONAR4_READ (PINC & _BV(1))

#define VCIW_SONAR5_TRIG _BV(7)          // portD
#define VCIW_SONAR5_READ (PING & _BV(1)) // portG

#define VCIW_SONAR6_TRIG _BV(7)          // port L
#define VCIW_SONAR6_READ (PINL & _BV(5))

#define VCIW_SONAR7_TRIG _BV(3)          // port L 
#define VCIW_SONAR7_READ (PINL & _BV(1))    

#define VCIW_SONAR8_TRIG _BV(3)          //port B
#define VCIW_SONAR8_READ (PINB & _BV(1))

/* port reset for sonars thats is OFF on the trig pins*/

#define VCIW_SONAR1_RESET (VCIW_SONAR1 & ~_BV(0))          // PortA
#define VCIW_SONAR2_RESET (VCIW_SONAR2 & ~_BV(4))          // portA
#define VCIW_SONAR3_RESET (VCIW_SONAR3 & ~_BV(7))          // portC
#define VCIW_SONAR4_RESET (VCIW_SONAR4 & ~_BV(3))          // portC
#define VCIW_SONAR5_RESET (VCIW_SONAR5_T & ~_BV(7))        // portD
#define VCIW_SONAR6_RESET (VCIW_SONAR6 & ~_BV(7))          // portL
#define VCIW_SONAR7_RESET (VCIW_SONAR7 & ~_BV(3))          // portL 
#define VCIW_SONAR8_RESET (VCIW_SONAR8 & ~_BV(3))          // portB

/* Port Definitions for SONARS specific to the MEGA shield */

#define VCIW_SONAR1          PORTA
#define VCIW_SONAR2          PORTA
#define VCIW_SONAR3          PORTC
#define VCIW_SONAR4          PORTC
#define VCIW_SONAR5_T        PORTD   
#define VCIW_SONAR5_R        PORTG   
#define VCIW_SONAR6          PORTL
#define VCIW_SONAR7          PORTL
#define VCIW_SONAR8          PORTB

/* charge sense */
#define VCIW_CHARGE          A0

/* Addresses of the COMPASS DEVICE HMC6352 from Sparksfun*/
#define HMC6352_SLAVE_ADDRESS 0x42
#define HMC6352_READ_ADDRESS  0x41  

// structure to hold sensor mask bytes
typedef struct SensorMask{
  unsigned char mask1;          // BITS :[7:0] -> mapping [SONAR1, SONAR2, SONAR3, SONAR4, SONAR5, SONAR6, SONAR7, SONAR8]
  unsigned char mask2;          // BITS :[7:0] -> mapping [COMPASS, ALL_SONAR, CHARGE, X, X, SEND_DATA, X, X]
};

unsigned long int initTime = 0;     // initial time. Offset. it is set to crrent millis once " * " command is passed from the HOST

typedef struct timeStamp{
  
      long int h;            // hour
      long int m;
      long int s;
      unsigned long int ms;           // millisecond 
};

timeStamp computeTimeStamp();         // prototype for the time stamp computation func.. declared in the global code area.. after the classes

enum Masks {MASK1, MASK2};    

/* below are the structures... etc.. related to RRIWheelchair class*/  
struct motorDrive{        // structure for motor drive info
    byte motor1_IN1;
    byte motor1_IN2;
    byte motor1_PWM;
    
    byte motor2_IN1;
    byte motor2_IN2;
    byte motor2_PWM;  
};

struct motionParamsPWM{   // structure holding PWM values of both motors  
    byte left; 
    byte right;
};
    
enum stopType{ FAST, FREE};

/* --------END OF #DEFINES, ENUMS definitions, and STRUCTURES ----------------*/





/* ***************************************************************************
 * class for manipulatin the GPIOs of OpenVCIK       
 * Optimization Notes: we can use the PIN_MANIPULATION to reduce function call 
 *                     overhead as done in Sonar Sensors
 *****************************************************************************/

enum Type{DIGITAL_IN = 0, DIGITAL_IN_PULLUP, DIGITAL_OUT};

class GPIO{
  
  byte *_pinNoArray;      // the array containing the actual pin number of arduino in the index location corresponding to the OpenVCIK GPIO pin number
  byte _numPins;          // total numner of GPIO pins
  
  public:

    //GPIO(byte pwmPS, byte pwmPE): _pwmPinStart(pwmPS),_pwmPinEnd(pwmPE){

    //constructor
    GPIO(){
      
      // note: we have only 14 many pins in this Version of Kit
      _numPins = 12;      
      
      // init the pin array
      _pinNoArray = new byte[_numPins];
      
      // load the arduino pin numbers to the correspionding OPenVCIK GPIO no.
      for(int i = 0; i<_numPins; ++i)
        _pinNoArray[i] = i+2;                  
    
      // by defualt the GPIO pins are in DIGITAL_OUTPUT
      setType(0,11, DIGITAL_OUT);
      writeDigital(0,11, LOW);

    }
    
    //set the Type of an individual GPIO pin
    void setType(byte pin, Type type){

      // validate the type
      if(type >=0 && type <= 2){
          
        // validating GPIO pin numbers
        if( pin>=0 && pin <_numPins){
        
          switch(type){
            
            case DIGITAL_IN_PULLUP:                            
                                pinMode(_pinNoArray[pin], INPUT_PULLUP);      // again we will give this functionalities to the higher levels in this versioin
                                break;
                              
            case DIGITAL_IN:
                                pinMode(_pinNoArray[pin], INPUT);      
                                break;
            case DIGITAL_OUT:
                                pinMode(_pinNoArray[pin], OUTPUT);      
                                break;
          }
        }
      }
    }

    // sets the type of a range of GPIO pins.
    void setType(byte from, byte to, Type type){

      //vaidate the type 
      if(type >= 0 && type <=2){
          
        // validating GPIO pin numbers
        if(from >=0 && to <_numPins){
        
          switch(type){
            
            case DIGITAL_IN_PULLUP:
                              for(int i=from; i<=to; ++i)
                                pinMode(_pinNoArray[i], INPUT_PULLUP);      // again we will give this functionalities to the higher levels in this versioin
                              break;
                              
            case DIGITAL_IN:
                              for(int i=from; i<=to; ++i)
                                pinMode(_pinNoArray[i], INPUT);      // again we will give this functionalities to the higher levels in this versioin
                              break;
            case DIGITAL_OUT:
                              for(int i=from; i<=to; ++i)
                                pinMode(_pinNoArray[i], OUTPUT);      
                              break;
          }
          
        }
      }
    }
   
    // write to range of GPIO pins in digital mode:
    void writeDigital(byte from, byte to, boolean value){
      
      // validate the GPIO pin numbers
      if(from>=0 && to<_numPins){
        
        for(int i=from; i<=to; ++i)
            digitalWrite(_pinNoArray[i], value);
      }
    }

    // write to an individual pin of GPIO in digital mode:
    void writeDigital(byte pin, boolean value){
      
      // validate the GPIO pin numbers
      if(pin>=0 && pin<_numPins){
        
          digitalWrite(_pinNoArray[pin], value);
      }
    }

    // write to range GPIO pins in PWM mode:
    void writePWM(byte from, byte to, byte value){
      
      // validate the GPIO pin numbers
      if(from>=0 && to<_numPins){
        
        for(int i=from; i<=to; ++i)
          analogWrite(_pinNoArray[i], value);
      }
      
    }  

    // write to an individual GPIO pins in PWM mode:
    void writePWM(byte pin, byte value){
      
      // validate the GPIO pin numbers
      if(pin>=0 && pin<_numPins){
        analogWrite(_pinNoArray[pin], value);
      }
      
    } 
   
    // this function will read the GPIO and send them as some character
    void read(){}
 
 
};

/*  --------------------- END OF THE GPIO CLASS ------------------------------*/


/* Class RRIWheelchair - This class bundles all the motion control including the PI         *
 *                       control of motors of the Test Wheelchair used for demonstration.   *
 * It interfaces with Motor Driver - NexRobotics' Hercules 6V-36V 16Amp Motor Driver        *  
 */

class RRIWheelchair{
  
  private:
    int _pwmLevel;                           // defines the level PWM of the motors;
    int _pwmMaxDev;                          // defines the dev of the PWM of the motors from the level pwm;

    float _Kp;                          
    float _Ki;                               
    int _integralError;                      // for total integral error make it zero
    int _wheelchairMode;                     // maintains the current mode of the chair
    motionParamsPWM _wheelchairMotorPWM;     // for left and right PWM values
    motorDrive _wheelchairMotorDrive;        // structure holding pins info for motordrivers 
    boolean _PIControlFlag;
    
    // inits the motor drivers related pins
    void initDrive(){
        _wheelchairMotorDrive.motor1_IN1 = 8;
        _wheelchairMotorDrive.motor1_IN2 = 10;
        _wheelchairMotorDrive.motor1_PWM = 9;
        _wheelchairMotorDrive.motor2_IN1 = 12;
        _wheelchairMotorDrive.motor2_IN2 = 13;
        _wheelchairMotorDrive.motor2_PWM = 11;  
       
       
        pinMode(_wheelchairMotorDrive.motor1_IN1, OUTPUT);
        pinMode(_wheelchairMotorDrive.motor1_IN2, OUTPUT);
        pinMode(_wheelchairMotorDrive.motor1_PWM, OUTPUT);
        pinMode(_wheelchairMotorDrive.motor2_IN1, OUTPUT);
        pinMode(_wheelchairMotorDrive.motor2_IN2, OUTPUT);
        pinMode(_wheelchairMotorDrive.motor2_PWM, OUTPUT);  
        
        digitalWrite(_wheelchairMotorDrive.motor1_IN1, HIGH);
        digitalWrite(_wheelchairMotorDrive.motor1_IN2, HIGH);
        analogWrite(_wheelchairMotorDrive.motor1_PWM, 0);
        digitalWrite(_wheelchairMotorDrive.motor2_IN1, HIGH);
        digitalWrite(_wheelchairMotorDrive.motor2_IN2, HIGH);
        analogWrite(_wheelchairMotorDrive.motor2_PWM, 0);  
    }
  
  public:
  
      // constructor
      RRIWheelchair(){
         _pwmLevel = 30;
         _pwmMaxDev = 30;
         _Kp = 3;
         _Ki = 0.05;
         _wheelchairMode = -1;     // stopped condition.    
       
         _wheelchairMotorPWM.left = 0;
         _wheelchairMotorPWM.right = 0;
         
         _PIControlFlag = true;
          initDrive();   
      }
      
      // sets or resets PID control flag
      void setPIControlFlag(boolean flag){
        _PIControlFlag = flag;
      }
      
      // set the pwm values expelcitly.. give direct control over pwm values
      void setPWMValues(byte left, byte right){
        
        _wheelchairMotorPWM.left = left;
        _wheelchairMotorPWM.right = right;
      }
      
      // pass the left,right pwm values to the motor driver. i.e. apply pwm control signals.
      void actuate(){

        analogWrite(_wheelchairMotorDrive.motor1_PWM,_wheelchairMotorPWM.left);
        analogWrite(_wheelchairMotorDrive.motor2_PWM,_wheelchairMotorPWM.right);    
      }
      // set the gains
      void setGains(float Kp, float Ki){ 
        _Kp = Kp;
        _Ki = Ki;
      }
      
      // sets the PWM parameters used by the control procedure.. level and deviation of PWM
      void setPWMParameters(int level, int dev){
          _pwmLevel = level;
          _pwmMaxDev = dev; 
      }
      
      // Setting the MODE of the wheelchair that is FORWARD OR LEFT OR RIGHT OR SPIN etc     
      // Parameters : wheelchair Mode   [-1 - STOP, 0-for, 1-lspin , 2-rspin ]               
      void setWheelchairMode(int m){ 
      
        if(m!=_wheelchairMode){
          switch(m){
            case -1:                               //stopMode              
                    _wheelchairMode = -1; 
                    break;
            case 0:                                //FORWARD
                    stopWheelchair(FREE);
                    _delay_ms(20);
                    digitalWrite(_wheelchairMotorDrive.motor1_IN1, LOW);
                    digitalWrite(_wheelchairMotorDrive.motor2_IN1, LOW);
                    
                    digitalWrite(_wheelchairMotorDrive.motor1_IN2, HIGH);
                    digitalWrite(_wheelchairMotorDrive.motor2_IN2, HIGH);
                    _wheelchairMode = 0;
                    break;
            case 1:                                                                    // backward
                    stopWheelchair(FREE);                  
                    _delay_ms(20);              
                    digitalWrite(_wheelchairMotorDrive.motor1_IN1, HIGH);
                    digitalWrite(_wheelchairMotorDrive.motor2_IN1, HIGH);
                    
                    digitalWrite(_wheelchairMotorDrive.motor1_IN2, LOW);
                    digitalWrite(_wheelchairMotorDrive.motor2_IN2, LOW);
                    _wheelchairMode = 1;
                    break;
            case 2:                                                          
                    stopWheelchair(FREE);                                            //rsping
                //    Serial.println("CLK");              
                    _delay_ms(20);
                    digitalWrite(_wheelchairMotorDrive.motor1_IN1, LOW);
                    digitalWrite(_wheelchairMotorDrive.motor2_IN2, LOW);
                    
                    digitalWrite(_wheelchairMotorDrive.motor1_IN2, HIGH);
                    digitalWrite(_wheelchairMotorDrive.motor2_IN1, HIGH);
                    _wheelchairMode = 2;
                    break;
            case 3:                                                            // lspin
                    stopWheelchair(FREE);              
                    _delay_ms(20);
                    digitalWrite(_wheelchairMotorDrive.motor1_IN1, HIGH);
                    digitalWrite(_wheelchairMotorDrive.motor2_IN2, HIGH);
                    
                    digitalWrite(_wheelchairMotorDrive.motor1_IN2, LOW);
                    digitalWrite(_wheelchairMotorDrive.motor2_IN1, LOW);
                    _wheelchairMode = 3;
                    break;              
          }
        }
      }
      
      // returns the current wheelchair mdoe
      int getWheelchairMode(){
        return _wheelchairMode;
      }
      
      // Stops the wheelchari and set the wheelchair mode to -1. Arguements : FAST|FREE, FAST locks the motors and FREE just cuts the signal
      void stopWheelchair(stopType type){
        // first stop signal and then decide we are going to FAST STOP it or FREE it
        analogWrite(_wheelchairMotorDrive.motor1_PWM,0);
        analogWrite(_wheelchairMotorDrive.motor2_PWM,0);
      
        switch(type){
          case FAST:
                      _wheelchairMotorPWM.left = 0;
                      _wheelchairMotorPWM.right = 0;    
                      digitalWrite(_wheelchairMotorDrive.motor1_IN1, HIGH);
                      digitalWrite(_wheelchairMotorDrive.motor1_IN2, HIGH);
                      digitalWrite(_wheelchairMotorDrive.motor2_IN1, HIGH);
                      digitalWrite(_wheelchairMotorDrive.motor2_IN2, HIGH);                
                      break;
          case FREE:
                      _wheelchairMotorPWM.left = 0;
                      _wheelchairMotorPWM.right = 0;
        }
        
        _integralError = 0;
        _wheelchairMode = -1;
      }
      
      //close loop control of the direction of the wheelchair using the compass readings 
      // arguements are ctrlTIme thats is the loop time, currentHeading whcih is wheelchairHeading, and desired heading
      void control(int ctrlTime, int wheelchairHeading, int desiredHeading){
        
        //if PID control is wanted ...         
        if(_PIControlFlag){  
       
          int errorPWM=0;
          int errorClk = 0;
          int errorAClk = 0;
        
          boolean isCorrectionRequired = false; // checks if error exists further
        
        //First calculating the error in angle from both SIDES OF ACTUATION - clock wise and anti_clockWise
          if(desiredHeading > wheelchairHeading){  
            errorClk = desiredHeading - wheelchairHeading;
            errorAClk = wheelchairHeading + (360 - desiredHeading);
            isCorrectionRequired = true;      
            
          }
          else if(desiredHeading < wheelchairHeading){  
            errorClk = 360 - wheelchairHeading + desiredHeading;
            errorAClk = wheelchairHeading - desiredHeading;
            isCorrectionRequired = true;
          }
          else{
            _wheelchairMotorPWM.left = _pwmLevel;
            _wheelchairMotorPWM.right = _pwmLevel;   
            _integralError = 0;                   // nutraliZing the integral error   
            isCorrectionRequired = false;
            //nop
          }    
          
          // then appl;ying appropriate copntrol on approporate wheel
          if(isCorrectionRequired){
            if(errorClk <= errorAClk) {   // if clock wise is the adequate option
           //   Serial.println("CLOCK");
              if(_wheelchairMode!=0) setWheelchairMode(2);
              // we might have to nutralize the integral Error
              
              if(errorClk > 2){
                _integralError = _integralError + errorClk;
              }
              else  
                _integralError = 0;     // rounding the integral error to 0 if error is less that 2
                
              errorClk = (float(errorClk) *_Kp) + (float(_integralError) * _Ki*ctrlTime);
              errorPWM = map(errorClk,1,90,1,_pwmMaxDev);
              if(errorPWM<=0) errorPWM = 0;
              if(errorPWM >(_pwmMaxDev)) errorPWM = _pwmMaxDev;
              
              _wheelchairMotorPWM.left  = abs(_pwmLevel + errorPWM);
              _wheelchairMotorPWM.right = abs(_pwmLevel - errorPWM);
            }
            else{
            //  Serial.println("ANTI_CLOCK");        
              if(_wheelchairMode!=0) setWheelchairMode(3);        
              
              if(errorAClk > 2){
                          _integralError = _integralError + errorAClk;
              }
              else  
                _integralError = 0;      // rounding the integral error to 0 if error is less 2
                
                
              errorAClk = (float(errorAClk) *_Kp) + (float(_integralError) * _Ki*ctrlTime);
              errorPWM = map(errorAClk,1,90,1,_pwmMaxDev);
              
              if(errorPWM<=0) errorPWM = 0;
              if(errorPWM >(_pwmMaxDev)) errorPWM = _pwmMaxDev;
              
              _wheelchairMotorPWM.left  = abs(_pwmLevel - errorPWM);
              _wheelchairMotorPWM.right = abs(_pwmLevel + errorPWM); 
            }
          }
        }

      }
      
      
};

/* --------------------------- End of the RRIWheelchair Class -----------------------------*/



/*  Sensors : Class that bundles all implementaions related to the Sensors of the OpenVCIK    *
 *            viz. SONARS, COMPASS, ENCODERS, CHARGE. It has APIs to access the sensors and   *
 *            their data                                                                      */
 
class Sensors{

    private:
          
          long int _timeTaken[8][3];                 // for storing time take for ranging 
          unsigned long int _timeout;                // in microsecods
          unsigned long int _timeoutCompass;         // timeout for compass data to be retrieved. in milliseconds
          int _maxDistance;                          // in cm
          float _timeToDistanceFactor;               // the factor which on multiplying converts time to distance
          float _sampleToVoltage;                    // the factor to convert the sampled value to voltage in 12v
          String _sensorDataString;
          int _sensorData[12];                       // all the sensor readings are stored. Masked are 0. mapping : [Sonar1:Sonar8,COMPASS,CHARGE]
          timeStamp _ts;                             // timestamp
          int _i,_j;
          SensorMask _mask;                          // holds the mask
        
      /* function to write the sensor data in CSV format to the Serial COMM in the way 
       * Sonar sensor data will start with an "S" i.e. eg. $S1:2:3:3,12,12,12,12,12,0,0,0.
       */
          void sendSonarData(timeStamp ts){
                     
                    _sensorDataString  = "$";
                    
                    // put the time stamp
                    _sensorDataString = _sensorDataString + "S" + ts.h + ":" + ts.m + ":" + ts.s + ":" + ts.ms + ",";
                    
                    // prepare the data string from the data[]
                    for(_i=0; _i<7; ++_i){
                        _sensorDataString = _sensorDataString + _sensorData[_i] + ",";  
                    }
                    _sensorDataString = _sensorDataString + _sensorData[_i] + ".";
                    
                    char buf[_sensorDataString.length()+1]; 
                    
                    _sensorDataString.toCharArray(buf, _sensorDataString.length()+1);      // convert String to Char Array for Serial.write () function
                    
                    Serial.println(buf);                                                   // write to COMM                    
          }
      
      /* function to write the sensor data in CSV format to the Serial COMM in the way : "&<1>,<2>,   ."
       * Sonar COMPASS data will start with an "C" i.e. eg. $C1:2:3:3,111.
       */
          void sendCompassData(timeStamp ts){
      
                   // init the sensorDatraString variable
                    _sensorDataString = "$";
                    
                    // put the time stamp
                    _sensorDataString = _sensorDataString + "C" + ts.h + ":" + ts.m + ":" + ts.s + ":" + ts.ms + ",";
                    
                    _sensorDataString = _sensorDataString + _sensorData[8] + ".";
                    
                    char buf[_sensorDataString.length()+1]; 
                    
                    _sensorDataString.toCharArray(buf, _sensorDataString.length()+1);      // convert String to Char Array for Serial.write () function
                    Serial.println(buf);                                                 // write to COMM                    
          }
          
      /* function to write the CHARGE data in CSV format to the Serial COMM in the way : "&<1>,<2>,   ."
       * CHARGE data will start with an "V" i.e. eg. $V1:2:3:3,2. -- charge data is in % i.e. 0-100 percent
       */

          void sendChargeData(timeStamp ts){
      
                   // init the sensorDatraString variable
                    _sensorDataString = "$";
                    
                    // put the time stamp
                    _sensorDataString = _sensorDataString + "V" + ts.h + ":" + ts.m + ":" + ts.s + ":" + ts.ms + ",";
                    
                    _sensorDataString = _sensorDataString + int((_sensorData[9]/1024)*100) + ".";
                    
                    char buf[_sensorDataString.length()+1]; 
                    
                    _sensorDataString.toCharArray(buf, _sensorDataString.length()+1);      // convert String to Char Array for Serial.write () function
                    Serial.println(buf);                                                 // write to COMM                    
          }
          
          
    public:        
    
/* Constructor */

    Sensors(){   
      
      // by default set the compass timeout to be 11 ms
      setCompassTimeout(11);
      // init stuffs 
      _mask.mask1 = B11111111;
      _mask.mask2 = B11111111;
    }
    
/* set/Reset i.e. loads the passed bytes to both the mask bytes */

    void setMasks(unsigned char m1, unsigned char m2)
    {
      _mask.mask1 = m1;
      _mask.mask2 = m2;
    }
    
/* set/Reset i.e. loads the passed byte to specific mask illustrated by the maskingType*/

    void setSpecificMask(Masks mask,unsigned char m){  
      switch(mask){
                
        case MASK1:
                    _mask.mask1 = m;
                    break;
        
        case MASK2:
                    _mask.mask2 = m;
                    break;
      }    
    }
    
/* function for setting the parameters related to ranging 
 * maxdist - maximum distance we want to range
 * factor - is the conversion factor from time_of_travel of sound waves to actual distance
 */
 
    void setSonarParameters(int maxdist, float factor){
            _maxDistance = maxdist;
            _timeout = int(float(maxdist)/factor);
            _timeToDistanceFactor = factor;
      
    }      
        
/* initiallize the pins for sonars */

    void initSonars(){
   
            // setting all the sonar trig pins as output 
            // for port information see above comments in #define section
          
            DDRA = DDRA | _BV(0);
            DDRA = DDRA | _BV(4);
            DDRC = DDRC | _BV(7);
            DDRC = DDRC | _BV(3);
            DDRD = DDRD | _BV(7);
            DDRL = DDRL | _BV(7);
            DDRL = DDRL | _BV(3);
            DDRB = DDRB | _BV(3);
            
            // set them all low
            VCIW_SONAR1      = VCIW_SONAR1_RESET;
            VCIW_SONAR2      = VCIW_SONAR2_RESET;
            VCIW_SONAR3      = VCIW_SONAR3_RESET;
            VCIW_SONAR4      = VCIW_SONAR4_RESET;
            VCIW_SONAR5_T    = VCIW_SONAR5_RESET;
            VCIW_SONAR6      = VCIW_SONAR6_RESET;
            VCIW_SONAR7      = VCIW_SONAR7_RESET;
            VCIW_SONAR8      = VCIW_SONAR8_RESET;
            
          
            // setting all sonar read pins as input
            DDRA = DDRA & ~_BV(2);
            DDRA = DDRA & ~_BV(6);
            DDRC = DDRC & ~_BV(5);
            DDRC = DDRC & ~_BV(1);
            DDRG = DDRG & ~_BV(1);
            DDRL = DDRL & ~_BV(5);
            DDRL = DDRL & ~_BV(1);
            DDRB = DDRB & ~_BV(1);
            
           // digitalWrite(25, LOW);
      
    }
 
/* Reads the all the 8 sonars simultaneously and puts them in the sensorData array, also if SEND_DATA bit set, it sends the sensor 
 * data to the SERVICE LAYER 
 * 
 * Note: always init the timeTaken array before you run this function as the junk/previous data in the array might currurpt the data
 */
 
    void readSonars(){
             
            // chech if ALL_SONAR MASK TRUE|FALSE. Then while reading individual sonars we have individual SONAR mask..
            if(_mask.mask2 & _BV(6)){ 
                unsigned long int startTime;
                unsigned long int timeElapsed;
          
                // reset the _timeTaken Array
                //    start time           endtime          doneflag
                _timeTaken[0][0] = _timeTaken[0][1] = _timeTaken[0][2] = -1; 
                _timeTaken[1][0] = _timeTaken[1][1] = _timeTaken[1][2] = -1;
                _timeTaken[2][0] = _timeTaken[2][1] = _timeTaken[2][2] = -1;
                _timeTaken[3][0] = _timeTaken[3][1] = _timeTaken[3][2] = -1;
                _timeTaken[4][0] = _timeTaken[4][1] = _timeTaken[4][2] = -1;
                _timeTaken[5][0] = _timeTaken[5][1] = _timeTaken[5][2] = -1;
                _timeTaken[6][0] = _timeTaken[6][1] = _timeTaken[6][2] = -1;
                _timeTaken[7][0] = _timeTaken[7][1] = _timeTaken[7][2] = -1;
                      
                VCIW_SONAR1      = VCIW_SONAR1_RESET;
                VCIW_SONAR2      = VCIW_SONAR2_RESET;
                VCIW_SONAR3      = VCIW_SONAR3_RESET;
                VCIW_SONAR4      = VCIW_SONAR4_RESET;
                VCIW_SONAR5_T    = VCIW_SONAR5_RESET;
                VCIW_SONAR6      = VCIW_SONAR6_RESET;
                VCIW_SONAR7      = VCIW_SONAR7_RESET;
                VCIW_SONAR8      = VCIW_SONAR8_RESET;
                
                _delay_us(10);
                
                //Read the expresssions as : 
                //                 if(sonar1 not masked)then {Trigger it} else {leave it as it is}
                VCIW_SONAR1      |= ((_mask.mask1 & _BV(7))?(VCIW_SONAR1_TRIG):(0));
                VCIW_SONAR2      |= ((_mask.mask1 & _BV(6))?(VCIW_SONAR2_TRIG):(0));
                VCIW_SONAR3      |= ((_mask.mask1 & _BV(5))?(VCIW_SONAR3_TRIG):(0));
                VCIW_SONAR4      |= ((_mask.mask1 & _BV(4))?(VCIW_SONAR4_TRIG):(0));
                VCIW_SONAR5_T    |= ((_mask.mask1 & _BV(3))?(VCIW_SONAR5_TRIG):(0));
                VCIW_SONAR6      |= ((_mask.mask1 & _BV(2))?(VCIW_SONAR6_TRIG):(0));
                VCIW_SONAR7      |= ((_mask.mask1 & _BV(1))?(VCIW_SONAR7_TRIG):(0));
                VCIW_SONAR8      |= ((_mask.mask1 & _BV(0))?(VCIW_SONAR8_TRIG):(0));
    
                _delay_us(10);                                                     // according to the data sheet we should provide a 10uS pulse 
    
                VCIW_SONAR1      = VCIW_SONAR1_RESET;
                VCIW_SONAR2      = VCIW_SONAR2_RESET;
                VCIW_SONAR3      = VCIW_SONAR3_RESET;
                VCIW_SONAR4      = VCIW_SONAR4_RESET;
                VCIW_SONAR5_T    = VCIW_SONAR5_RESET;
                VCIW_SONAR6      = VCIW_SONAR6_RESET;
                VCIW_SONAR7      = VCIW_SONAR7_RESET;
                VCIW_SONAR8      = VCIW_SONAR8_RESET;
                
                _delay_us(2);
                
                startTime = micros();                      // init the time for time out calculation
                
                
                // below in every read operation ... we check SONAR<number> mask true or not.. that is individual mask
                do{                                                        
    
                  if(!(VCIW_SONAR1_READ && (_mask.mask1 & _BV(7))) && _timeTaken[0][0] >= 0 && _timeTaken[0][2] == -1){
                      _timeTaken[0][1] = micros() - _timeTaken[0][0];
                      _timeTaken[0][2] = 0;

                  }
    
                  else if((VCIW_SONAR1_READ && (_mask.mask1 & _BV(7))) && _timeTaken[0][0] == -1 ){
                      _timeTaken[0][0] = micros();                  
                  }
                      
                  if(!(VCIW_SONAR2_READ && (_mask.mask1 & _BV(6))) && _timeTaken[1][0] >= 0 && _timeTaken[1][2] == -1){
                      _timeTaken[1][1] = micros() - _timeTaken[1][0];
                      _timeTaken[1][2] = 0;
                  }
                  
                  else if((VCIW_SONAR2_READ && (_mask.mask1 & _BV(6))) && _timeTaken[1][0] == -1)
                      _timeTaken[1][0] = micros();
                      
                  if(!(VCIW_SONAR3_READ && (_mask.mask1 & _BV(5))) && _timeTaken[2][0] >= 0 && _timeTaken[2][2] == -1) {
                      _timeTaken[2][1] = micros() - _timeTaken[2][0];
                      _timeTaken[2][2] = 0;
                  }
    
                  else if((VCIW_SONAR3_READ && (_mask.mask1 & _BV(5))) && _timeTaken[2][0] == -1)
                      _timeTaken[2][0] = micros();
                    
                  if(!(VCIW_SONAR4_READ && (_mask.mask1 & _BV(4))) && _timeTaken[3][0] >= 0 && _timeTaken[3][2] == -1){
                      _timeTaken[3][1] = micros() - _timeTaken[3][0];
                      _timeTaken[3][2] = 0;
                  }
    
                  else if((VCIW_SONAR4_READ && (_mask.mask1 & _BV(4))) && _timeTaken[3][0] == -1)
                      _timeTaken[3][0] = micros();
                    
                  if(!(VCIW_SONAR5_READ && (_mask.mask1 & _BV(3))) && _timeTaken[4][0] >= 0 && _timeTaken[4][2] == -1){
                      _timeTaken[4][1] = micros() - _timeTaken[4][0];
                      _timeTaken[4][2] = 0;
                  }
    
                  else if((VCIW_SONAR5_READ && (_mask.mask1 & _BV(3))) && _timeTaken[4][0] == -1)
                      _timeTaken[4][0] = micros();
                    
                  if(!(VCIW_SONAR6_READ && (_mask.mask1 & _BV(2))) && _timeTaken[5][0] >= 0 && _timeTaken[5][2] == -1){
                      _timeTaken[5][1] = micros() - _timeTaken[5][0];
                      _timeTaken[5][2] = 0;
                  }
    
                  else if((VCIW_SONAR6_READ && (_mask.mask1 & _BV(2))) && _timeTaken[5][0] == -1)
                      _timeTaken[5][0] = micros();
                    
                  if(!(VCIW_SONAR7_READ && (_mask.mask1 & _BV(1))) && _timeTaken[6][0] >= 0 && _timeTaken[6][2] == -1){
                      _timeTaken[6][1] = micros() - _timeTaken[6][0];
                      _timeTaken[6][2] = 0;
                  }
    
                  else if((VCIW_SONAR7_READ && (_mask.mask1 & _BV(1))) && _timeTaken[6][0] == -1)
                      _timeTaken[6][0] = micros();
                    
                  if(!(VCIW_SONAR8_READ && (_mask.mask1 & _BV(0))) && _timeTaken[7][0] >= 0 && _timeTaken[7][2] == -1){
                      _timeTaken[7][1] = micros() - _timeTaken[7][0];
                      _timeTaken[7][2] = 0;
                  }
    
                  else if((VCIW_SONAR8_READ && (_mask.mask1 & _BV(0))) && _timeTaken[7][0] == -1)
                      _timeTaken[7][0] = micros();
                              
                }while((micros()-startTime) <= _timeout);
                
                //compute the timestamp
                
               _ts = computeTimeStamp();
                
                // now we start the distance calculation from here
                
                for(_i = 0 ; _i<8 ; ++_i){
                      
                      if(_timeTaken[_i][1] >0 &&_timeTaken[_i][0] >=0){ 
                          // use the factor to convert the time into distance
                          _sensorData[_i] = int (float(_timeTaken[_i][1]) * _timeToDistanceFactor);
                      }
                      else{
                          _sensorData[_i] = 0;
    
                      }
                }
              
                // if SEND_DATA bit 1
                if( _mask.mask2 & _BV(2) ){ 
                    sendSonarData(_ts);
                }    
            }
            //delay(3000);
    }
    
/* set compass timeout. This is used while waiting for the I2C data to arrive from compass while reading compass. So if timeout then data is -1 */

    void setCompassTimeout(int timeout){
      
      _timeoutCompass = timeout;
    }
    
/* Function read compass using I2C protocol making use of arduino WIRE libraryr
 * Note: Its a bsy wait based library. results -1 if masked 
 */ 
    void readCompass(){
             int num_bytes_read = 0;
             // check for the COMPASS mask. If true only then read. 
             if(_mask.mask2 & _BV(7)){  
                 
                 unsigned long int init_time=0;
                 boolean data_available=false;
                 
                 Wire.beginTransmission(HMC6352_SLAVE_ADDRESS >> 1); // begin I2C comm
                 Wire.write(HMC6352_READ_ADDRESS);              // The "Get Data" command
                 Wire.endTransmission();  
                 
                 Wire.requestFrom(HMC6352_SLAVE_ADDRESS >> 1, 2); //get the two data bytes, MSB and LSB
                 init_time = millis();
                 do{                         // wait for data to come. Also estimate the time so that it can be documented. Probably we must see the datasheet.
                                             //The heading output data will be the value in tenths of degrees from zero to 359 and provided in binary format over the two bytes.
                  
                    if((millis() - init_time) > _timeoutCompass){
                        data_available = false;  
                        break;
                          
                    }
                    data_available = true;

                 }while(Wire.available()==0);
                 if(data_available){
                   // compute time stamp on data arrival
                   _ts = computeTimeStamp();
                   
                   byte MSB = Wire.read();
                   byte LSB = Wire.read();
                
                   float headingSum = (MSB << 8) + LSB; //(MSB / LSB sum)
                   float headingInt = headingSum / 10;
                
                   _sensorData[8] = (int(headingInt)); 
                 }
                 else{
                   _sensorData[8] = -1;   // if no data retrieved
                 }
             
            // if SEND_DATA bit 1
            if( _mask.mask2 & _BV(2) ){                   
               sendCompassData(_ts);
            }
          }   

    }
 
    
/* Senses charge of the 12 V battery, and send it in percentage equivalent i.e. percentage charge left.
 * returns 0 if charge 0 or nothing connected
 */
    void readCharge(){
      
              // check for the CHARGE MASK.
              if(_mask.mask2 & _BV(5)){
                  
                  _sensorData[9] = int(_sampleToVoltage * float(analogRead(VCIW_CHARGE)));
          
                  // compute time stamp
                  _ts = computeTimeStamp();
                  
                  // if SEND_DATA bit 1
                  if( _mask.mask2 & _BV(2) ){    
                      sendChargeData(_ts);                   
                  }
              }
              
//              _sensorData[9] = -1;
              
    }

/* function to access the complete sensor data by reference. 
 * Returns - an array of 10 elements [1:8 sonar, compass, charge] by reference
 */
    const int* getSensorData(){
      
             return _sensorData;
    }
    

    /* NOTE : We will also have a global masking in the funciton where all the sensor read
     * functions are being called for efficiency 
     */
      
};

/* ----------------------------- End of the Sensor Class ---------------------------------*/



/**************************************************************************************************************************************** */
/*                                           UTILITY FUNCTIONS AND RELEVANT VARIABLES                                                     */
/*            Note: These functions are used for specific purpose for example parseCommand() parse the commands sent by the users via the */
/*                  Service Layer.                                                                                                        */  
/**************************************************************************************************************************************** */

// related to command generation 
boolean cmdAvailable = false;            // Command Available/not flag
String cmd="";                           // buffer holding command     
char ch;       
int desiredHeading = 0;
boolean isControlRequired = false;       // this will be used to decide if control should be executed.
boolean isCommandAvailable = false;          
int controlFrequency = 0;                // control frequency in milliseconds
boolean suppressWheelchair = false;      // flag to control active/suppress of wheelchair interface

const int *sensorReadings;               // will store the sensor data [sonar1:8,compass,charge]
boolean startSCALoop = false;

/* Objects of the CLASSES defined above */

RRIWheelchair wheelchair;                
Sensors sensors;
GPIO gpio;


// below two variables will be used to keep time for passing the time for control loop function
unsigned long int prevTime;
unsigned long int controlTime;

// function to compute the Time Stamp and return it
timeStamp computeTimeStamp(){
      
      timeStamp t; 
      unsigned long int rh, rm,rms;                                          // variables to store remaineders for Time Stamp computing
      long unsigned int currTime = millis()- initTime;    // gather cuurent in time in millis and offeseted by the init time

      t.s = (currTime/1000);

      t.ms =  (currTime%1000);
      
      t.m = (t.s/60);
      t.s = (t.s % 60);
      
      t.h = t.m/60;
      t.m = t.m%60;
      
      return t;
}



// this function is temporary included just for testing. It should and will be removed as it will slow the loop by 200 ms once a command has come
// GPIO pin No. 5 
void blinkLED(){
   
   digitalWrite(7, HIGH);
   _delay_ms(50);
   digitalWrite(7,LOW);
   _delay_ms(50);
   digitalWrite(7, HIGH);
   _delay_ms(100);
   digitalWrite(7,LOW);
}


/* Event based serial routine for reading serial data*/
// The command format is <command>. 
void serialEvent(){
  char ch;
  if(!isCommandAvailable){
    ch = Serial.read();
    
      // syncing the timestamp
      if(ch == '%'){
           //  delay(100);
             Serial.println('%');
      }
      else if(ch == '@'){
        initTime = millis();
        startSCALoop = true;
      }
        
      else if(ch != '.')
        cmd = cmd + ch;
      else{
      isCommandAvailable = true;              
      }
      
      
    }
}


//
///* Event based serial routine for reading serial data*/
//// The command format is <command>. 
//void serialEvent(){
//  char ch;
//  if(!isCommandAvailable){
//    ch = Serial.read();
//    if(ch!= '*'){
//      // syncing the timestamp
//      if(ch == '%'){
//             delay(100);
//             Serial.print('%');
//      }
//
//      if(ch != '.')
//        cmd = cmd + ch;
//      else{
//      isCommandAvailable = true;              
//      }
//      
//      
//    }
//    else{      
//      // init time directly
//      /*
//      String syncTS = "";      
//      do{
//        ch = Serial.read();
//        syncTS += ch;
//      }while(ch!= '.')
//      
//      */   
//      initTime = millis();
//      
//    }
//
//  }
//}

/* syncs the timestamp with the provided timestamp by converting the provided timestamp into _initTime 
void syncTimestamp(String ts){
  
    int hh=0,mm=0,ss=0,ms=0;
    hh = ts.substring(0)
}

*/

//* This function takes the 'Command' as Arguement and then Parses the valuses out of it.          *//
// it also performs necessary actions on the parsed data..
// it recognizes commands of format <two character command><argument1>,<arguement2>... <period i.e. a "dot">  .. e.g. $ms1a.
void parseCommand(String c){
  
  String sensorData="";

  // Stop the chair in free mode
    if(c[0] == 'b'){        
          int type = (c.substring(1,c.length())).toInt();
          wheelchair.stopWheelchair(stopType(type));
          isControlRequired = false;
          blinkLED();
    } 
  
  // turn at spot to some angle
    else if(c[0] == 't'){        
      
       // here in the turn we dont change the mode yet as we have not yet determined the direction to choose so as to move as little as possible, 
       // hence mode switching happens in the control in this case//
    
         wheelchair.setWheelchairMode(-1);     
         desiredHeading = (c.substring(1,c.length())).toInt();
         desiredHeading = (desiredHeading>359)?359:desiredHeading;
         wheelchair.setPWMParameters(0,30);
         isControlRequired = true;     
         blinkLED();
    }
    
  // move the wheelchair forward with the passed heading value
    else if(c[0] == 'f'){
         wheelchair.setWheelchairMode(0);    
         wheelchair.setPWMParameters(30,30); 
         desiredHeading = (c.substring(1,c.length())).toInt();
         desiredHeading = (desiredHeading>359)?359:((desiredHeading<0)?0:desiredHeading);     // rounding the desired theta to 359 deg
         isControlRequired = true;
         blinkLED();     
    }
  // speed up ONLY IN FORWARD  
    else if(c[0] == 's' && c[1] == 'u'){       
         if(wheelchair.getWheelchairMode() ==0){    
           wheelchair.setPWMParameters(50,30);
         } 
    }
    
  // speed slow  ONLY IN FORWARD
    else if(c[0] == 's' && c[1] == 'd'){       
         if(wheelchair.getWheelchairMode() ==0){    
           wheelchair.setPWMParameters(30,30);
         } 
    }
  
  // for Setting gains format: g<kp>,<num/den>.  eg. g10,1/10
    else if(c[0] == 'g'){            
       
       // set gains P and I parameters
         int Kp,Ki;
         int num,den;     
         Kp = (c.substring(1,c.indexOf(','))).toInt();
         num =(c.substring(c.indexOf(',')+1, c.indexOf('/'))).toInt();
         den = (c.substring(c.indexOf('/')+1, c.length())).toInt();
         Ki = float(num)/float(den);
         Ki = (Ki<0)?0:Ki;
         Kp = (Kp<0)?0:Kp;     
         wheelchair.setGains(Kp, Ki);
    
    }

  // for mask (both bytes) eg. ma<mask1><mask2>.  
    else if(c[0] == 'm' && c[1] == 'a'){
         // sensors.setMasks(c[2], c[3]);
          sensors.setMasks(char(255), char(255));
    }
    
  // for specific mask defined by the number 1 or 2          eg.  ms<mask_num><mask>.
    else if(c[0] == 'm' && c[1] == 's'){
      
          if((c[2]-48) == 1)
              sensors.setSpecificMask(MASK1, c[3]);
              
          else if((c[2]-48) == 2)
              sensors.setSpecificMask(MASK2, c[3]);    
          
                        
    }
  
  // set the control frequency
    else if(c[0] == 'c' && c[1] == 'f'){
             controlFrequency = (c.substring(2,c.indexOf(','))).toInt();
             controlFrequency = (controlFrequency <0)?0:controlFrequency;     
             Serial.println(controlFrequency);
    }
  
  // set/reset heading control (PI COntrol) FLAG
    else if(c[0] == 'h' && c[1] == 'c'){
  
        int flag = (c.substring(2,c.indexOf(','))).toInt();
        if(flag == 1){
              wheelchair.setPIControlFlag(1);
        }
        else if(flag == 0){
              wheelchair.setPIControlFlag(0);
        }
    }
  
  // GPIO manipulation - write in Individual mode i.e. write on one pin e.g. dwi3,1.
    else if(c[0] == 'd' && c[1] == 'w' && c[2] == 'i'){
        Serial.println("ok");
        int pinNo = (c.substring(3,c.indexOf(','))).toInt();
        int value = (c.substring(c.indexOf(',')+1, c.length())).toInt();
        Serial.println(pinNo);
        Serial.println(value);
        
        gpio.writeDigital(pinNo, value);        
    }
    
  // GPIO manipulation - range mode i.e. write to manipulate a range of PINS e.g. dwr0-10,1
    else if(c[0] == 'd' && c[1] == 'w' && c[2] == 'r'){
  
        int from = (c.substring(3,c.indexOf('-'))).toInt();
        int to = (c.substring(c.indexOf('-')+1, c.indexOf(','))).toInt();
        int value = (c.substring(c.indexOf(',')+1, c.length())).toInt(); 
        gpio.writeDigital(from, to, value);            
    }

  // GPIO manipulation - write PWM in Individual mode i.e. write on one pin
    else if(c[0] == 'd' && c[1] == 'p' && c[2] == 'i'){
        
        int pinNo = (c.substring(3,c.indexOf(','))).toInt();
        int value = (c.substring(c.indexOf(',')+1, c.length())).toInt();
        gpio.writePWM(pinNo, value);        
    }
  
  // GPIO manipulation - range mode i.e. write PWM to manipulate a range of PINS e.g. dwr0-10,1
    else if(c[0] == 'd' && c[1] == 'p' && c[2] == 'r'){
  
        int from = (c.substring(3,c.indexOf('-'))).toInt();
        int to = (c.substring(c.indexOf('-')+1, c.indexOf(','))).toInt();
        int value = (c.substring(c.indexOf(',')+1, c.length())).toInt(); 
        gpio.writePWM(from, to, value);            
    }


  // GPIO manipulation - set mode - individual mode
    else if(c[0] == 'd' && c[1] == 's' && c[2] == 'i'){
  
        int pinNo = (c.substring(3,c.indexOf(','))).toInt();
        int value = (c.substring(c.indexOf(',')+1, c.length())).toInt();   // 0 - 3
        gpio.setType(pinNo,Type( value));            
    }

  // GPIO manipulation - set mode - range mode
    else if(c[0] == 'd' && c[1] == 's' && c[1] == 'r'){
  
        int from = (c.substring(3,c.indexOf('-'))).toInt();
        int to = (c.substring(c.indexOf('-')+1, c.indexOf(','))).toInt();
        int value = (c.substring(c.indexOf(',')+1, c.length())).toInt(); 
        gpio.setType(from, to, Type(value));                  
    }
  
  // reverse - no control
    else if(c[0] == 'r'){
      
         wheelchair.setWheelchairMode(-1);
         wheelchair.setWheelchairMode(1);    
         wheelchair.setPWMValues(20,20);     
         blinkLED();           
    }
    
  // put the pwm to the motors directly  //comamand frmat : p123,345.
    else if(c[0] == 'p'){                                                              
      
         byte lpwm, rpwm;
         lpwm = byte (c.substring(1,c.indexOf(',')).toInt());
         rpwm = byte(c.substring(c.indexOf(',')+1, c.length()).toInt());
         
         lpwm = (lpwm>255)?255:((lpwm<0)?0:lpwm);
         rpwm = (rpwm>255)?255:((rpwm<0)?0:rpwm);
              
         wheelchair.setPWMValues(c.substring(1,c.indexOf(',')).toInt(),c.substring(c.indexOf(',')+1, c.length()).toInt());
         isControlRequired = false;
         blinkLED();
    }   

    // command to activate or suppress wheelchair interface. Note that by default the wheelchair interface is active just that we suppress it by this command. 
    // and when ever required get back the control.  e.g. wis == suppress it , wia == activate it - default;   
    else if(c[0] == 'w' && c[1] == 'i'){
          if(c[2] == 's' && !suppressWheelchair)            
            suppressWheelchair = true;
          else if(c[2] == 'a' && suppressWheelchair)            
            suppressWheelchair = false;          
    }

  cmd = "";              //once command parsed make it NULL
  isCommandAvailable= false;   //

}


/* -------------------------------------END OF THE UTILITY FUNCTIONS ----------------------------------------*/


void setup(){
  
  Serial.begin(9600);
  
  Wire.begin(); 
  pinMode(7,OUTPUT);                // this is used to set the LED pin output-- JUST FOR TESTING
  controlFrequency = 200;          // 200 ms default
  
  sensors.initSonars(); 
  sensors.setSonarParameters(100,0.0172); 
  sensors.setCompassTimeout(11); 
  

  
  prevTime = millis();  
  
 
  /*
    // DEBUG CODE......SHOULD BE REMOVED IN THE ACTUAL CODE
   
     String s;
     s= "ms2,";
     s = s+char(4);
     s= s+'.';
     parseCommand(s);
     sensors.setSpecificMask(MASK2, char(4));//B11011111
   
   */
   
   /* Wait for start SenseActLoop character '@' */
   
   
   // set the current millis as the initTime i.e. make the timestamp as 0:0:0:0
   
 }


/**********************************************************************************
 *                   SENSE - COMMUNICATE - LOOP                                   *
 **********************************************************************************/




void loop(){
   
    if(startSCALoop){
        
        
       /* Sensing phase  - it communicates also */
        
         sensors.readSonars();    
         sensors.readCompass();
         sensors.readCharge();
        
         sensorReadings = sensors.getSensorData();
        
        /* Communicate Loop -- 
         * Parse command and communicate the arguements to relevant interfaces viz RRIWheelchair, GPIO, Sensor, PID etc.       
         * Note: Sensor data is communicated in right where it is sensed
         */
        
        if(isCommandAvailable){    
          parseCommand(cmd);
        }
        
        // if wheelchair interface required only then do this else no...By default this works.
        if(!suppressWheelchair){
            /* Control Phase */
            
            controlTime = millis() - prevTime;                        // read time elapsed to check whether control should be done or not
            if(controlTime > controlFrequency && isControlRequired){
              
                  wheelchair.control(0.200, sensorReadings[8], desiredHeading);
                  prevTime = millis();
            }
            
            
            /* Actuation Phase */
            
            wheelchair.actuate();
            delay(100);
        }
    
    }// end of 
}


/*
          COMMANDS AND THEIR MEANING
       ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^   

all commands used in the current scenario

f<angle>.             -  forward in some angle
t<anlge>.             -  turn at spot to some angle
cf<freq>.             -  control frequency in ms
p<L>,<R>.             -  direct PWM control L-left motor PWM value.. simillarly for R
b.                    -  stop freely
g<Kp>,<Ki>/<divider>. -  set gains. divider is used so that one can put floating point for Ki
ma<m1><m2>.           -  sensor mask all masks. note that there is no comma (,)
sm<number><m>.        -  specific mask. first arguement says which mask. note that there is no comma (,)
su.                   -  speed up only when in forward mode
sd.                   -  come back to normal only in forward mode
*/

