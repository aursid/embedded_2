#include "mbed.h"
#include "rtos.h"
#include "slre.h"
//#include "string"

 //Photointerrupter input pins
#define I1pin D2
#define I2pin D11
#define I3pin D12

//Incremental encoder input pins
#define CHA   D7
#define CHB   D8

//Motofr Drive output pins   //Mask in output byte
#define L1Lpin D4           //0x01
#define L1Hpin D5           //0x02
#define L2Lpin D3           //0x04
#define L2Hpin D6           //0x08
#define L3Lpin D9           //0x10
#define L3Hpin D10          //0x20

//Mapping from sequential drive states to motor phase outputs
/*
State   L1  L2  L3
0       H   -   L
1       -   H   L
2       L   H   -
3       L   -   H
4       -   L   H
5       H   L   -
6       -   -   -
7       -   -   -
*/
//Drive state to output table
const int8_t driveTable[] = {0x12,0x18,0x09,0x21,0x24,0x06,0x00,0x00};

//Mapping from interrupter inputs to sequential rotor states. 0x00 and 0x07 are not valid
const int8_t stateMap[] = {0x07,0x05,0x03,0x04,0x01,0x00,0x02,0x07};
//const int8_t stateMap[] = {0x07,0x01,0x03,0x02,0x05,0x00,0x04,0x07}; //Alternative if phase order of input or drive is reversed

//Phase lead to make motor spin
int8_t lead = -2;  //2 for forwards, -2 for backwards

//Status LED
DigitalOut led1(LED1);

//Photointerrupter inputs
DigitalIn I1(I1pin);
DigitalIn I2(I2pin);
DigitalIn I3(I3pin);

//Quadrature Outputs
DigitalIn Q1(CHA);
DigitalIn Q2(CHB);

//Motor Drive outputs
PwmOut L1L(L1Lpin);
DigitalOut L1H(L1Hpin);
PwmOut L2L(L2Lpin);
DigitalOut L2H(L2Hpin);
PwmOut L3L(L3Lpin);
DigitalOut L3H(L3Hpin);

Ticker music;
float notes[4] = {0.00023889, 0.00025309, 0.00026814, 0.00028409}; 
Serial pc(SERIAL_TX, SERIAL_RX);
bool spin = false;

//------------------------------------------------------------------------//

//Thread definitions
Thread thread1;

//Interrupt definitions
InterruptIn interruptI1(I1pin);

int rev_counter = 0; 
double set_velocity = 20;
double velocity_current = 0;
double velocity_old = 0;
int revsToTarget = 0; //distance to target

Timer timer;
double time_current = 0;
int set_revs = 100;

void music_fn(){
    for(int i = 0; i < 12; i++){
        L1L.period(notes[i]);
        L2L.period(notes[i]);
        L3L.period(notes[i]);
    }
}   

void findVel(){
    float time = timer.read();
    velocity_old = velocity_current;
    velocity_current = 1/(time - time_current);
    time_current = time;
}

void incrementRev_findVel(){
    findVel();
    rev_counter++;
}             
    
double implementControlVel(){
 
    double k = 5;   
    double kp = 2;     
    double v = set_velocity - velocity_current;   
    return k*kp*v; 
        
}

double implementControlRev(){
   
    double k = 33.5; 
    double kp = 2.1;
    double kd = -5;
    double ki = 0.14;
   
    revsToTarget = set_revs - rev_counter;
    return k*(kp*revsToTarget + kd*velocity_current + ki);
     
}

struct cap captures[3];
void command(){
    char receivedChars[10];
    int n = pc.scanf("%s", receivedChars); 
    struct slre slre1, slre2;
    slre_compile(&slre1, "R-(\d*)"); 
    slre_compile(&slre2, "R(\d*)"); 
    if(slre_match(&slre1, receivedChars, 3, captures)){
        spin = true; 
        lead = -2;
    }
    else if(slre_match(&slre2, receivedChars, 3, captures)){
        spin = true; 
        lead = 2; 
    }
}
 
//Set a given drive state
void motorOut(int8_t driveState) {
    
    //Lookup the output byte from the drive state.
    int8_t driveOut = driveTable[driveState & 0x07];
    
    double setPwm;
       
    double pwm_r = implementControlRev();
    double pwm_v = implementControlVel();
    
    if (pwm_r < pwm_v)
        setPwm = pwm_r;
    else
        setPwm = pwm_v;

    //Turn off first
    if (~driveOut & 0x01) L1L = 0;
    if (~driveOut & 0x02) L1H = 1;
    if (~driveOut & 0x04) L2L = 0;
    if (~driveOut & 0x08) L2H = 1;
    if (~driveOut & 0x10) L3L = 0;
    if (~driveOut & 0x20) L3H = 1;

    //Then turn on
    
    if (driveOut & 0x01) L1L = setPwm;
    if (driveOut & 0x02) L1H = 0;
    if (driveOut & 0x04) L2L = setPwm;
    if (driveOut & 0x08) L2H = 0;
    if (driveOut & 0x10) L3L = setPwm;
    if (driveOut & 0x20) L3H = 0;
}

//Convert photointerrupter inputs to a rotor state
inline int8_t readRotorState() {
    return stateMap[I1 + 2*I2 + 4*I3];
}

//Basic synchronisation routine
int8_t motorHome() {
    //Put the motor in drive state 0 and wait for it to stabilise
    motorOut(0);
    wait(2.0);
    rev_counter = 0;

    //Get the rotor state
    return readRotorState();
}

void runMotor() {
    int8_t orState = 0;    //Rotot offset at motor state 0

    //Initialise the serial port (and the timer)
    RawSerial pc(SERIAL_TX, SERIAL_RX);
    int8_t intState = 0;
    int8_t intStateOld = 0;

    //Run the motor synchronisation
    orState = motorHome();
    pc.printf("Rotor origin: %x\n\r",orState);
    //orState is subtracted from future rotor state inputs to align rotor and motor states

    //Poll the rotor state and set the motor outputs accordingly to spin the motor
    while (rev_counter <= set_revs) {
        intState = readRotorState();
        if (intState != intStateOld) {
            intStateOld = intState;
            motorOut((intState-orState+lead+6)%6); //+6 to make sure the remainder is positive
        }
    }
}    

void spinForNRotations(){
    
    spin = false; 
    set_revs = atoi(captures[1].ptr); 
    timer.start();
    thread1.start(runMotor);
    interruptI1.rise(&incrementRev_findVel);
    int old_rev_counter = -1;
    while(true) {
        if(old_rev_counter != rev_counter){
            old_rev_counter = rev_counter;
            printf("rev_counter: %d\n", rev_counter);
        }
    }
    
}   

//Main--------------------------------------------------------------------------
int main() {
    
    pc.baud(9600);
    while(true){
        command();
        if(spin == true){
            spinForNRotations();
        }
    }

}
