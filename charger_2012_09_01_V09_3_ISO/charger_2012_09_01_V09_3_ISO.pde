 /*
EMW SmartCharger-12000
A 12kW+ charging system

DIY charger inspired by the work of SimonRafferty & jackbauer on DIYelectriccar.com:
http://www.diyelectriccar.com/forums/showthread.php/200-build-your-own-intelligent-charger-36627.html. 

DETAILED FORUM DISCUSSION OF THIS DESIGN IS AT 
http://www.diyelectriccar.com/forums/showthread.php/10kw-60a-diy-charger-open-source-59210p39.html

Controller: Arduino Pro Mini 5V (based on a ATmega328P-PU microcontroller)

Pinout assignments: see below in code

----------- Basic code structure:
------ Startup:
* 2 timeouts - one 5 sec for config, one 10 sec for power setting. can be interrupled by any button
* check mains voltage. If 110, limit power to ~1.5kW
* set duty cycle to 0
------ Charging (CV or CC):
* increase duty cycle until the condition is met (target voltage or target current)
* monitor condition by taking frequent samples averaging over 120Hz ripple waveform
* based on average value of condition, change duty cycle accordingly (slow down update frequency
  as we get closer to the target voltage)
* break when exit condition satisfied

Created Jan 2011 by Valery Miftakhov, Electric Motor Werks, LLC & Inc. Copyright 2011-2013
Commercial use prohibited without written approval from the Author or EMW.
*/

// need this to remap PWM frequency
#include <EEPROM.h>
#include "EEPROM_VMcharger.h"
#include <TimerOne.h>
// my LCD library for 4D systems display (http://www.4dsystems.com.au/prod.php?id=121)
#include <uLCD_144.h>
uLCD_144 *myLCD;

//----------- DEBUG switch - careful, this disables many safety features...---------------
//#define DEBUG0 // just increase maximums
// #define DEBUG // additional printouts / delays / etc.
//----------------------------------------------------------------------------------------


//------------------------------ MAIN SWITCHES -----------------------------------
#define drop110power // reduce power to ~1.5kW when connected to 110VAC?
// #define hall150U // 150A output current unidirectional sensor?
#define hall100U // 100A output current unidirectional sensor? (default for kits / chargers shipped from 12/12)
// #define MCC100A // 100A output rating - use ONLY with a custom-wound inductor
//#define A7520_V // using A7520 optoisolation for outV sensing? (as opposed to ISO124)
#define PFC // is this a PFC unit?
// #define UV12 // enable gate supply undervoltage protection?
// #define NiXX // do we want support for Nickel chemistries?
//------------------------------- END MAIN SWITCHES ------------------------------


//---------------- pin-out constants ----------------
//========== analog pins
const int pin_C=0; // output current pin
const int pin_bV=1; // output / battery voltage pin
const int pin_heatSinkT=2; // charger heatsink temp - for thermal derating 
const int pin_12Vsense=3; // implementing undervoltage protection
// 4 - spare prewired as temp input
const int pin_mV=5;
//========== digital pins
// 0/1 reserved for serial comms with display etc
const int pin_pwrCtrlButton=2; // this is wired to the button (used for menu step)
const int pin_pwrCtrl2Button=3; // this is wired to the button2 (used for menu select)
const int pin_inrelay=4; // precharges input caps
const int pin_outrelay=5; // protects from reverse polarity on traction battery, precharges output resistors
const int pin_PWMpulldown=6; 
const int pin_J1772=7; // J1772 pilot input. 1.3k is hardwired on V14+ pcbs so J1772 will power on on connect
const int pin_fan=8; // fan control - this is pin4 in all kits shipped before Mar '2912
const int pin_PWM=9; // main PWM pin

// max current reference voltage (using PWM) -  was 6 in the V13 pcb (kits shipped before March 2012)
// now moved to pin 10 so that we can use higher PWM frequency 20kHz PWM
const int pin_maxC=10; 

// 110/220vac relay control - for non-PFC units only
// If PFC is connected, relay would never close so can be removed
// also can be left unused / unconnected if separate 110V / 220V inputs are used 
const int pin_110relay=11; // in kits shipped before Mar 2012, this is pin 5

const int pin_EOC=12; // end-of-charge output (see pinout diagram) - pulled low when charge is complete

// end-of-charge input from BMS. Pull low / disconnect from positive TTL signal to activate
//     (normallly will be realized via connecting NC BMS loop between this pin and EOC pin (or +5V)
const int pin_BMS=13; 
//---------------- END PINOUTS -----------------------


//============= BATTERY INFO  =====
struct config_t {
  int battType;
  int nCells;
  int AH;
  int CV; // per cell
  int CC; // max output current
  int mainsC; // max input current
  // sensor config
  float Vcal;
  float Vcal_k;
  float mVcal;
  float Ccal;
} configuration;

// battery type 
// 0 = LiFePo4, 1 = NiMh/NiCad, 2 = Lead Acid (in dev), 3 = LiCo (in dev)
const char * battTypeLabel[] = { "    LiFePo4   ", "  NiMh/NiCad  " };
int battTypeLen = 2;
//============= END BATTERY INFO USER EDITABLE PARAMETERS =====================================================


//------------------------------- default constants for various battery chemistries
// min battery voltage - if below this, do not start the charger
const float minBattVs[4]={2.5, 0.9, 10., 3.0};

// CV constant (N/A for LA, Ni)
const float CVs[4]={3.5, -1, -1, 4.2}; // charging voltage for CALB 3.6V per cell. Using 3.55 here to ensure reliable detecion of end-of-charge for a bottom-balanced pack

// absolute maximum voltage
const float maxBattVs[4]={3.8, 1.6, 15., 4.5};

// Nickel chemistries dVdt cutoff
const float dVdt_stop=0.; // in %/s. at 1C, safe value is between -1E-05 and +1E-05

// DO NOT CHANGE THESE!
float CV;
int minMains=30; // min mains voltage to (1) test sensor connectivity and (2) detect mains disconnect 
const float min_CV_Crating=0.05; // wait until the current goes to XC (use values from your battery's datasheet)
// spread for duty cycle ramp conditions to avoid jitter - in volts
// With 10k resistor, 40ma sensor, 51V/A constant, voltage sensitivity is ~1V so no point setting this lower
const float spreadV=2.;
// With 50A sensor, 0.06V/A constant, current sensitivity is ~0.1A. But current being off is not a big deal...
const float spreadC=1.; 
float maxOutV=0; // absolute maximum output voltage - will be set later in the code
#ifdef PFC
  const float charger_efficiency=0.92; 
#else
  const float charger_efficiency=0.95; 
#endif
// ------------------------------- END battery constants -----------------------------


//---------------- MAX CURRENTS
// absolute maximum average output current (used in CV mode) - leave at 0 here - will be set via power button
float maxOutC=0.; 
#ifdef MCC100A
  float absMaxChargerCurrent=100; // 100A rating with high-current output toroid inductor
#else
  float absMaxChargerCurrent=70; // 70A default rating with new toroid inductors
#endif
float absMaxChargerPower=12000; // 12kW rating with new toroid inductors
// when does the current limiter kick in? 1.2-1.3 is a good compromise to get the most out of 
// the input caps while keeping overall ripple below 50% 
// this is mostly relevant for 120Hz ripple. Switching frequency ripple is controlled by the automatic
// frequency selection and low-ESR high-freq output cap
// if using smaller inductors (e.g., <200 uH), may have to use higher ratio here (e.g., 1.4-1.5)
const float instantMaxCRatio=1.4; 
int timeOut=0; // in min, 0 means no timeout

//------------- THERMAL DERATING OF CHARGER 
// for now, simple protection by pausing charger until cooldown to certain temp
// note that heatSink temp at the point of measurement is generally 20-30 deg C LOWER than temperature 
// of critical components attached to heatsink (due to distance from components to probe)
// use maxHeatSinkT of <60 to ensure <85 deg C temp of components
// this assumes thermistor placement near the heat generating components
// BTW, modest airflow (120mm PC fan) with a large (8x10x3" heatsink should be sufficient for 
// up to 30A output at max power 
#ifndef MCC100A
  const int maxHeatSinkT=55; // in Centigrades - will start derating here
#else
  const int maxHeatSinkT=47; // more aggressive derating at high current output
#endif
const int ABSmaxHeatSinkT=85; // in Centigrades - will stop the charger altogether here
int midHeatSinkT=45; // turn on the fans here; also wait until cool down to this temp before resuming at the prev power level 
int lowHeatSinkT=35; // turn off the fans here 
const int MAXFANDUTY=160; // with 15.5V supply, do not send more than 10V to the fan = 60-65% duty
int fanduty=0;
//--------------------------------------------------------

// SAFETY - mostly for testing. 
const int PWM_res=1024;
const int MAXDUTY=PWM_res*0.97; // very short off pulses are bad (diode does not recover by the time IGBT turns on again - generally limit Toff to MORE than 1us)
int period=70; // set PWM period in microseconds (max without driver cooling for IGBT half-bridge seems to be 20kHz)

// sensor supply
const float Vcc=5.0; 
float V_o_V0=0.; // 0.0 for voltage transducers (ISO124)
float V_o_C0=Vcc/2;
const float Aref=Vcc; 

//=============== voltage dividers settings ===========================
const float ISO124_inputR=200.; // account for 200k input impedance of the ISO124 opamp
const float lowerR0_mV=27.; // 27k
const float upperR0_mV=2000.; // 2M
float divider_k_mV=-1.; 
// Vout @ 0 AT
float V_o_mV=V_o_V0; 

const float upperR0_bV=2000.; // 2M
float divider_k_bV=-1.;
// Vout @ 0 AT
#ifdef A7520_V
  const float gain=Vcc/0.512; // per datasheet, assuming Vref=Vcc
  // resistor from -5V regulator; should form a ~-.25V divider together with the 
  // bottom resistor => >20x * bottom resistor 
  // for 2.7k bottom resistor, pick between 60k and 82k; 68k is a good choice... 
  float lowerR_neg5_bV=68.; // in kOhms
  float V_o_bV0=Vcc/2-Vcc*2.7/lowerR_neg5_bV*gain; // -5V input, 2.7k bottom resistor, ~10x gain; // ~2.5V for A7520
  const float lowerR0_bV=2.7*gain; // +-0.256V range for input, ~10x gain, 2.7k bottom resistor
#else
  float V_o_bV0=V_o_V0;
  const float lowerR0_bV=27.; // 27k
#endif
float V_o_bV=V_o_bV0;
//================ end voltage dividers setup =========================

//========== charger current sensor ==============================
// V/A constant for the charger current sensor 
#ifdef hall150U
  const float k_V_C=0.0267; 
#else
  #ifdef hall100U
    const float k_V_C=0.04; 
  #else
    // here, only 50A
    const float k_V_C=0.06; 
  #endif
#endif
// Vout @ 0 AT
// 0.6V for Allegro X050U, X100U and 150U
float V_o_C=0.6; 

//=================== TEMP sensor block================
const float T0=298; // standard reference temp for NTC thermistors = 25 centigrade
//=========== charger heatsink temp sensor ========================
const float B_hST=4540; // this is the B value
const float R0_hST=100000; // nominal (at T=25) resistance of the thermistor
const float R1_hST=100000; // voltage divider resistor
const float V_hST=5; // voltage applied to the divider
//====================== END SENSOR CONSTANTS block ==============

//===================== charger cycle timers =====================================
#define SLOWUPDATE // slow down update upon reaching the target condition
// for stepDelay=1000, use measCycle_len=300, dVdt_measCycles=150
// when changing stepDelay, change the other 2 variables so that stepDelay*measCycle_len = 0.5-1 sec
// and stepDelay*measCycle_len*dVdt_measCycles = 100-200 sec
const int stepDelay0=4; // primary charger loop delay in millioseconds
const int measCycle_len0=100; // how many primary loop cycles per display cycle
const int nSamplesStopVar0=10; // how many samples for moving averages of output voltage / current
int stepDelay; // this will be changed in the loop
int measCycle_len; // this will be changed in the loop
int nSamplesStopVar;
const int stopCycles=5; // how many primary charger cycles to require stop condition to exist before exiting
const int nReadSamples0=2; // how many samples to average in a single call of readX() functions
int nReadSamples;
const int waitReadSamples=200; // wait between samples for voltage / current readouts in microseconds
//===================== end charger cycle timers =================================

//=========== these should be global vars
int BMSon=1; // this will be changed in code
float duty=0;
float mainsV=0, outV=0, outC=0;
int charger_run=0;
unsigned long timer=0, timer_ch=0;
float AH_in=0, AH_charger=0;
unsigned int min_up=0;
char str[100];
byte state;
float temp;

//------------------------- Running Averages for dV/dt calcs -------------
float V_ravg[2];
unsigned long t_ms = 0;
unsigned long ele_counter=0;
float dVdt = 0.0;
//-----------------------Navigate Menu--------------------
const char * configMenu[] = { " Run Charger  ", " Config Power ", " Config Time  "  };
const unsigned int configMenuLen = 3;
const char * menuNavigate[] = { "     Yes      ", "      No      " };
const unsigned int menuNavigateLen = 2;
// ------------- end global vars ---------------------------

void setup() {
  #ifdef DEBUG0
    absMaxChargerCurrent=140; // 140A for testing - limited by the sensor saturation 
    absMaxChargerPower=20000; // 20kW for testing
  #endif

  // set analog input pins
  pinMode(pin_C, INPUT);
  pinMode(pin_bV, INPUT);
  pinMode(pin_heatSinkT, INPUT);
  pinMode(pin_12Vsense, INPUT);
  pinMode(pin_mV, INPUT);

  // digital inputs
  pinMode(pin_pwrCtrlButton, INPUT);
  pinMode(pin_pwrCtrl2Button, INPUT);
  pinMode(pin_J1772, INPUT);
  pinMode(pin_BMS, INPUT);

  // set output digital pins
  pinMode(pin_PWMpulldown, OUTPUT);
  pinMode(pin_PWM, OUTPUT);
  pinMode(pin_maxC, OUTPUT);
  pinMode(pin_EOC, OUTPUT);
  pinMode(pin_fan, OUTPUT);
  pinMode(pin_inrelay, OUTPUT);
  pinMode(pin_outrelay, OUTPUT);
  pinMode(pin_110relay, OUTPUT);
  
  // reset voltage dividers to account for the input resistance of ISO124
  float lowerR_mV=1/(1/lowerR0_mV+1/ISO124_inputR);
  divider_k_mV=upperR0_mV/lowerR_mV;
  float lowerR_bV=1/(1/lowerR0_bV+1/ISO124_inputR);
  divider_k_bV=upperR0_bV/lowerR_bV;

  // get the display going
  *myLCD=uLCD_144(9600); // max is 100kbps and is dependent on the noise levels in the charger
  myLCD->setBgColor(0, 0, 0);
  myLCD->setContrast(0x0f);
  myLCD->clrScreen();
  myLCD->setOpacity(1);
  
  nReadSamples=nReadSamples0; // need this here (otherwise all measurements in setup are screwed)

  //==================================== ONE-TIME CONFIG =================================
  // check if needed to go into config 
  int forceConfig=0;
  EEPROM_readAnything(0, configuration);
  if(configuration.CC<=0) {
    forceConfig=1; // first time running the charger after assembly
    configuration.Vcal_k=1.; // prefill the calibration with unity so we don't get zero readings if calibration menu is skipped
    configuration.CV=CVs[configuration.battType]*100;
  }
  
  int x = 0;
  const byte STATE_DONE = 0xff;
  const byte STATE_BT = 0x0;
  const byte STATE_CV = 0x1;
  const byte STATE_CELLS = 0x2;
  const byte STATE_CONFIRM = 0x3;
  const byte STATE_CAPACITY = 0x4;
  const byte STATE_CALIBRATE = 0x5; // sensitivity calibration only. zero point calibration done automatically on power-on
  state = STATE_BT;
    
  while(state != STATE_DONE)
  {
    switch(state)
   {
     case STATE_BT:
       myLCD->printStr(0, 0, 2, 0, 0x3f, 0x00, "Thank you for choosing EMW Charger! Press any button to configure"); 
       // if config is not forced, just timeout and send to end of config. Else, wait until button press
       if(forceConfig==0) {
         forceConfig=BtnTimeout(5, 5); // -1 if no button pressed; 1 otherwise
       }
       if(forceConfig==-1) {
         state=STATE_DONE;
       } else { // forceConfig=1 here
         myLCD->clrScreen();
         myLCD->printStr(0, 0, 2, 0x1f, 0x3f, 0x00, "Cell Type:          ");
         configuration.battType=MenuSelector2(battTypeLen, battTypeLabel);
         state = STATE_CV;
       }
       break;
     case STATE_CV:
       myLCD->printStr(0, 0, 2, 0x1f, 0x3f, 0x00, "CV cutoff:         ");
       configuration.CV = DecimalDigitInput3(configuration.CV); 
       state = STATE_CELLS;       
       break;
     case STATE_CELLS:
       myLCD->printStr(0, 0, 2, 0x1f, 0x3f, 0x00, "Number of cells:   ");
       configuration.nCells = DecimalDigitInput3(configuration.nCells); 
       state = STATE_CAPACITY;       
       break;
     case STATE_CAPACITY:
       myLCD->printStr(0, 0, 2, 0x1f, 0x3f, 0x00, "Capacity:          ");
       configuration.AH = DecimalDigitInput3(configuration.AH); 
       state = STATE_CALIBRATE;       
       break;
     case STATE_CALIBRATE:
       // first, zero calibration
       myLCD->printStr(0, 0, 2, 0x1f, 0x3f, 0x00, "Short output and press any button");
       while(!(digitalRead(pin_pwrCtrlButton) || digitalRead(pin_pwrCtrl2Button)));
       outV=readV();
       outC=readC();
       if(abs(outV)<40) { // if too far off, fault out
         // output voltage calibration
         temp=outV/divider_k_bV;
         V_o_bV+=temp; // this needs to be adjusted HERE because we are calling readV() again below for sensitivity calibration
         configuration.Vcal=temp; 
         // output current calibration
         configuration.Ccal=outC*k_V_C;
         myLCD->printStr(0, 5, 2, 0x1f, 0x3f, 0x00, "Calibrated zero");
         delay(1000);
       }
       // now at voltage. first, double-check we have reset to zero point
       outV=readV(); // get the readings with zero-point already calibrated
       if(abs(outV)<3) { // should be pretty tight after zero calibration
         myLCD->clrScreen();
         myLCD->printStr(0, 0, 2, 0x1f, 0x3f, 0x00, "Connect battery to calibrate or press any button to skip");
         delay(1000); // to avoid reading same button state as in prev step
         while(1) {
           outV=readV();
           if(digitalRead(pin_pwrCtrlButton) || digitalRead(pin_pwrCtrl2Button))  break;
           if(outV>10) { // loop until battery not connected
             delay(5000); // let settle
             outV=readV(); // read settled voltage
             // calibrate
             myLCD->printStr(0, 0, 2, 0x1f, 0x3f, 0x00, "Measure & enter actual battery voltage:");
             // calibration routine here - if actual voltage > shown, REDUCE the constant
             configuration.Vcal_k=DecimalDigitInput3(int(outV))/outV;
             break; // from while() loop
           }
         }
       }
       // now mains? problem is - PFC is 380VDC, rectified/doubled in older design are 330V...
       // skip for now - mains sensing is good enough without calibration
       configuration.mVcal=0;
       state = STATE_CONFIRM;
       break;
     case STATE_CONFIRM:
       myLCD->clrScreen();
       myLCD->printStr(0, 0, 2, 0x1f, 0x3f, 0x00, "Confirm:        ");
       sprintf(str, "%d %s cells, %dAH", configuration.nCells, battTypeLabel[configuration.battType], configuration.AH);       myLCD->printStr(0, 1, 2, 0x1f, 0x3f, 0x00, str);
       x=MenuSelector2(menuNavigateLen, menuNavigate);
       if(x == 0) state = STATE_DONE;
       if(x == 1) state = STATE_BT;
       break;
     default: break;
   } 
  }

  // parameters calculated from config variables go here
  maxOutV=maxBattVs[configuration.battType]*configuration.nCells;
  // adjust core sensor constants
  V_o_bV=V_o_bV0+configuration.Vcal;
  V_o_mV+=configuration.mVcal;
  V_o_C+=configuration.Ccal;
  divider_k_bV*=configuration.Vcal_k; 
}
  

void loop() {  
  // ---------------real loop()
  float pwr;
  int J1772_dur;
  mainsV=read_mV();
  outV=readV();

  // check for battery connection
  int x=1;
  if(outV<minBattVs[configuration.battType]*configuration.nCells) {
    // either no battery or reverse polarity
    printClrMsg("Battery not connected or reverse polarity. Press any button to ignore (CAREFUL!)", 50, 0x1f, 0x3f, 0);
    x=BtnTimeout(10, 7);
  }

  // protect buck's freewheeling diode from high current at low duty - limit to average of 80A through diode
  absMaxChargerCurrent=min(absMaxChargerCurrent, 80./(1-outV/400));
  
  delay(1000);

  // run charger if: 
  //         (1) charger has NOT been run yet in this cycle, or 
  //         (2) has been run over a week ago
  //         (3) positive battery voltage has been detected or zero/negative been commanded to ignore
  //         (4) green button is pressed to override
    if(x==1 && (isBtnPressed(pin_pwrCtrl2Button) || charger_run==0 || (charger_run==1 && millis()-timer_ch>1000*3600*24*7))) {
      // get the charger going
      x=0;
      
      //----------------------------
      // run state machine:
      const byte STATE_TOP_MENU = 0x0;
      const byte STATE_CONFIG_PWR = 0x1;
      const byte STATE_CONFIG_TIMER = 0x2;
      const byte STATE_RUN_CHARGER = 0x3;
      const byte STATE_CHARGE = 0x4;
      const byte STATE_WAIT_TIMEOUT = 0x5;
      const byte STATE_SHUTDOWN = 0xff;
      state = STATE_WAIT_TIMEOUT;
      if(configuration.CC<=0) state=STATE_CONFIG_PWR;
      
      while(state != STATE_SHUTDOWN)
      {
        myLCD->clrScreen();
        myLCD->printStr(0, 6, 2, 0x1f, 0x3f, 0, "Params      ");
        sprintf(str, "IN: %dV, %dA", int(mainsV), configuration.mainsC); myLCD->printStr(1, 7, 2, 0x1f, 0x3f, 0, str);
        sprintf(str, "OUT: %dA", configuration.CC); myLCD->printStr(1, 8, 2, 0x1f, 0x3f, 0, str);
        sprintf(str, "TIMEOUT: %dmin", timeOut); myLCD->printStr(0, 9, 2, 0x1f, 0x3f, 0, str); 
        
        switch(state)
        {
        case STATE_WAIT_TIMEOUT:
          myLCD->printStr(0, 0, 2, 0x1f, 0x3f, 0x00, "press BTN to change parameters");
          x=BtnTimeout(10, 3);
          
          // check J1772
          J1772_dur=pulseIn(pin_J1772, HIGH);
          if(J1772_dur>50) { // noise control. also, deals with the case when no J1772 signal present at all
            absMaxChargerPower=mainsV*6/100*J1772_dur; // J1772 spec - every 100uS = 6A input
          }
          
          if(x == 1) state = STATE_TOP_MENU;
          if(x == -1) // nothing pressed
           { 
            state = STATE_CHARGE;
           }
          break;
        case STATE_TOP_MENU:
          myLCD->printStr(0, 0, 2, 0x1f, 0x3f, 0x00, "Action:                         ");
          x=MenuSelector2(configMenuLen, configMenu);
          switch(x)
          {
            case 0: state = STATE_RUN_CHARGER; break;
            case 1: state = STATE_CONFIG_PWR; break;
            case 2: state = STATE_CONFIG_TIMER; break;
            default: break;
          }
          break;
        case STATE_CONFIG_PWR:
          myLCD->printStr(0, 0, 2, 0x1f, 0x3f, 0x00, "max INput current ");      
          configuration.mainsC = DecimalDigitInput3(configuration.mainsC); 
          myLCD->printStr(0, 0, 2, 0x1f, 0x3f, 0x00, "max OUTput current");      
          configuration.CC = DecimalDigitInput3(configuration.CC); 
          state = STATE_TOP_MENU;
          break;
        case STATE_CONFIG_TIMER:
            // now set the timer using the same button       
            myLCD->printStr(0, 0, 2, 0x1f, 0x3f, 0x1f, "timeout (min, 0 for no timeout):");
            timeOut=DecimalDigitInput3(0); 
           state = STATE_TOP_MENU;
           break;
        case STATE_RUN_CHARGER:
            myLCD->printStr(0, 0, 2, 0x1f, 0x3f, 0x00, "Confirm CHARGE [Y/n]:");
            x=MenuSelector2(menuNavigateLen, menuNavigate);
          if(x == 1) state = STATE_TOP_MENU;
          if(x == 0) 
          {
            state = STATE_CHARGE;
          }
           break;
         case STATE_CHARGE:
            pwr=configuration.mainsC; 
            // curb power on 110VAC
#ifdef drop110power            
            // 110VAC=160VDC rectified, 220VAC=320VDC - 240VDC is a midpoint in non-PFC
            // 165VDC is a midpoint in PFC version. Use below midpoint between 240 and 160 = 180
            if(mainsV<180 && mainsV>minMains) { 
              pwr/=2; // later, pwr is assumed to be a 220VAC-equivalent current
              pwr=min(pwr, 9.); // equivalent 15A from 110VAC // DEBUG

#ifndef PFC              
              // close 110VAC relay for doubler to operate
              digitalWrite(pin_110relay, HIGH); 
              delay(1000);
#endif
            }
#endif            
            maxOutC=min(pwr*charger_efficiency*230/maxOutV, absMaxChargerCurrent);
            maxOutC=min(maxOutC, absMaxChargerPower/maxOutV );
            // curb further if user-spec'ed current is less
            maxOutC=min(maxOutC, configuration.CC); 
                        
            // write out the configuration to EEPROM for next time
            EEPROM_writeAnything(0, configuration);
            
            CV=float(configuration.CV)/100; // 350 = 3.5V
  
            timer_ch=millis(); // reset the timer
            
            //========================== MAIN RUN CHARGER FUNCTION=======================
            // by this point, at least 15sec have passed since AC connection
            //                and at least 10sec since battery connection
            //                therefore, all caps should be pre-charged => close relays
            //   (note: this requires precharge resistors across relays - MAX of 300R for inrelay
            //          and MAX of 1k for outrelay. >3W power rating. Place small 1000V diode in
            //          series with the outrelay resistor - anode to battery - to avoid precharge on
            //          reverse polarity connection)     
            digitalWrite(pin_inrelay, HIGH);
            digitalWrite(pin_outrelay, HIGH);
            runCharger();
            digitalWrite(pin_inrelay, LOW);
            digitalWrite(pin_outrelay, LOW);
            //===========================================================================

            charger_run=1; // charger has run this mains cycle...
            state = STATE_SHUTDOWN; //STATE_TOP_MENU;   

           break; 
           default: break;
        }
      }
    }
  
}


//-------------------------------- main charger routine ------------------
int runCharger() {
  // initialize timer here - this way will reset every time when returning from no-mains break
  Timer1.initialize(period); 
  Timer1.pwm(pin_PWM, 0);           
  Timer1.pwm(pin_maxC, 0);           
 
  // reset AH counter
  AH_charger=0;
 
  // reset the EOC pin (this pin is active LOW so reset means setting to HIGH) 
  // high means charging is commencing. this will be pulled down by the charger when charge ends
  digitalWrite(pin_EOC, HIGH); 
  
  // see if BMS pin is connected to EOC pin through a closed-lood BMS system
  // BMSon=0; // comment this out if you want hard requirement for BMS in order to run charger
  // if the connection is there, then BMS pin will read HIGH. If not, it will read LOW as it's 
  // pulled down on the control board
  if(digitalRead(pin_BMS)==HIGH) { // assume that batteries are good in the beginning of charge
    BMSon=1;
  }
  
  //----------------- MAIN CHARGING ENTRY POINT -------------------------------------
  if(configuration.battType==0 || configuration.battType==3) {
    //---------------- CCCV for LiFePo4 or LiPoly --------------------
    // CC step, end condition - voltage goes to CV
    if(!runChargeStep(1, round(maxOutC), 1, configuration.nCells*CV)) {
      // CV step
      runChargeStep(2, configuration.nCells*CV, 2, configuration.AH*min_CV_Crating);
    }
  }
  
#ifdef NiXX  
  if(configuration.battType==1) {
    //---------------- CC with dVdt termination for NiMh --------------------
    // CC step, end condition - dVdt goes below pre-determined value
    runChargeStep(1, round(maxOutC), 3, dVdt_stop);
  }
#endif 

/*
  if(configuration.battType==2) {
    //---------------- Lead Acid - CV forever --------------------
    CV+=10.0; // convention for CV parameter storage in flash
    runChargeStep(2, configuration.nCells*CV, 2, configuration.AH*min_CV_Crating);
  }
*/
//------------------------------------------------------

  digitalWrite(pin_fan, LOW);    
  digitalWrite(pin_EOC, LOW); // active low
  myLCD->clrScreen();
  myLCD->printStr(0, 0, 2, 0x1f, 0x3f, 0x1f, "Charging Complete! Press right button to run again");      
  sprintf(str, "%dAH in", int(AH_charger)); 
  myLCD->printStr(0, 6, 2, 0x1f, 0x3f, 0x1f, str);      

  AH_in+=AH_charger;
}


//---------------------------- basic charging step function ----------------------------
// cycle type = 1 for CC, 2 for CV
// universal charging stage function - stop variable is different from start (i.e. if cycleType=1 (CC), CX is amps, 
// stop is voltage 
int out1Reached=0;

int runChargeStep(int cycleType, float CX, int stopType, float stopValue) {
  float outC_avg=0.;
  float outV_avg=0.;
  float outV0=0; // initial voltage at start of cycle
  float out1=0.;
  float out2=0.;
  float heatSinkT=0.;
  float spread=0;
  int breakCnt=0;
  int breakCycle=0;
  V_ravg[0] = 0;
  V_ravg[1] = 0;
  t_ms = 0; 
  duty=0; // start with the duty cycle = 0 until you hit the constraint
  
  myLCD->clrScreen();
  sprintf(str, "type=%d, CX=%d, sType=%d, max=%d", cycleType, int(CX), stopType, int(stopValue)); 
  myLCD->printStr(0, 4, 2, 0x1f, 0x3f, 0x1f, str);      
  delay(5000);
  myLCD->clrScreen();
  // start sensor readouts at some value so we can feed the averages
  outV_avg=outV=outV0=readV();
  outC_avg=outC=readC();
  setMaxC(maxOutC*instantMaxCRatio); // need this here before 5 sec delay so the RC net stabilizes
  // for derating
  float maxOutC1=maxOutC;
  
  // reset timer for AH metering
  timer=millis();
  int n=0, nn=0;
  
  // here, out1 is what is being controlled (kept constant), out2 is a termination criterion
  if(cycleType==1) {
    // CC - constant current, stop by max voltage
    spread=spreadC;
  } else if(cycleType==2) {
    // CV - constant voltage, stop by min current
    spread=spreadV;
  } else {
    // wrong charge profile
#ifdef DEBUG        
        printClrMsg("Wrong charge profile!", 5000, 0x1f, 0x3f, 0);
#endif
    return -1;
  }
  
  resetDelayParams();

  while(1) {
    // loop counter
    n++;
    min_up=(unsigned int)1.*(millis()-timer_ch)/60000;

    delay(stepDelay); // reasonable delay but not so that everything slows down too much
    
#ifdef UV12
    if(sampleRead(pin_12Vsense)<3.5) { // 12V supply dropped below 10.5V
      duty=0;
      digitalWrite(pin_PWMpulldown, HIGH);
    } else {
      digitalWrite(pin_PWMpulldown, LOW);
    } 
#endif

    Timer1.setPwmDuty(pin_PWM, duty);           
    
    // here, out1 is what is being controlled (kept constant), out2 is a termination criterion
    outC=readC(); // every cycle
    outC_avg=(outC_avg*(nSamplesStopVar-1)+outC)/nSamplesStopVar; // moving average
    outV=readV();
    outV_avg=(outV_avg*(nSamplesStopVar-1)+outV)/nSamplesStopVar; // moving average
    if(cycleType==1) {
      // CC - constant current, stop by max voltage
      out1=outC_avg; // controlled variable
      out2=outV_avg; // stop variable
    } else if(cycleType==2) {
      // CV - constant voltage, stop by min current
      out1=outV_avg; // controlled variable
      out2=outC_avg;  // stop variable
    }

    
    // print out stats - but only every few hundred cycles; n=300 generally corresponds to ~2-3Hz
    if(n>measCycle_len) {
      n=0;
      nn++; // count meascycles - generally # of secs

      // AH meter
      unsigned long timer_now=millis();
      unsigned int delta=int(timer_now-timer);
      AH_charger+=outC_avg*delta/1000/3600;
      timer=timer_now;

#ifdef NiXX
      // this preps for the dVdT etc
      updateMovingAverages(outV_avg);
#endif      
      
      // check thermal parameters
      heatSinkT=read_heatSinkT();
      if(heatSinkT<lowHeatSinkT) {
        // turn off the fans
        fanduty=0; analogWrite(pin_fan, 0);
      } else {
        if(heatSinkT>midHeatSinkT) {
          // warming up. turn on the fans
          setFanDuty(fanduty, MAXFANDUTY); // gradual ramp
          fanduty=MAXFANDUTY;
          if(heatSinkT>=maxHeatSinkT) {
            // start derating
            // map 0-100% derating (relative to the current max current) to 
            // maxHeatSinkT-ABSmaxHeatSinkT heatsink range
            float factor=abs((ABSmaxHeatSinkT-heatSinkT)/(ABSmaxHeatSinkT-maxHeatSinkT));
            maxOutC1=maxOutC*factor;
            if(maxOutC*maxOutV>5000) {
              maxOutC1*=factor; // additional derating due to faster heatup
            } 
            if(heatSinkT>ABSmaxHeatSinkT) {
              // overheating. stop charger.  wait until drops enough to restart
              stopPWM();
              out1=out2=outV=outC=outC_avg=outV_avg=0; // force duty ramp
              resetDelayParams();
              myLCD->clrScreen();        
  
              while(1) {
                sprintf(str, "Overheating, T=%dC. Cooling to T=%dC", (int)heatSinkT, (int)midHeatSinkT);
                myLCD->printStr(0, 1, 2, 0x1F, 0, 0, str);
                delay(1000);
                heatSinkT=read_heatSinkT();
                if(heatSinkT<midHeatSinkT) {
                  myLCD->clrScreen();
                  break; // exit cycle when the temp drops enough
                }
              }
            } // ABSmaxHeatSink condition
          } 
        }

      } // end thermal management
      
      // check HVC signal from BMS - only if BMS is activated
      if(BMSon && digitalRead(pin_BMS)==LOW) { // active LOW (so you need to pull up by connecting miniBMS loop to EOC signal)
        // BMS commanding charger to stop
        // noise protection - ensure signal stays on for 100ms or so
        int m=0;
        while(1) {
          m++;
          if(digitalRead(pin_BMS)==HIGH) break; // false alarm - just exit the loop and continue
          delay(1);
          if(m>100) {
            // this is for real
#ifdef DEBUG        
            printClrMsg("BMS Stop Signal. Exiting in 5 sec...", 5000, 0x1f, 0x3f, 0);
#endif
            stopPWM();          
            return 1; // assume this is a normal end-of charge but do not allow any more steps (return 1)
          }
        }
      } 
      
      // check the timer
      unsigned int runtime=(unsigned int)((millis()-timer_ch)/60000); // in minutes
      if(timeOut>0 && runtime>timeOut) {
        // timer run out
#ifdef DEBUG        
        printClrMsg("Timeout. Exiting in 5 sec...", 5000, 0x1f, 0x3f, 0);
#endif
        stopPWM();      
        return 1; // assume this is a normal end-of charge but do not let any more steps (return 1)
      }
      
      //==================== print all parameters
      printParams(duty, outV_avg, outC_avg, heatSinkT, AH_charger, dVdt);
     
#ifdef DEBUG      
      // reset duty to starting point every so often to trace the voltage-current cross-over bug
      if(nn>100) {
        nn=0;
#ifdef PFC    
        duty=PWM_res*(outV0/370);
#else
        duty=PWM_res*(outV0/330);  
#endif        
      }
#endif
      
      // check if need to stop
      if(isBtnPressed(pin_pwrCtrlButton)) {
        int b1=0, b2=0;
        printClrMsg("charger stopped at user request. Press same button to exit. Press the other button to resume", 1000, 0x1f, 0x3f, 0);   
        stopPWM();
        out1=out2=outV=outC=outC_avg=outV_avg=0; // force duty ramp
  
        do {
          delay(100);
          b1=isBtnPressed(pin_pwrCtrlButton);
          b2=isBtnPressed(pin_pwrCtrl2Button);      
        } while(!b1 && !b2);
        myLCD->clrScreen();
        // button pressed. which one?
        if(b1) {
          stopPWM();           
          return 1; // out of the main charger loop, do not allow second cycle
        }

        // resume operation
        myLCD->clrScreen();
        resetDelayParams();
      }
      
#ifndef DEBUG        
      // check mains
      if(read_mV()<minMains) {
        printClrMsg("Lost AC input. Exiting in 5 sec...", 5000, 0x1f, 0x3f, 0);
        stopPWM();           
        return 1;
      }
#endif
      
      // end measCycle loop
    }
        
    //-------------- MAIN DUTY CYCLE MANAGEMENT LOGIC ----------------------------
    // use small hysteresis (spread*2) to avoid jitter   
    // if current or voltage too LOW, INcrease duty cycle
    if(out1 < CX-spread) {
      if(duty<MAXDUTY && outC<maxOutC1 && outV<maxOutV) {
        duty++; 
      }
    } else {
      out1Reached=1;
    }
    // if current or voltage too HIGH, DEcrease duty cycle       
    if(out1 > CX || outC>maxOutC1 || outV>maxOutV) {
      if(duty>0) {
        duty--;
      }
    }
    
    // slow down when getting close to the resting battery voltage - for stability
#ifdef SLOWUPDATE
    // ramp fast to approximate duty cycle that would be needed to match battery voltage
    //               then slow down to 0.1% duty / sec
#ifdef PFC    
    if(duty/PWM_res>outV0/370 && outC>5) { // PFC DC rail voltage
#else
    if(duty/PWM_res>outV0/330 && outC>5) { // 240VAC rectified or 120VAC doubled-rectified (non-PFC units)  
#endif
      if(outV0<10) { // this means we are starting into a resistive load. don't slow down just as much
        stepDelay=100; // 0.1 second per cycle
        measCycle_len=10; 
        nSamplesStopVar=10; 
        nReadSamples=10; 
      } else {
        stepDelay=1000; // 1 second per cycle
        measCycle_len=1; // every cycle is a measuring cycle
        nSamplesStopVar=1; // no moving average since cycle is so slow
        nReadSamples=100; // more averaging - we need to average over at least half-cycle of input AC (>8ms)
      }
    }
#endif
    
    if(out1 < -10 || out2 < -10) {
      // sensor polarity problems. abort
      sprintf(str, "Sensor polarity / calibration problems (out1=%d, out2=%d). Turn off and recalibrate / check your wiring", int(out1), int(out2));
      printClrMsg(str, 30000, 0, 0x3f, 0);
    }
    //---------- END MAIN DUTY CYCLE MANAGEMENT LOGIC ----------------------------

    // check for break conditions - only on secondary variable!
    breakCycle=0;
    if(stopType==1 && out2 > stopValue) {
      breakCycle=1;
    } 
    // also, no point to break on stopMin before the CX condition has been reached
    if(stopType==2 && out2 < stopValue && out1Reached==1) {
      breakCycle=1;
    }
#ifdef NiXX
    // check dV/dt and break if it is too small - stopValue is in % of pack voltage change per second
    // on a 216V nominal pack, 1E-05 stopValue corresponds to dVdt=2mV/s
    if(stopType==3 && dVdt/maxOutV < stopValue) {
      if(min_up > 5) { // ignore first few min of the charge
        breakCycle=1;        
      }
    }
#endif

    // do we REALLY need to break?
    if(breakCycle) {
      breakCnt++;
      if(breakCnt>stopCycles) {
        delay(1000);
        // printClrMsg("Step done. Exiting", 5000, 0, 0x3f, 0);
        break;
      }
    } else {
      breakCnt=0;
    }
    
  }; 

  stopPWM();
  
  return 0;
}

//============================ HELPER FUNCTIONS ====================================
void stopPWM() {
  duty=0; 
  Timer1.setPwmDuty(pin_PWM, 0);           
}

//============================ voltage readout functions =====================
// read output voltage
float readV() {
  return (sampleRead(pin_bV)-V_o_bV)*divider_k_bV; // ISO124 isolation opamp
}

// read mains voltage
float read_mV() {
  return (sampleRead(pin_mV)-V_o_mV)*divider_k_mV; // ISO124 isolation opamp
}
//============================ end voltage readout functions =====================

void setMaxC(float maxC) {
  Timer1.setPwmDuty(pin_maxC, 1024./Aref*(V_o_C+k_V_C*maxC));
}

//============================ current readout functions =====================
// read output charger current
float readC() {
  // read current pin
  return (sampleRead(pin_C)-V_o_C)/k_V_C;
}
//================================= end current functions ========================

//====================== temperature readout functions ===========================
// read the charger heatsink temp
float read_heatSinkT() {
  return read_T(pin_heatSinkT, R0_hST, R1_hST, V_hST, B_hST);
}
// master temp readout, using formulas from http://en.wikipedia.org/wiki/Thermistor
float read_T(int pin, float R0, float R1, float V, float B) {
  float sensor_T_V=sampleRead(pin);
  float R=R1/(V/sensor_T_V-1);
  return 1/(log(R/R0)/B+1/T0)-273; 
}

float sampleRead(int pin) {
  float sum=0;
  for(int i=0; i<nReadSamples; i++) {
    sum+=analogRead(pin)*Aref/1024.; // 10-bit ADC
    if(waitReadSamples!=0) delayMicroseconds(waitReadSamples);
  }
  return sum/nReadSamples;
}

void resetDelayParams() {
    // reset delay parameters
    stepDelay=stepDelay0;
    measCycle_len=measCycle_len0;
    nSamplesStopVar=nSamplesStopVar0;
    nReadSamples=nReadSamples0;
    
    out1Reached=0;
}

// SerialLCD Functions
void printParams(float duty, float outV, float outC, float b1T, float curAH, float dVdt) {
  char tempstr[16];
  // myLCD->setOpacity(1); // so that we override previous text
  sprintf(str, "Duty = %s%%  ", ftoa(tempstr, 100.*duty/PWM_res, 1)); myLCD->printStr(0, 0, 2, 0x1f, 0x3f, 0x1f, str);      
//   sprintf(str, "Freq = %dkHz ", int(1000/period)); myLCD->printStr(0, 1, 2, 0x1f, 0x3f, 0x1f, str);      
  sprintf(str, "Out = %sA, %dV   ", ftoa(tempstr, outC, 1), int(outV)); myLCD->printStr(0, 3, 2, 0x1f, 0x3f, 0, str);      
  sprintf(str, "Temp = %dC  ", int(b1T)); myLCD->printStr(0, 5, 2, 0x1f, 0, 0, str);      
  sprintf(str, "AH in = %sAH", ftoa(tempstr, curAH, 1)); myLCD->printStr(0, 6, 2, 0x1f, 0x3f, 0, str);      
  sprintf(str, "Runtime = %umin", min_up); myLCD->printStr(0, 8, 2, 0, 0, 0x1f, str);

#ifdef NiXX
  if(min_up>=5) {
    // print dVdt only if we are past initial settling period
    sprintf(str, "dVdt = %s     ", ftoa(tempstr, dVdt*1000., 1)); myLCD->printStr(0, 9, 2, 0, 0, 0x1f, str);
  }
#endif
}

void printClrMsg(const char *str, const int del, const byte red, const byte green, const byte blue) {
  myLCD->clrScreen();
  myLCD->printStr(0, 2, 2, red, green, blue, str);      
  delay(del);
}

char *ftoa(char *a, double f, int precision)
{
  long p[] = {0,10,100,1000,10000,100000,1000000,10000000,100000000};
  
  char *ret = a;
  long heiltal = (long)f;
  itoa(heiltal, a, 10);
  while (*a != '\0') a++;
  *a++ = '.';
  long desimal = abs((long)((f - heiltal) * p[precision]));
  itoa(desimal, a, 10);
  return ret;
}

void setFanDuty(int fd, int tfd) {
  // if duty too far from each other, do it slowly
  if(tfd>fd+10) {
    for(int f=fd; f<tfd; f+=10) {
      analogWrite(pin_fan, f);
      delay(1000); // let it speed up
    }
  } else {
    analogWrite(pin_fan, tfd);
  }  
}

//--------------------Calculate moving Averages-------------
void updateMovingAverages(float V) {
  float avg = V_ravg[1];
  unsigned int k = ele_counter;
  float updated_avg = avg * (k/float(k+1)) + V/float(k+1);
  if( ele_counter >= 150 /*|| abs(updated_avg - avg) < 0.00001*/) {
    // switch averages and calculate dVdt
    unsigned long now = millis();
    if(t_ms > 0) {
      float time_interval = (now - t_ms) * 0.001; 
      dVdt = (V_ravg[1] - V_ravg[0]) / time_interval;
    }
    ele_counter = 0;
    V_ravg[0] = V_ravg[1];
    t_ms = now;
  } 
  else {
    V_ravg[1] = updated_avg;
    ++ele_counter;
  }
}

unsigned int MenuSelector2(unsigned int selection_total, const char * labels[])
{
  unsigned int selection = 0;
  unsigned int temp_selection = 1;
  
  //sprintf(str, "[%s]", labels[temp_selection-1] ); 
  //myLCD->printStr(0, 3, 2, 0x1f, 0x3f, 0x1f, str);
  myLCD->printStr(0, 3, 2, 0x1f, 0x3f, 0x1f, "[              ]");
  myLCD->printStr(1, 3, 2, 0x1f, 0x3f, 0x1f, labels[temp_selection-1]);

  while(!selection)
  {
    int step_btn = digitalRead(pin_pwrCtrlButton);
    int select_btn = digitalRead(pin_pwrCtrl2Button);
    if(step_btn == HIGH)
    {
      ++temp_selection;
      if(temp_selection > selection_total) temp_selection = 1;
      myLCD->printStr(0, 3, 2, 0x1f, 0x3f, 0x1f, "[              ]");
      myLCD->printStr(1, 3, 2, 0x1f, 0x3f, 0x1f, labels[temp_selection-1]);
      
      // ideally, this should call a StatusDisplay method and simply pass selection index
      // StatusDisplay should encapsulate all the complexities of drawing status info onto the screen
      // alternatively myLCD can be re-purposed for this
    }
    else
    if(select_btn == HIGH)
    {
      selection = temp_selection;
      myLCD->printStr(0, 3, 2, 0x1f, 0x0, 0x0, "(              )");
      myLCD->printStr(1, 3, 2, 0x1f, 0x0, 0x0, labels[selection-1]);
      // similar to the above, should delegate display to StatusDisplay object
    } 
    delay(80);
  }

  delay(200);
  return selection - 1;
}

// this takes max of 50ms if the button is pressed
int isBtnPressed(int pin) {
  if(digitalRead(pin)==HIGH) {
    // check if noise
    for(int zz=0; zz<10; zz++) {
      if(digitalRead(pin)==LOW) return 0;
      delay(5);
    }
    return 1;
  } else {
    return 0;
  }
}
  

int BtnTimeout(int n, int line)
{
  while(n > 0)
  {
    sprintf(str, "(%d sec left) ", n); 
    myLCD->printStr(0, line, 2, 0x1f, 0x3f, 0, str);

    for(int k=0; k<100; k++) {
      if(digitalRead(pin_pwrCtrlButton)==HIGH || digitalRead(pin_pwrCtrl2Button) == HIGH) return 1;
      delay(10);
    }

    --n;
  }
  
  return -1;
}

int DecimalDigitInput3(int preset)
{
  //  myLCD->setOpacity(1);
  int d3=preset/100;
  int d2=(preset/10)%10;
  int d1=abs(preset%10);
  int digit[3] = { d3, d2, d1 };
  int x = 0; // 0-1-2-3-4
  // 0x30 ascii for "0"
  str[1] = 0x0; // eol 

  while(x < 4)
  {
    int step_btn = digitalRead(pin_pwrCtrlButton); // increments digit
    int select_btn = digitalRead(pin_pwrCtrl2Button); // moves to next digit
 
    if(step_btn == HIGH) {
      if(x > 2) x = 0;
      else {
        // increment digit
        ++digit[x];
        // wrap at 3 (for 100s) or at 9 (for 10s and 1s) 
        if(x == 0 && digit[x] > 3) digit[x] = 0;
        if(digit[x] > 9) digit[x] = 0;
      }      
    } else 
    if(select_btn == HIGH) {
      ++x;
    } 

    printDigits(0, digit, 1);
  
    if(x < 3) {
      // still on digits. Reprint the digit we are on in a different color now so we see what's being changed
      str[0] = 0x30+digit[x];
      printDigit(x, 0, str);
    } else 
    if(x == 3) {
      // selection made - show all in the 'changing' color
      printDigits(0, digit, 0);
    }
    
    delay(150);
  }
  
  printDigits(8, digit, 0);

  return (digit[0]*100+digit[1]*10+digit[2]);
}

void printDigits(int start, int * digit, int stat) {
  str[0] = 0x30+digit[0];
  printDigit(start++, stat, str);
  str[0] = 0x30+digit[1];
  printDigit(start++, stat, str);
  str[0] = 0x30+digit[2];
  printDigit(start, stat, str);
}
void printDigit(int x, int stat, char * str) {
  if(stat==0) myLCD->printStr(x, 5, 2, 0x1f, 0x3f, 0x0, str); // yellow
  if(stat==1) myLCD->printStr(x, 5, 2, 0x8, 0x8, 0x1f, str); // blue
}
