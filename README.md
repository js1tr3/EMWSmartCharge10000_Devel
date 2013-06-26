EMWSmartCharge10000_Devel
=========================

Development Firmware

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
