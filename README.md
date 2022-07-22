# REDMOON: Radiation Environment and Dose Monitoring Onboard a Nano-rover
Code for the Radiation Payload of the Lunar Zebro Rover

The Code is divided into the following main categories:
## BREADBOARD CODE
Code written during developement of the Radiation Payload Firmware using two different Breadboard models:
A) Arduino connected to an FGDOS Breakout Board
B) MSP430FR5969 LaunchPad connected to an FGDOS Breakout Board via a level shifter

Also contains code used to work out the functionality of various breakout boards such as:
1) Oscillator
2) TMP-100 temperature sensor
3) FRAM
4) RS-485 to UART transceiver
5) INA 219 Power measurment
6) Level shifters
7) MCP9600 Thermocouple readout

## FGDOS SOFTWARE
Consists of code used to interface with the FGDOS sensor using an Arduino in a breadboard setup.
Also code contributed by WdM for testing the FGDOS on a breadboard at HollandPTC

## TESTING CODE
Arduino and Energia scripts used for various functional, performance and development tests related to the Radiation Payload and the FGDOS Detector



## MISSION CODE


## Telemetry and Commands



## Jupyter Notebooks
This folder consists of various Jupyter Notebooks used for visulaization and data analysis of:
1) FGDOS Proton Beam Characterization
2) FGDOS Temperature Characterization
3) Radiation Payload Performance - Noise, Power and Temperature
4) Radiation Payload Performance - Proton Beam
5) FGDOS Characterization: Irradiation and Thermal Variation
