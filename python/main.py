#!/usr/bin/python
# -*- coding:utf-8 -*-

import time
import ADS1263
import RPi.GPIO as GPIO
import math

# Environmental constant
surface = "asphalt"

REF = 2.5 #Internal reference of 2.5V         
TEST_ADC1 = True

# Thermistor  108 related constants
Vx = 5.000
RTD_108_res = 41000
A = 8.271111e-4
B = 2.088030e-4
C = 8.059200e-8

#Second thermistor ir120 related constants
Vy = 3.300
RTD_120_res = 77020
A1 = 9.555514e-4
B1 = 2.164256e-4
C1 = 1.437491e-7

#Thermopile calibration constants ir120
IRSensor_x = 1.956170E-05
IRSensor_y = 3.316540E-01
IRSensor_z = -1.288665E+00
Boltz_const = 5.67E-8


EMISSIVITY_PRESETS = {
    'blackbody': 1.00,
    'track': 0.94,
    'asphalt': 0.93,
    'concrete': 0.92,
    'grass': 0.96,
    'water': 0.96,
    'soil': 0.92,
    'sand': 0.90,
    'snow': 0.97,
    'rubber': 0.95,
}

def get_emissivity(surface: str, default=1.0) -> float:
    """
    Return emissivity for a given surface name.
    """
    return EMISSIVITY_PRESETS.get(surface.lower(), default)



def thermistor_to_temp(Rs, A, B, C):
    """
    Convert thermistor resistance Rs (ohms) to temperature in Celsius.
    
    Args:
        Rs (float): Thermistor resistance in ohms.
        A, B, C (float): Steinhart-Hart coefficients for the thermistor.
    
    Returns:
        float: Temperature in °C
    """
    lnR = math.log(Rs)  # natural log of resistance
    Tc = (1 / (A + B * lnR + C * lnR**3)) - 273.15
    return Tc


def thermopile_to_temp(Van, sign, temp, emissivity, bg_temp_c):
    """
    Convert voltage from thermopile to temperature in Celsius.
    
    Args:
        Van (float): Analog voltage from thermopile
        temp (float): Thermistor temperature
        emissivity (float): Emissivity of the surface (0 to 1)
        bg_temp_c (float): Background temperature in °C (optional)
        sign (float): Sign of the voltage (1.0 or -1.0)
    
    Returns:
        float: Temperature in °C
    """
    IRSensor_Volt = Van * 1000.0 * sign # Converting to mV
    IRSensorCan_Temp = temp
    IRSensor_Volt_TC = IRSensor_Volt * 1.0004 ** (IRSensorCan_Temp - 25.0)
    #‘Apply calibration factors
    IRSensor_E = IRSensor_x*(IRSensor_Volt_TC**2) + (IRSensor_y*IRSensor_Volt_TC) + IRSensor_z


    if bg_temp_c is None:
        bg_temp_c = IRSensorCan_Temp
    
    T4 = (IRSensor_E/Boltz_const)+((bg_temp_c+273.15)**4) #Stefan Boltzmann
    
    if emissivity < 1.0:
        T4 = ((T4 - ((bg_temp_c + 273.15)**4 * (1-emissivity)))/ emissivity)

    ITSensor_T = (T4**0.25)-273.15
    return ITSensor_T


try:
    ADC = ADS1263.ADS1263()
    
    # ---------------------------------------------------------------
    # STEP 1: Initialize with default High Speed (To avoid KeyError)
    # Use high data rate to reduce timeout risk and speed reads.
    # ---------------------------------------------------------------
    if (ADC.ADS1263_init_ADC1('ADS1263_400SPS') == -1):
        exit()
    
    # ADS1263 initialization sequence 
    ADC.ADS1263_WriteReg(0x04, 0x80) #FIR mode filter enabled, default config
    ADC.ADS1263_WriteReg(0x03, 0x10) # Enable CHOP mode
    ADC.ADS1263_WriteReg(0x05, 0x88) # PGA bypassed with data rate of 400SPS
    ADC.ADS1263_WriteReg(0x01, 0x13) #Reset, VBIAS enabled (perhaps not required), internal reference enabled
    ADC.ADS1263_WriteReg(0x0F, 0x00) # REFMUX = 0x00 for internal reference

    #Calibration sequence
    time.sleep(0.5) # Wait before calibration
    ADC.ADS1263_WriteReg(0x06,0xFF)  # For calibration chainge input mux to FFH, all inputs are floating
    ADC.ADS1263_WriteCmd(0x19) # Send SFOCAL (Self Offset Calibration) command
    time.sleep(2) # Wait for calibration to finish
    ADC.ADS1263_WriteReg(0x06,0x01) # Back to default value
    
    # --------------------------------------------------------------- ADS1263 initialization sequence end ----------------------------------------------------------------

    ADC.ADS1263_SetMode(1)# Differential mode 

    if(TEST_ADC1):       # ADC1 Test, not required
        channelList = [0, 1, 2]
        while(1):
            ADC_Value = ADC.ADS1263_GetAll(channelList) 
            for i in channelList:
                if ADC_Value[i] is None:
                    print("ADC1 IN%d read skipped due to timeout" % i)
                    continue
                if(ADC_Value[i]>>31 ==1):
                    Van = (REF*2 - ADC_Value[i] * REF / 0x80000000) 
                    sign = -1.0 # capturing the sign since Van is always positive. The sign indicates the direction of the differential voltage with reference to the thermistor voltage. This is relevant for thermopile
                    print("ADC1 IN%d = -%lf" %(i, (REF*2 - ADC_Value[i] * REF / 0x80000000)))  
                else:
                    Van = (ADC_Value[i] * REF / 0x7fffffff)   # 32bit
                    sign  = +1.0
                    print("ADC1 IN%d = %lf" %(i, (ADC_Value[i] * REF / 0x7fffffff)))   # 32bit
                if ( i == 0): # Thermistor 108 channel
                    Rs = 1000 * (Vx/Van) - RTD_108_res
                    print("Resistor value Channel 0:%lf"%(Rs))
                    temp = thermistor_to_temp(Rs, A, B, C)
                    print("Temperature value Channel 0: %lf"%(temp))
                if ( i == 1): # Thermistor 120 channel
                    Rs1 = (Van/(Vy-Van)*RTD_120_res)
                    print("Resistor value Channel 1: %lf"%(Rs1))
                    temp1 = thermistor_to_temp(Rs1, A1, B1, C1)
                    print("Temperature value Channel 1: %lf"%(temp1))
                if ( i == 2): # Thermopile 120 channel
                    emissivity = 1.0
                    bg_temp_c = None
                    emissivity = get_emissivity(surface)
                    temp2 = thermopile_to_temp(Van, sign, temp1, emissivity, bg_temp_c)
                    print("Temperature value Channel 2: %lf"%(temp2))
        
    ADC.ADS1263_Exit()

except IOError as e:
    print(e)
   
except KeyboardInterrupt:
    print("ctrl + c:")
    print("Program end")
    ADC.ADS1263_Exit()
    exit()

