# -*- coding: utf-8 -*-
"""
Created on Thu Jul 18 16:10:02 2024

@author: amiles92
"""

import sys
import argparse
import pyvisa as visa
import time
import numpy as np
import matplotlib.pyplot as plt

plt.ion()

class voltageSettings:
    voltageRange = 0
    def __init__(self, resetFlag, threshVoltage, normIncrement, 
                 threshIncrement, numMeasurements, trigDelay):
        self.ivCheck = True
        self.instrument = initialisePowerSupply(resetFlag, numMeasurements, trigDelay)
        self.threshVoltage = threshVoltage
        self.normIncrement = normIncrement
        self.threshIncrement = threshIncrement
        self.numMeasurements = numMeasurements
        self.trigDelay = trigDelay

    def __init__(self, resetFlag, threshVoltage, normIncrement, threshIncrement):
        self.ivCheck = False
        self.instrument = initialisePowerSupply(resetFlag, 0, 0)
        self.threshVoltage = threshVoltage
        self.normIncrement = normIncrement
        self.threshIncrement = threshIncrement
        return

def grabArgs():

    usage = 'python3 RS232_IV_21.py [-h] <voltage-level> <voltage-increment> ' + \
        '<threshold-voltage> <threshold-increment> [-i <file-basename> ' + \
        '<num-measurements> <trigger-delay>] [-j <jump-voltage>] [--no-reset]'

    parser = argparse.ArgumentParser(prog="VoltageControl_RS232",
                                        description='Monash slowcontrol system for Keithley 6487 power supply',
                                        usage=usage,
                                        formatter_class=argparse.RawTextHelpFormatter)

    parser.add_argument('targetVoltage', type=float, metavar='voltage-level',
                    help='Voltage Level Required (volts .1f)')

    parser.add_argument('normIncrement', type=float, metavar='voltage-increment',
                        help='Voltage Increment Before Threshold Voltage (volts .1f)')

    parser.add_argument('threshVoltage', type=float, metavar='threshold-voltage',
                        help='Threshold Voltage (volts .1f)')

    parser.add_argument('threshIncrement', type=float, metavar='threshold-increment',
                        help='Voltage Increment After Threshold Voltage (volts .1f)')

    parser.add_argument('--no-reset', dest='reset', action='store_false',
                            help='Stop reset flag being sent to the hardware. Mostly to be used\n'
                                '  when the connection was previously severed unexpectedly')

    parser.add_argument('-i', dest='ivSettings', action="extend", nargs=3, type=str, 
                        metavar=('file-basename', 'num-measurements', 'trigger-delay'),
                        help='For taking the IV Curve, three arguments are required\n'
                            '  Base file name for saving data to\n'
                            '  Number of current measurements at each voltage (int)\n'
                            '  Trigger delay between measurements (seconds .1f)')

    parser.add_argument('-j', dest='jumpTarget', type=float, metavar='jump-voltage',
                        help='Set a SAFE voltage to jump to/from (volts .1f)')


    if len(sys.argv)==1:
        parser.print_help()
        parser.exit()

    args = vars(parser.parse_args())

    if (args['ivSettings'] != None):
        try:
            args['ivSettings'] = [args['ivSettings'][0], int(args['ivSettings'][1]), float(args['ivSettings'][2])]
        except:
            parser.exit(message="Invalid value for IV Curve settings (str, int, float): %s\n" % ", ".join(args['ivSettings']))
    
    return args

def initialisePowerSupply(resetFlag, numMeasurements, trigDelay):
    ############################################Initialisation Commands##############################################
    rm = visa.ResourceManager()
    print(rm.list_resources())
    #Create a connection with the RS232 port listed when printing list_resources
    time.sleep(0.5)
    instrument=rm.open_resource(u'ASRL/dev/ttyUSB0::INSTR')
    instrument.baud_rate=9600
    instrument.write_termination = '\n'
    instrument.read_termination = '\n'
    if numMeasurements > 0:
        instrument.timeout=np.max([100*numMeasurements+trigDelay,25])
    #Setup command character terminations - basically the settings so that the PC and instrument
    #recognise the end of a command - this is instrument dependent unfortunately and not always
    #clear as to what it should be set to.

    #Send a query command for instrument identification to test both read and write commands are
    #functioning at both ends

    #Display settings
    print("%%%%%%%%%RS232-USB Configuration Settings%%%%%%%%%%%")
    # print("IP Address: 192.168.10.200")
    # print("Connection Port: 1234")
    print("Write Termination Character: \\n")
    print("Read Termination Character: \\n")
    print("Instrument Address: 22")

    if(resetFlag==1):
        print(instrument.write("*RST"))
        print("RESET")

    instrument.write('*IDN?')
    print(instrument.read('\n'))

    return instrument

def getVoltage(instrument):
    instrument.write("INIT")
    instrument.write("FORM:ELEM READ,TIME,VSO")
    instrument.write("READ?")
    return float(str(instrument.read()).replace('A','').split(',')[2])

def setVoltageRange(vs, targetVoltage):
    if(targetVoltage<10.0):
        vs.instrument.write("SOUR:VOLT:RANG 10") #Set voltage range to 10 V range
        print("Voltage Range: 10 V")
        vs.voltageRange = 10
    elif(targetVoltage>=10.0 and targetVoltage<50.0):
        vs.instrument.write("SOUR:VOLT:RANG 50") #Set voltage range to 50 V range
        print("Voltage Range: 50 V")
        vs.voltageRange = 50
    elif(targetVoltage>=50.0 and targetVoltage<500.0):
        vs.instrument.write("SOUR:VOLT:RANG 500") #Set voltage range to 50 V range
        print("Voltage Range: 500 V")
        vs.voltageRange = 500
    else:
        print("Invalid Target Voltage:", targetVoltage)
        print("---ISSUES---")
        return 0
    return 1

def runSetup(vs, targetVoltage, currentLimit, currentRange=None):
    if vs.ivCheck:
        vs.instrument.write("FORM:ELEM READ,TIME,VSO")#Read current, timestamp, voltage
        vs.instrument.write("TRIG:DEL 0.0") #Trigger delay of 0;;; IT'S NOT WORKING!!
        vs.instrument.write("TRIG:COUN " + str(vs.numMeasurements)) #Trigger count of 25
        vs.instrument.write("NPLC .01")
        vs.instrument.write("RANG " + currentRange) #Current range of 2 uA
        vs.instrument.write("AVER:COUN 100") #100 point averaged filter
        vs.instrument.write("AVER:TCON REP") #Repeating filter
        vs.instrument.write("AVER:ON") #Turn filter on
    
    # --------------------------------------------------------------------------
    out = setVoltageRange(vs, targetVoltage)
    if out == 0:
        exit()

    vs.instrument.write("SOUR:VOLT:ILIM " + currentLimit)
    vs.instrument.write("SOUR:VOLT:STAT ON") #Turn voltage source on
    return
    # --------------------------------------------------------------------------

def rampDown(vs, targetVoltage, measure):
    voltageRead = getVoltage(vs.instrument)

    data = [[], [], []]

    if measure:
        measurement = takeMeasurement(vs)
        for i in range(len(measurement)):
            data[i].append(measurement[i])

    while (voltageRead > targetVoltage):
        newVoltage = voltageRead
        if (voltageRead > vs.threshVoltage):
            newVoltage = voltageRead - vs.threshIncrement
        else:
            newVoltage = voltageRead - vs.normIncrement
        
        if newVoltage < targetVoltage:
            newVoltage = targetVoltage

        command = "SOUR:VOLT " + str(round(newVoltage,2))
        sys.stdout.write("\r Voltage: %.2f V " % newVoltage)
        sys.stdout.flush()
        vs.instrument.write(command)

        if measure:
            measurement = takeMeasurement(vs)
            for i in range(len(measurement)):
                data[i].append(measurement[i])
        else:
            time.sleep(2)

        voltageRead = getVoltage(vs.instrument)
    
    print("")

    if measure:
        return data

def rampUp(vs, targetVoltage, measure):
    voltageRead = getVoltage(vs.instrument)

    data = [[], [], []]

    if measure:
        measurement = takeMeasurement(vs)
        for i in range(len(measurement)):
            data[i].append(measurement[i])

    while (voltageRead < targetVoltage):
        newVoltage = voltageRead
        if (voltageRead < vs.threshVoltage):
            newVoltage = voltageRead + vs.normIncrement 
        else:
            newVoltage = voltageRead + vs.threshIncrement
        
        if newVoltage > targetVoltage:
            newVoltage = targetVoltage

        command = "SOUR:VOLT " + str(round(newVoltage,2))
        sys.stdout.write("\r Voltage: %.2f V " % newVoltage)
        sys.stdout.flush()
        vs.instrument.write(command)

        if measure:
            measurement = takeMeasurement(vs)
            for i in range(len(measurement)):
                data[i].append(measurement[i])
        else:
            time.sleep(2)
            
        voltageRead = getVoltage(vs.instrument)
    
    print("")

    if measure:
        return data

def rampVoltage(vs, targetVoltage, measure=0):
    print("Ramping to {:.2f} V".format(targetVoltage))

    if measure == 1:
        vs.instrument.write("TRIG:COUN " + str(vs.numMeasurements))
        vs.instrument.write("TRAC:POIN " + str(vs.numMeasurements))
        vs.instrument.write("TRAC:CLE")
        vs.instrument.write("AVER:COUN 100")
        vs.instrument.write("AVER:ON")
    else:
        vs.instrument.write("TRIG:COUN 1")
        vs.instrument.write("TRAC:POIN 1")
        vs.instrument.write("TRAC:CLE")
        vs.instrument.write("AVER:COUN 1")
        vs.instrument.write("AVER:OFF")

    voltageRead = getVoltage(vs.instrument)

    if (voltageRead < targetVoltage):
        return rampUp(vs, targetVoltage, measure)

    if (voltageRead > targetVoltage):
        return rampDown(vs, targetVoltage, measure)

    return

def jumpVoltage(vs, targetVoltage):
    ############################################################################
    ##
    ##  THIS FUNCTION IS DANGEROUS AND SAFETY OF ELECTRONICS SHOULD BE CONFIRMED
    ##  BEFORE USING WITH SENSITIVE ELECTRONICS
    ##
    ############################################################################

    if targetVoltage > vs.voltageRange:
        setVoltageRange(vs, targetVoltage)

    command = "SOUR:VOLT " + str(round(targetVoltage,2))
    sys.stdout.write("\r Voltage: %.2f V " % targetVoltage)
    sys.stdout.flush()
    vs.instrument.write(command)
    print("")

    time.sleep(5)
    return

def takeMeasurement(vs):
    start = time.time()
    if vs.trigDelay > 0:
        time.sleep(vs.trigDelay)
    vs.instrument.write("SYST:ZCH OFF") #Turn zero checking off (zero checking is on it seems normally for changing the circuit)
    vs.instrument.write("SYST:AZER:STAT OFF") #
    vs.instrument.write("DISP:ENAB OFF") #Turn display off while setting up buffer
    vs.instrument.write("*CLS")
    vs.instrument.write("TRAC:POIN " + str(vs.numMeasurements)) #buffer expects 'n=measPoints' measurements
    vs.instrument.write("TRAC:CLE") #Clear buffer
    vs.instrument.write("TRAC:FEED:CONT NEXT") #I think this is for feeding measurements in one after the other
    vs.instrument.write("STAT:MEAS:ENAB 512")
    vs.instrument.write("*SRE 1")
    vs.instrument.write("*OPC?") #Check if ready
    vs.instrument.read() #Important to read this check otherwise this will be read into measurement array and the measurements will be read in the next iteration
    vs.instrument.write("INIT") #Start measurements
    vs.instrument.write("DISP:ENAB ON") #Turn display back on
    vs.instrument.write("TRAC:DATA?") #Request all stored readings

    results = (str(vs.instrument.read()).replace("A","")).split(',') #Read and format results
    RArray = (np.array(results)).astype(np.float64) #Convert results to float array

    diff = time.time() - start
    if diff < 2: # Makes sure we take at least two seconds between V jumps, to save MPPCs
        time.sleep(2 - diff)

    CArray = RArray[0::3]#Extract current measurements
    TArray = RArray[1::3]#Extract buffer timestamps
    VArray = RArray[2::3]#Read voltage source data (not needed so might take this out to improve read speeds)

    print("\nMean Voltage: {:.2e}\nMean Current: {:.2e}\n".format(np.mean(VArray),np.mean(CArray)))
    
    return CArray, TArray, VArray

def plotData(data):
    plt.figure()

    c, t, v = data

    plt.errorbar(np.mean(v,axis=2), np.mean(c,axis=2), yerr = np.std(c,axis=2))

    plt.show()

    return

def main(args, currentLimitStr, currentRangeStr):

    resetFlag = args['reset']

    targetVoltage   = args['targetVoltage'] # How high voltage is to be set (V)
    normIncrement   = args['normIncrement'] # Voltage increment below the threshold
    threshVoltage   = args['threshVoltage'] # Voltage level after which it will increment in ThreshIncrement volts
    threshIncrement = args['threshIncrement'] # Voltage increment above the threshold

    jump = False
    if args['jumpTarget'] != None:
        jump = True
        jumpTarget = args['jumpTarget']

    if args['ivSettings'] != None:

        directory =  "./data/IV_Curves/"
        fileName, numMeasurements, trigDelay = args['ivSettings']

        vs = voltageSettings(resetFlag, threshVoltage, normIncrement, 
                            threshIncrement, numMeasurements, trigDelay)
    
    else:
        vs = voltageSettings(resetFlag, threshVoltage, normIncrement, 
                            threshIncrement)

    if not resetFlag: # This is going to be pointless if reset is run
        rampVoltage(vs, 0, 0)
        vs.instrument.write("*RST") # reset now that Voltage is definitely 0

    if vs.ivCheck:
        runSetup(vs, targetVoltage, currentLimitStr)
    else:   
        runSetup(vs, targetVoltage, currentLimitStr, currentRangeStr)

    if jump:
        jumpVoltage(vs, jumpTarget)

    if vs.ivCheck:
        data = rampVoltage(vs, targetVoltage, 1)
    else:
        rampVoltage(vs, targetVoltage)

    if vs.ivCheck:
        with open(directory + fileName + ".npy", "wb") as f:
            datanp = np.array(data,dtype=np.float64)
            np.save(f, datanp)

    inputVoltage = float(input("Enter next voltage:\n"))
    while inputVoltage > 0:
        try:
            if inputVoltage > 500:
                print("Voltage entered too high!!")
            else:
                targetVoltage = inputVoltage
                if targetVoltage > vs.voltageRange:
                    out = setVoltageRange(vs, targetVoltage)
                    if out == 0:
                        jump = False
                        break
                rampVoltage(vs, inputVoltage, 0)

            inputVoltage = float(input("Enter next voltage:\n"))
            
        except:
            print("\nUnexpected error occurred, ramping down!")
            jump = False
            break
    try:
        if jump:
            rampVoltage(vs, jumpTarget, 0)
            jumpVoltage(vs, 0)
        else:
            rampVoltage(vs, 0, 0)

        vs.instrument.write("*RST") # Reset all parameters now that safely ramped down
    except:
        print("\nUnexpected error occurred while ramping to 0!!")
        print("Please ramp down the device manually")
        print("    To do so, press the \"Config/Local\" button and then use the")
        print("    up and down arrow buttons in the \"V-SOURCE\" box, with gray")
        print("    body colour and a white triangular arrow.")
        print("")
        print("    Change which digit to increment with the left and right arrow")
        print("    buttons directly below them, with white body colour and gray")
        print("    arrows.")
        print("")
        print("    Once safely ramped down, press the \"OPER\" button to switch")
        print("    the voltage off.")
        print("")
        print("    DO NOT USE THE PURE WHITE \"RANGE\" BUTTONS!!!")
        exit()

    
if __name__ == "__main__":

    ############################################################################
    # Fixed Parameters

    currentRangeStr = "2e-3" # 2 mA # Applicable to combined MPPCs
    # currentRangeStr = "2e-2" # 20 mA # Applicable to LEDs
    currentLimitStr = "2.5e-4"
    ############################################################################

    args = grabArgs()

    main(args, currentLimitStr, currentRangeStr)
