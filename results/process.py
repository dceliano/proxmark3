import csv
import os
import numpy
from collections import OrderedDict


NUM_TRIALS = 15

items = os.listdir(".")
filenames = []
for name in items:
    if name.endswith(".txt"):
        filenames.append(name.split('.')[0]) #don't include the file extension
#print(filenames)

fpga_times = {}
arm_times = {}
used_configurations = []
dbflag = 0

#Read in all the data from the .txt files.
for filename in filenames:
    configuration = filename[:19]
    if(configuration not in used_configurations):
        fpga_times[configuration] = [] #add an empty array to the dict
        arm_times[configuration] = [] #add an empty array to the dict
        used_configurations.append(configuration)
    with open('{}.txt'.format(filename), 'rb') as rawfile:
        for line in rawfile:
            if("03F2" in line):
                dbflag = 1
            if(dbflag and ("Cycle count (for one bit)" in line)):
                armtimestamp = line[81:-11]
                arm_times[configuration].append(int(armtimestamp))
            if(dbflag and ("Just read timestamp over SPI" in line)):
                fpgatimestamp = line[71:-12]
                fpga_times[configuration].append(int(fpgatimestamp))
                dbflag = 0
#print("fpga times: ", fpga_times)
#print("arm times: ", arm_times)
for array in fpga_times:
    num_rounds = int(array[12])
    fpgaarray = fpga_times[array]
    armarray = arm_times[array]
    print "------------------------"
    print "Stats for num rounds = " + str(num_rounds) + " (configuration = " + array + ")"
    print "------------------------"
    for j in range(0,num_rounds):
        rndtimes = [fpgaarray[i] for i in range(j, NUM_TRIALS*num_rounds, num_rounds)]
        print "++++++++++++++++++++ Stats for rnd " + str(j+1) + " ++++++++++++++++++++"
        #print "FPGA times: "
        #print(rndtimes)
        mean = numpy.mean(rndtimes) * (1/16e6) * 1e3
        stddev = numpy.std(rndtimes) * (1/16e6) * 1e3
        print "FPGA mean = " + str(mean) + "ms, std_dev = " + str(stddev) + "ms"
        rndtimes = [armarray[i] for i in range(j, NUM_TRIALS*num_rounds, num_rounds)]
        #print "ARM times: "
        #print(rndtimes)
        osc_clk = 13.56e6
        ssp_clk = osc_clk / 16
        mean = numpy.mean(rndtimes) * (1/ssp_clk) * 1e3
        stddev = numpy.std(rndtimes) * (1/ssp_clk) * 1e3
        print "ARM mean = " + str(mean) + "ms, std_dev = " + str(stddev) + "ms"
