import numpy

gps_file = '../config/log/Graph1.txt'
acc_file = '../config/log/Graph2.txt'

gps_values = numpy.loadtxt(gps_file, delimiter=',', dtype='Float64', skiprows=1)[:,1]
#print(gps_values)

acc_values = numpy.loadtxt(acc_file, delimiter=',', dtype='Float64', skiprows=1)[:,1]

gps_stddev = numpy.std(gps_values)
print(gps_stddev)

acc_stddev = numpy.std(acc_values)
print(acc_stddev)