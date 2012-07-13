from math import *

gammaTableSize = 256
gamma = 2.5
maxValue = 255


def pad(val):
	if val < 10:
		return "  " + str(val)
	elif val < 100:
		return " " + str(val)
	else:
		return str(val)

table = []
for i in range(gammaTableSize):
	table.append( int(ceil( pow( float(i)/(gammaTableSize-1), gamma ) * maxValue )) )

paddedTable = [pad(val) for val in table]

print "{" + ', '.join( paddedTable ) + "}"

