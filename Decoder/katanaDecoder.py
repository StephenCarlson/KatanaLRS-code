import wave, struct
from array import *
import matplotlib.pyplot as plt
from pylab import *
import sys

#print "Program Start"

file_name = ""

if len(sys.argv)>1 :
	file_name = sys.argv[1] + ".wav"
else:
	file_name = "SDRSharp_20140121_180030Z_430500kHz_AF" + ".wav"
print 'Using',file_name
#print file_name + ".wav"

w = wave.open(file_name,'r')

nchannels = w.getnchannels()
samplewid = w.getsampwidth()
framerate = w.getframerate()
nframes   = w.getnframes()

print "Channels: " + repr(w.getnchannels())
print "Width:    " + repr(w.getsampwidth())
print "Rate:     " + repr(w.getframerate())
print ""

#for i in w.readframes(20) #range(0,20)
#a = array('i') #[]
#a = w.readframes(1)

#print len(a)
length = w.getnframes()
#print a

a = zeros(length)

for i in range(0,length): #len(a)):
	frame = w.readframes(1)
	data = struct.unpack('hh',frame)[0]
	#print data
	a[i]=data
	


baudrate = 4800.0 #9600.0 #9143.0 # Increase Slightly Some more 9183.0 #
period = 1/baudrate
sampleDelta = 1/framerate

sampPerPeriod = framerate/baudrate #3.4 #
print "Sa/Mark: " + repr(sampPerPeriod)



# Discover Zero Crossings
diff = zeros(length)   #(1,length))
last = 0;
t = arange(0,length,1)
#print len(diff)

dcBias = 0 #0 #8500
max = 0

for i in range(0,len(diff)):
	#print t[i] # repr(a[i]) + " + " + repr(last)
	if( (a[i] >= dcBias and last < dcBias) ): # or (a[i] < 0 and last >= 0) ):
		diff[i] = 1000;
	elif (a[i] < dcBias and last >= dcBias):
		diff[i] = -1000;
		#print i
	if(a[i] > max):
		max = a[i]
	last = a[i]
	
#print "Max: " + repr(max)

annoyingOffset = -4000


# Find Preamble Sequence
last = 0
delta = 0
mark = zeros(length)+annoyingOffset
valid = zeros(length)+annoyingOffset
preambleCounter = 0
deltaSum = 0
state = "PREAMBLE"
	
for i in range(0,len(diff)):
	delta = i - last
	if(diff[i] != 0):
		last = i
		if(state == "PREAMBLE"):
			if( (sampPerPeriod*.5 < delta) and (delta < sampPerPeriod*1.5) ):
				mark[i] = 2000
				preambleCounter += 1
				deltaSum += delta
				#print i
			elif( (sampPerPeriod*3.6 < delta) and preambleCounter >= 10 and diff[i] < 0):
				line = "Valid Porch @ " + repr(i)
				#if(i<100000): line += "\t"
				line += "\tAvg Preamble Sa/Period: " + repr((deltaSum*1.0)/preambleCounter)
				print line
				state = "UART_START"
			else:
				preambleCounter = 0;
				deltaSum = 0;
				#state = "PREAMBLE"
		if(state == "UART_START"):
			if(diff[i] > 0):
				valid[i] = 3000
				state = "PREAMBLE"
			#elif(sampPerPeriod*3.6 < delta)
	# if(state == "UART_DECODE"):
		# if(sampPerPeriod*3.6 < delta
		

# for i in range(100):
	# print repr(int(round(sampPerPeriod*(i+0.5)))) + " " + repr(int(sampPerPeriod*(i+0.5)))
	# range(i+sampPerPeriod*0.5,i+10*sampPerPeriod,sampPerPeriod):

# Decode Serial Frames on Valid Syncs
timing = zeros(length)+annoyingOffset
#timing = timing*dcBias*.1

offset = 11 # (-1)
bytes = 80 #12 for 7Ch

for i in range(0,len(valid)):
	if(valid[i] != annoyingOffset): # Yes, yes, I know this is like O(n^3) or something like that, need to optimize
		line = "" #[]
		if bytes*sampPerPeriod*10+i >len(valid):
			break
		for j in range(bytes):
			octet = ""
			for k in range(8):
				l = int(sampPerPeriod*(j*8+k))+i+offset #int(round(sampPerPeriod*(j+0.5)))+i
				timing[l] = dcBias*.1 #-2000
				if(a[l] >=dcBias):
					octet += '0' #.append(0)   #<<= 1 #print '0'
				else:
					octet += '1' #.append(1) #print '1'
			line += octet + '\n' #(octet[::-1] + ' ')
		print line

		#print '---'

		
lower = 0 #12100
upper = length-1 #12700


skip = 0
if len(sys.argv)>2 :
	if sys.argv[2] == "-s": skip = 1
if skip == 0:
	#plt.plot(t[lower:upper],a[lower:upper]*.5,'b',t[lower:upper],diff[lower:upper],'r',t[lower:upper],mark[lower:upper],'y',t[lower:upper],valid[lower:upper],'g')
	plt.plot(a[lower:upper]*.1,'b')
	#plt.plot(diff[lower:upper]*.2,'r')
	#plt.plot(mark[lower:upper],'yo')
	plt.plot(valid[lower:upper],'go')
	plt.plot(timing[lower:upper],'co')

	plt.show() #12k to 12.7k
	
	
	
	#print len(frame)
	# data = struct.unpack('hh',frame)[0]
	# dataa = struct.unpack('hh',frame)[1]
	# print repr(data) + ' and ' + repr(dataa)#int(data[0])
	#print i
	#print a[i]
	#r = a[i]+1
	#print r



	
	
	
	
	
	
	
	
	
	
	
"""
baudrate = 9600
period = 1/9600
sampleDelta = 1/32000

# 11111011010111101000011010101111111011101011101010000000110001001011010000001100110010100000001011101010001011011000110110101000
       | ........| ........| ........| ........| ........| ........| ........| ........| ........| ........| ........| ........| ........| ........

if zero crossed
	delta = now - last
	if state == PREAMBLE
		if .75*period < delta < 1.25*period
			preambleCounter++
		else
			preambleCounter = 0
		if preambleCounter >= preambleThreashold
			state = BLANKING
	else if state == BLANKING
		if 2.75*period < delta < 8.25*period && HIGH_TO_LOW# The blank has been encountered
			preambleCounter = 0
			state = UART_PRESTART
		else if .75*period < delta < 1.25*period # Still in Preamble
			preambleCounter++
		else # encountering invalid frame
			state = PREAMBLE
	else if state == UART_PRESTART
		if .75*period < delta < 1.25*period #&& LOW_TO_HIGH #Has to be LOW_TO_HIGH, previous state was BLANKING w/ HIGH_TO_LOW checked
			state = UART_START
		else
			state = PREAMBLE
	else if state == UART_START
		if .75*period < delta # Stable Start Condition
			state = UART_CAPTURE
		else
			state = PREAMBLE
	else if state == UART_CAPTURE # Every 8 bits, there should be a valid PRESTART and START, otherwise, the frame is over
		deltaAccum += delta
		if deltaAccum >= 7.75*period
		else
			if LOW_TO_HIGH
				shift '1' onto byte for delta%period entries
			# This is getting sticky without any sort of sample timing generator
"""	
	
	
	
	
	
	
	
	
	
"""
	
	
for i in range(0,len(diff)):
	delta = i - last
	if(diff[i] != 0):
		last = i
		if(state == "PREAMBLE"):
			if( (sampPerPeriod*.6 < delta) and (delta < sampPerPeriod*1.4) ):
				mark[i] = 20000
				preambleCounter += 1
				deltaSum += delta
				#print i
			else:
				preambleCounter = 0;
				deltaSum = 0;
				#state = "PREAMBLE"
			if(preambleCounter >= 10):
				#print "Preamble @ " + repr(i)
				#print "Avg Delta: " + repr((deltaSum*1.0)/preambleCounter)
				#valid[i] = 30000
				state = "BLANKING"
		elif(state == "BLANKING"): # SYNC is also a competing state definition
			if( (sampPerPeriod*3.6 < delta) ):
				#valid[i] = 3000
				line = "Valid Porch @ " + repr(i)
				if(i<100000): line += "\t"
				line += "\tAvg Preamble Sa/Period: " + repr((deltaSum*1.0)/preambleCounter)
				print line
				state = "UART_PRESTART"
			elif( (sampPerPeriod*.6 < delta) and (delta < sampPerPeriod*1.4) ):
				
	if(state == "UART_PRESTART"):
	
	
	
	
	
	
	
	
	
	
	
	
	
	
	
	
	
	
	
	
	
	

# 11111011010111101000011010101111111011101011101010000000110001001011010000001100110010100000001011101010001011011000110110101000
       | ........| ........| ........| ........| ........| ........| ........| ........| ........| ........| ........| ........| ........| ........

if zero crossed
	delta = now - last
	if state == PREAMBLE
		if .75*period < delta < 1.25*period
			preambleCounter++
		else
			preambleCounter = 0
		if preambleCounter >= preambleThreashold
			state = BLANKING
	else if state == BLANKING
		if 2.75*period < delta < 8.25*period && HIGH_TO_LOW# The blank has been encountered
			preambleCounter = 0
			state = UART_PRESTART
		else if .75*period < delta < 1.25*period # Still in Preamble
			preambleCounter++
		else # encountering invalid frame
			state = PREAMBLE
	else if state == UART_PRESTART
		if .75*period < delta < 1.25*period #&& LOW_TO_HIGH #Has to be LOW_TO_HIGH, previous state was BLANKING w/ HIGH_TO_LOW checked
			state = UART_START
		else
			state = PREAMBLE
	else if state == UART_START
		if .75*period < delta # Stable Start Condition
			state = UART_CAPTURE
		else
			state = PREAMBLE
	else if state == UART_CAPTURE # Every 8 bits, there should be a valid PRESTART and START, otherwise, the frame is over
		deltaAccum += delta
		if deltaAccum >= 8.25*period
		else
			
			# This is getting sticky without any sort of sample timing generator
		
	
	
	
	
	
	
	
	
	
	
	
	
	
	
	
	
	
	
	
	

if zero crossed
	delta = now - last
	if state = PREAMBLE
		if .75*period < delta < 1.25*period
			preambleCounter++
		else
			preambleCounter = 0
		if preambleCounter >= preambleThreashold
			state = BLANKING
	else if state = BLANKING
		if 2.75*period < delta < 8.25*period # The blank has been encountered
			preambleCounter = 0
			glitchCounter = 0
			state = UART_PRESTART
		else if .75*period < delta < 1.25*period # Still in Preamble, or encountering invalid frame
			preambleCounter++
		else
			glitchCounter++
		if glitchCounter >= glitchLimit
			glitchCounter = 0
			state = PREAMBLE
			
	else if state = UART
		if .75*period < delta < 1.25*80*period
			if low to high
				
			if high to low
				if 
		else
			preambleCounter = 0
			state = PREAMBLE

			
			
			
"""