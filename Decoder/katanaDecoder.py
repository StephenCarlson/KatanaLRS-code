import wave, struct
from array import *
import matplotlib.pyplot as plt
from pylab import *
import sys, getopt, os.path #time

#print "Program Start"

file_name = ""
graph = 0
doAvg = 0
filterOrder = 5 #0
baudrate = 4800.0 #9600.0 #9143.0 # Increase Slightly Some more 9183.0 #
offset = 22 #11 # (-1)
bytes = 80 #12 for 7Ch
preambleBits = 10

def printHelp():
	print "KatanaLRS Decoder by Stephen Carlson Jan 2014\n \
	-h	Help \n \
	-g	Graph \n \
	-a	Work on Average of all valid frames (must have multiple packets)\n \
	-i	<inputfile>        (Default: Newest \"_AF.wav\") \n \
	-f	<Filter Order>     (Default: 5)   \n \
	-b	<Baudrate>         (Default: 4800)\n \
	-B	<Bytes in Payload> (Default: 80)  \n \
	-o	<Preamble Offset>  (Default: 22)  \n \
	-p	<Preamble Bits>    (Default: 10)"

try:
	opts, args = getopt.getopt(sys.argv[1:],"higaf:b:o:B:p:",["ifile=","help"])
except getopt.GetoptError:
	printHelp()
	print "\nError with command line arguments\n"
	sys.exit(2)

for opt, arg in opts:
	if opt in ("-h", "--help"):
		printHelp()
		sys.exit()
	if opt in ("-g"):
		graph = 1
	if opt in ("-a"):
		doAvg = 1
	if opt in ("-i", "--ifile"):
		file_name = arg
	if opt in ("-f"):
		filterOrder = int(arg)
	if opt in ("-b"):
		baudrate = int(arg) * 1.0
	if opt in ("-B"):
		bytes = int(arg)
	if opt in ("-o"):
		offset = int(arg)
	if opt in ("-p"):
		preambleBits = int(arg)
	
if file_name != "":
	file_name = file_name + ".wav"
else:
	recent = 0.0
	for file in os.listdir(os.getcwd()):
		if file.endswith("_AF.wav") and (os.path.getsize(file) < 900000) :
			if(os.path.getmtime(file) > recent): 
				file_name = file
				recent = os.path.getmtime(file)
			#print file, os.path.getmtime(file),os.path.getsize(file)
print '\nUsing',file_name
#print file_name + ".wav"

w = wave.open(file_name,'r')

nchannels = w.getnchannels()
samplewid = w.getsampwidth()
framerate = w.getframerate()
nframes   = w.getnframes()

# print "Channels: " + repr(w.getnchannels())
# print "Width:    " + repr(w.getsampwidth())
# print "Rate:     " + repr(w.getframerate())
# print ""

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
	
if(int(filterOrder) > 0): # Moving into the Realm of Windowing Functions and Signal Processing
	for i in range(len(a)-int(filterOrder)): # Moving Average, no Weights
		for j in range(int(filterOrder)):
			a[i] += a[i+j+1]
		a[i]/(1.0+int(filterOrder))

period = 1.0/baudrate
sampleDelta = 1.0/framerate

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
			elif( (sampPerPeriod*3.6 < delta) and preambleCounter >= preambleBits and diff[i] < 0):
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

# offset = 22 #11 # (-1)
# bytes = 80 #12 for 7Ch

byteArray = [] #zeros(bytes,dtype=numpy.int) #zeros(bytes,Int) #array('B') #[] #zeros(bytes) #[bytes+1]
for i in range(bytes):
	byteArray.append(0)

#print "Avg Buffer:",int(bytes*sampPerPeriod*8)
avgArray = zeros(int(bytes*sampPerPeriod*8))

validCount = 0;
for i in range(0,len(valid)):
	if(valid[i] != annoyingOffset): # Yes, yes, I know this is like O(n^3) or something like that, need to optimize	
		for l in range(bytes):
			byteArray[l] = 0;
		
		#  line = "" #[]
		if bytes*sampPerPeriod*10+i >len(valid):
			break
		
		validCount += 1
		for s in range(len(avgArray)):
			avgArray[s] += a[i+s]
		
		for j in range(bytes):
			#   octet = ""
			for k in range(8):
				l = int(sampPerPeriod*(j*8+k))+i+offset #int(round(sampPerPeriod*(j+0.5)))+i
				timing[l] = dcBias*.1 #-2000
				if(a[l] >=dcBias):
					#   octet += '0' #.append(0)   #<<= 1 #print '0'
					byteArray[j] = (byteArray[j])<<1 | 0x00
				else:
					#   octet += '1' #.append(1) #print '1'
					byteArray[j] = (byteArray[j])<<1 | 0x01
			#   line += octet + '\n' #(octet[::-1] + ' ')
		#print line

		#print '---'

		str = ">\t" #"\n>\t"
		for i in range(bytes):
			if(byteArray[i] in range(0x20, 0x7E)): #== '*'):
				if(chr(byteArray[i]) == '*'): 
					str += "*" #\n"
					break
				else: str += chr(byteArray[i]) #repr(bin(byteArray))
		print str
	
#print "\nValid Frames:",validCount
if (validCount > 0): 
	if(doAvg): timing = zeros(len(avgArray))+annoyingOffset
	for s in range(len(avgArray)):
		avgArray[s] = avgArray[s] / validCount
	# Ok, with the code about to be pasted and modified below, I have officially crossed the line, I need to do lots of def modules
	for i in range(len(avgArray)-int(filterOrder)): # Moving Average, no Weights
		for j in range(int(filterOrder)):
			avgArray[i] += avgArray[i+j+1]
		avgArray[i]/(1.0+int(filterOrder))
	
	for l in range(bytes):
		byteArray[l] = 0;
	for j in range(bytes):
		for k in range(8):
			l = int(sampPerPeriod*(j*8+k))+offset
			if(l<len(avgArray)):
				if(doAvg): timing[l] = dcBias
				if(avgArray[l] >=dcBias):
					byteArray[j] = (byteArray[j])<<1 | 0x00
				else:
					byteArray[j] = (byteArray[j])<<1 | 0x01

	str = "\n>+++\t"
	for i in range(bytes):
		if(byteArray[i] in range(0x20, 0x7E)):
			if(chr(byteArray[i]) == '*'): 
				str += "*"
				break
			else: str += chr(byteArray[i]) #repr(bin(byteArray))
	print str

lower = 0 #12100
upper = length-1 #12700


if graph == 1:
	#plt.plot(t[lower:upper],a[lower:upper]*.5,'b',t[lower:upper],diff[lower:upper],'r',t[lower:upper],mark[lower:upper],'y',t[lower:upper],valid[lower:upper],'g')
	plt.plot(a[lower:upper]*.1,'b')
	#plt.plot(diff[lower:upper]*.2,'r')
	#plt.plot(mark[lower:upper],'yo')
	plt.plot(valid[lower:upper],'go')
	plt.plot(timing[lower:upper],'co')
	plt.show()
elif doAvg == 1:
	plt.plot(avgArray[0:int(len(avgArray))]*.01,'b')
	plt.plot(timing[0:int(len(avgArray))],'co')
	plt.show()
	
	
	
	#print len(frame)
	# data = struct.unpack('hh',frame)[0]
	# dataa = struct.unpack('hh',frame)[1]
	# print repr(data) + ' and ' + repr(dataa)#int(data[0])
	#print i
	#print a[i]
	#r = a[i]+1
	#print r



	
	
