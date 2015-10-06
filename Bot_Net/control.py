import os
import sys
import subprocess
import time
import struct
import math

#TOS Stuff
import Bot_Net

def main():
	if '-h' in sys.argv or len(sys.argv) < 3:
		print "Usage:", sys.argv[0], "Num_of_Bots", "BotID" , "sf@localhost:9002"
		sys.exit()
	if(len(sys.argv) < 4):
		test = subprocess.Popen(["motelist"], stdout=subprocess.PIPE)
		output = test.communicate()[0].split()
		arg = 'serial@'+output[7]+':115200'
	else:
		arg = sys.argv[3]
	print "------------------"
	print "Serial Started at:"+arg
	botID = int(sys.argv[2])
	Num_of_Bots = int(sys.argv[1])
	dl = Bot_Net.Bot_Net(arg, Num_of_Bots, botID)
	count = 0;
	
	while(1):
		time.sleep(1)
		dl.send_msg(count, botID-1, [500., 400., math.pi])
		count+=1

if __name__ == "__main__":
	try:
		main()
	except KeyboardInterrupt:
		pass
