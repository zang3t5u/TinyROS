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
		test = subprocess.Popen(['motelist| grep "/dev/ttyUSB"'], stdout=subprocess.PIPE, shell = True)
		output = test.communicate()[0].split("\n")
		base_index = output[0].find("/dev/ttyUSB")
		dev = output[0][base_index:base_index+12]
		
		arg = 'serial@'+dev+':115200'
	else:
		arg = sys.argv[3]
	print "------------------"
	print "Serial Started at:"+arg
	botID = int(sys.argv[2])
	print "botID      : ", botID
	print "Total Bots : ", Num_of_Bots
	Num_of_Bots = int(sys.argv[1])
	dl = Bot_Net.Bot_Net(arg, Num_of_Bots, botID)
	count = 0;
	sys.stdout.flush()
	while(1):
		time.sleep((1))
		if(botID!=0):
			dl.send_msg(botID, 0, 1, [botID*8000., botID*1000., math.pi])
		else:
			send_to = (count%Num_of_Bots)+1
			print "Sending to ", send_to
			dl.send_msg(botID, send_to, 1, [send_to*500., send_to*400., math.pi])
		#dl.display_database()
		count+=1
	

if __name__ == "__main__":
	try:
		main()
	except KeyboardInterrupt:
		pass
