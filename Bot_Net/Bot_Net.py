import os
import sys
import time
import struct

#tos stuff
import Bot_NetMsg
from tinyos.message import *
from tinyos.message.Message import *
from tinyos.message.SerialPacket import *
from tinyos.packet.Serial import Serial

#Total TimeInterval for 1 bot in TDMA
t_interval = 2
NBots = 1
uartBusy = False

class Bot_Net:
    def __init__(self, motestring, Num_of_Bots, botID):
    	self.counter = 0
    	self.mif = MoteIF.MoteIF()
    	self.tos_source = self.mif.addSource(motestring)
    	self.mif.addListener(self, Bot_NetMsg.Bot_NetMsg)
    	#For TimeSync
    	NBots = Num_of_Bots
    	self.start = False
    	print self.start
    	self.start_time = time.time()
    	self.last_time = self.start_time
    	self.timeOffset = botID
    	print self.timeOffset

    def receive(self, src, msg):
    	#List of data to be returned
    	abstracted_data = []
    	if msg.get_amType() == Bot_NetMsg.AM_TYPE:
    		#If first message received, start timer
    		if(not self.start):
    			self.start_time = time.time() - self.timeOffset
    			self.last_time = self.start_time
    			self.start = True

    		#Get Msg Source
    		msg_source = msg.getAddr()
    		
    		m = Bot_NetMsg.Bot_NetMsg(msg.dataGet())
    		#Msg sent by Central Comp
    		if(msg_source == 0):
    			#Data in message describes the recipient
    			msg_bot = m.get_rcv_Rob_ID()
    		
    		#Msg sent by Bot
    		else:
    			#Data in message describes the sender
    			msg_bot = msg_source
  			
  			#Extend list to include information
  			abstracted_data.extend([float(m.get_tx_timestamp())])
  			abstracted_data.extend([float(m.get_seqNo())])
  			abstracted_data.extend([msg_bot])
  			abstracted_data.extend([m.get_dataType()])
  			abstracted_data.extend([m.get_data()])
  			'''  			
  			#Print information
  			print "Received", msg
  			sys.stdout.flush()
  			'''
			#Return
			#Print information
			print "Received ", abstracted_data
			sys.stdout.flush()
			return abstracted_data

    def send_msg(self, recv_Rob_ID, dataType, data):
    	global uartBusy
    	if((self.start or (self.timeOffset==1)) and not uartBusy):
    		uartBusy = True
    		print "Sending packet ", self.counter
    		smsg = Bot_NetMsg.Bot_NetMsg()
    		smsg.set_seqNo(self.counter)
    		smsg.set_recv_Rob_ID(recv_Rob_ID)
    		smsg.set_dataType(dataType)
    		if(len(data)<3):
    			data.extend(0)
    		smsg.set_data(data)
    		
    		#Wait for timeslot
    		t = (time.time()-self.start_time)
    		while(t%(NBots*t_interval) != 0):
    			t = (time.time()-self.last_time)
    		self.last_time += t;
    		smsg.set_tx_timestamp(t+self.last_time-self.start_time)
    		self.mif.sendMsg(self.tos_source, 0xFFFF, smsg.get_amType(), 0, smsg)
    		uartBusy = False
    		self.counter+=1
    		self.counter%=256
'''
    def main_loop(self):
    	while 1:
    		time.sleep(1)
    		# send a message 1's per second
    		self.send_msg()
'''
'''
def main():
	if '-h' in sys.argv or len(sys.argv) < 2:
		print "Usage:", sys.argv[0], "sf@localhost:9002", "adc_rev_volt <Volt>" 
		sys.exit()
		
	dl = Bot_Net(sys.argv[1], float(sys.argv[2]))
	dl.main_loop()  # don't expect this to return...

if __name__ == "__main__":
	try:
		main()
	except KeyboardInterrupt:
		pass
'''
