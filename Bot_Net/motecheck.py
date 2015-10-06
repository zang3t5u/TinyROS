import sys
import os
import subprocess

def main():
	test = subprocess.Popen(["motelist"], stdout=subprocess.PIPE)
	output = test.communicate()[0].split()
	print output[7]
	#Ignore description and other lines
	'''
	for i in range(0,3):
		s = p.readline()
	p.close()
	print s[2]
	'''

if __name__ == "__main__":
	try:
		main()
	except KeyboardInterrupt:
		pass
