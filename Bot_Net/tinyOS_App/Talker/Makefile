COMPONENT=TalkerAppC
BUILD_EXTRA_DEPS = Bot_NetMsg.py
CLEAN_EXTRA = Bot_NetMsg.py Bot_NetMsg.pyc

Bot_NetMsg.py: Bot_Net.h
	@echo -e 'Python Class making....'
	mig python -target=$(PLATFORM) $(CFLAGS) -python-classname=Bot_NetMsg Bot_Net.h bot_net_msg -o $@

FORCE: Bot_NetMsg.py
	mv Bot_NetMsg.py ..

#CFLAGS += -DCC2420_NO_ACKNOWLEDGEMENTS
CFLAGS += -DCC2420_NO_ADDRESS_RECOGNITION
CFLAGS += -DTASKLET_IS_TASK

include $(MAKERULES)
