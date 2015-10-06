#ifndef BOT__NET_H
#define BOT__NET_H

enum {
  AM_BOT_NET_MSG = 0x89,
  AM_RADIO = 6
};

typedef nx_struct bot_net_msg 
{
	nx_uint32_t tx_timestamp;
	nx_uint8_t seqNo;
	nx_uint16_t recv_Rob_ID;
	/*
	 * 1.If Sender is Central Comp 	==> source == 0
	 * 		<Rob_poses> and <Vel_poses> are those of recv_Rob_ID
	 * 		i.e Published on create<recv_Rob_ID>
	 * 2.If Sender is Robot			==> source != 0
	 * 		<Rob_poses> and <Vel_poses> are those of sendID
	 * 		i.e Published on create<source>
	*/ 
	nx_uint8_t dataType;
	/*
	 * Type			Data			Size			Decode Index
	 * ----			----			----			------------
	 * 1			Pose			3 doubles		0 - x, 1 - y, 2 - theta
	 * 2			Vel				3 doubles		0 - x, 1 - y, 2 - theta
	 * 3			Neighbours		2 int			0 - Neighbour1, 1 - Neighbour2
	 * 4			Leader			3 doubles		0 - x, 1 - y, 2 - theta
	 * */
	nx_float data[3];
} bot_net_msg_t;

#endif /* BOT__NET_H */
