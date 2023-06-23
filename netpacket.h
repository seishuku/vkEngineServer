#ifndef __NETPACKET_H__
#define __NETPACKET_H__

// Packet magic uint32's
#define CONNECT_PACKETMAGIC		('C'|('o'<<8)|('n'<<16)|('n'<<24)) // "Conn"
#define DISCONNECT_PACKETMAGIC	('D'|('i'<<8)|('s'<<16)|('C'<<24)) // "DisC"
#define STATUS_PACKETMAGIC		('S'|('t'<<8)|('a'<<16)|('t'<<24)) // "Stat"
#define FIELD_PACKETMAGIC		('F'|('e'<<8)|('l'<<16)|('d'<<24)) // "Feld"

// Max number of clients
#define MAX_CLIENTS 16

// PacketMagic determines packet type:
//
// Connect:
//		Client sends connect magic, server responds back with current random seed and slot.
// Disconnect:
//		Client sends disconnect magic, server closes socket and removes client from list.
// Status:
//		Client to server: Sends current camera data
//		Server to client: Sends all current connected client cameras.
// Field:
//		Server sends current play field (as it sees it) to all connected clients at a regular interval.

// Camera data for sending over the network
typedef struct
{
	vec3 Position, Velocity;
	vec3 Forward, Up;
} NetCamera_t;

// Connect data when connecting to server
typedef struct
{
	uint32_t Seed;
	uint16_t Port;
} NetConnect_t;

// Overall data network packet
typedef struct
{
	uint32_t PacketMagic;
	uint32_t ClientID;
	union
	{
		NetConnect_t Connect;
		NetCamera_t Camera;
	};
} NetworkPacket_t;

typedef struct
{
	uint32_t clientID;
	Socket_t Socket;
	uint32_t Address;
	uint16_t Port;
	bool isConnected;
	double TTL;

	Camera_t Camera;
} Client_t;

#endif
