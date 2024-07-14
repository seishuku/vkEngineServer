#ifndef __NETPACKET_H__
#define __NETPACKET_H__

#include <stdint.h>
#include <stdbool.h>
#include "math/math.h"
#include "camera/camera.h"
#include "network/network.h"

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
	vec3 position, velocity;
	vec4 orientation;
} NetCamera_t;

// Connect data when connecting to server
typedef struct
{
	uint32_t seed;
	uint16_t port;
} NetConnect_t;

// Overall data network packet
typedef struct
{
	uint32_t packetMagic;
	uint32_t clientID;
	union
	{
		NetConnect_t connect;
		NetCamera_t camera;
	};
} NetworkPacket_t;

typedef struct
{
	uint32_t clientID;
	Socket_t socket;
	uint32_t address;
	uint16_t port;
	bool isConnected;
	double TTL;

	Camera_t camera;
} Client_t;

#endif
