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

// Status buffer:
// Magic = 4 bytes
// connected clients = 4 bytes (could be 1 byte)
// (potentionally) 16x:
//		clientID = 4 bytes
//		camera position = 12 bytes
//		camera velocity = 12 bytes
//		camera orientation = 16 bytes
//
// Worst case is 712 bytes being sent to all clients.

// Field buffer:
// Magic = 4 bytes
// asteroid count = 4 bytes
// (potentionally) 1000x:
//		asteroid position = 12 bytes
//		asteroid velocity = 12 bytes
//		asteroid orientation = 16 bytes
//		asteroid radius = 4 bytes
//
// 44008 bytes sent to all connected clients

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
