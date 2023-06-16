#ifdef WIN32
#include <Windows.h>
#include <conio.h>
#else
#include <unistd.h>
#include <sys/time.h>
#include <time.h>
#include <fcntl.h>
#include <termios.h>
#endif
#include <stdlib.h>
#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include "system/system.h"
#include "utils/list.h"
#include "network/network.h"

#define CONNECT_PACKETMAGIC		('C'|('O'<<8)|('N'<<16)|('N'<<24))
#define DISCONNECT_PACKETMAGIC	('D'|('I'<<8)|('S'<<16)|('C'<<24))
#define STATUS_PACKETMAGIC		('S'|('t'<<8)|('A'<<16)|('t'<<24))

// PacketMagic determines packet type
// Connect: Client sends packet with CONNECT magic, slot/seed 0, address/port is server's address and port,
//				server sends back address/port, current random seed and slot.
// Disconnect: Client sends packet with DISCONNECT magic, server just closes socket and client shuts down.
// Status: Client sends packet with STATUS magic, server sends back all other client statuses?

// Camera data for sending over the network
typedef struct
{
	struct { float x, y, z; } Position;
	struct { float x, y, z; } Velocity;
	struct { float x, y, z; } Forward, Up;
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
	uint32_t Address;
	uint16_t Port;
	bool isConnected;
	double TTL;

	NetCamera_t Camera;
} Client_t;

#define MAX_CLIENTS 16

Client_t Clients[MAX_CLIENTS];
uint32_t connectedClients=0;

Socket_t SendSocket;
Socket_t RecvSocket;
uint32_t seed=0;

// Get current time with best possible precision
double GetClock(void)
{
#ifdef WIN32
	static uint64_t Frequency=0;
	uint64_t Count;

	if(!Frequency)
		QueryPerformanceFrequency((LARGE_INTEGER *)&Frequency);

	QueryPerformanceCounter((LARGE_INTEGER *)&Count);

	return (double)Count/Frequency;
#else
	struct timespec ts;

	if(!clock_gettime(CLOCK_MONOTONIC, &ts))
		return ts.tv_sec+(double)ts.tv_nsec/1000000000.0;

	return 0.0;
#endif
}

#ifndef WIN32
// kbhit for *nix
int _kbhit(void)
{
	struct termios oldt, newt;
	int ch;
	int oldf;

	tcgetattr(STDIN_FILENO, &oldt);
	newt=oldt;
	newt.c_lflag&=~(ICANON|ECHO);
	tcsetattr(STDIN_FILENO, TCSANOW, &newt);
	oldf=fcntl(STDIN_FILENO, F_GETFL, 0);
	fcntl(STDIN_FILENO, F_SETFL, oldf|O_NONBLOCK);

	ch=getchar();

	tcsetattr(STDIN_FILENO, TCSANOW, &oldt);
	fcntl(STDIN_FILENO, F_SETFL, oldf);

	if(ch!=EOF)
	{
		ungetc(ch, stdin);
		return 1;
	}

	return 0;
}
#endif

// Add address/port to client list, return clientID
uint32_t addClient(uint32_t Address, uint16_t Port)
{
	// Lobby is full, can't add more clients
	if(connectedClients>=MAX_CLIENTS)
		return false;

	int newClientID=0;

	// Assign client ID based on the connected client count
	for(uint32_t i=0;i<MAX_CLIENTS;i++)
	{
		if(!Clients[i].isConnected)
		{
			newClientID=i;
			break;
		}
	}

	Clients[newClientID].clientID=newClientID;
	Clients[newClientID].Address=Address;
	Clients[newClientID].Port=Port;
	Clients[newClientID].isConnected=true;
	Clients[newClientID].TTL=UINT16_MAX*2;

	// Increment the connected client count
	connectedClients++;

	return newClientID;
}

// Remove client ID from client list
void delClient(uint32_t ID)
{
	if(ID>=MAX_CLIENTS)
		return;

	memset(&Clients[ID], 0, sizeof(Client_t));
	connectedClients--;
}

int main(int argc, char **argv)
{
#ifdef WIN32
	// Set windows console stuff for escape charactors
	HINSTANCE hInstance=GetModuleHandle(NULL);
	HANDLE hOutput=GetStdHandle(STD_OUTPUT_HANDLE);
	DWORD dwMode;

	GetConsoleMode(hOutput, &dwMode);
	SetConsoleMode(hOutput, dwMode|ENABLE_PROCESSED_OUTPUT|ENABLE_VIRTUAL_TERMINAL_PROCESSING);

	// Use process ID for random seed
	seed=GetCurrentProcessId();
#else
	// Use process ID for random seed
	seed=getpid();
#endif

	// Set seed
	srand(seed);

	// Clear client list
	memset(&Clients, 0, sizeof(Client_t)*MAX_CLIENTS);

	// Start up network
	Network_Init();

	// Create a new sockets
	SendSocket=Network_CreateSocket();
	RecvSocket=Network_CreateSocket();

	if(SendSocket==-1||RecvSocket==-1)
		return 1;

	// Bind that socket to 0.0.0.0 (any adapter) on port 4545
	if(!Network_SocketBind(RecvSocket, 0, 4545))
		return 1;

	DBGPRINTF(DEBUG_WARNING, "\033[25;0fCurrent seed: %d, waiting for connections...", seed);

	// Loop around pulling data that was sent, until you press a key to close (need to do something else for that)
	while(!_kbhit())
	{
		NetworkPacket_t Packet;

		int32_t length=0;
		uint32_t address=0;
		uint16_t port=0;

		if((length=Network_SocketReceive(RecvSocket, (uint8_t *)&Packet, sizeof(NetworkPacket_t), &address, &port))>0)
		{
			// Handle incoming connections
			if(Packet.PacketMagic==CONNECT_PACKETMAGIC)
			{
				DBGPRINTF(DEBUG_WARNING, "\033[25;0H\033[KConnect from: 0x%X port %d", address, port);

				NetworkPacket_t Response;
				Response.PacketMagic=CONNECT_PACKETMAGIC;
				Response.Connect.Seed=seed;
				Response.Connect.Port=port;
				Response.ClientID=addClient(address, port);

				Network_SocketSend(SendSocket, (uint8_t *)&Response, sizeof(NetworkPacket_t), address, port);
			}
			// Handle disconnections
			else if(Packet.PacketMagic==DISCONNECT_PACKETMAGIC)
			{
				DBGPRINTF(DEBUG_WARNING, "\033[%d;0H\033[KDisconnect from: #%d 0x%X:%d",
						  Packet.ClientID+1,
						  Packet.ClientID,
						  address,
						  port
				);
				delClient(Packet.ClientID);
			}
			// Handle status reports
			else if(Packet.PacketMagic==STATUS_PACKETMAGIC)
			{
				Client_t *Client=&Clients[Packet.ClientID];

				if(Client->isConnected)
				{
					// Copy camera to client's camera struct
					memcpy(&Client->Camera, &Packet.Camera, sizeof(Client->Camera));
					// Update time to live for sudden disconnect timeout (current time +5 seconds)
					Client->TTL=GetClock()+5.0;
				}
			}
		}

		// 1KB buffer should be enough for now
		// Magic = 4 bytes
		// connectedClients = 4 bytes (could be 1 byte)
		// 16x:
		//		clientID = 4 bytes
		//		Camera position = 12 bytes
		//		Camera velocity = 12 bytes
		//		Camera forward = 12 bytes
		//		Camera up = 12 bytes
		//
		// Worst case is 840 bytes being sent to all clients, not terrible.
		uint8_t Buffer[1024], *pBuffer=Buffer;

		// Magic being sent back to clients is also "status"
		uint32_t Magic=STATUS_PACKETMAGIC;

		// Clear buffer
		memset(Buffer, 0, sizeof(Buffer));

		// Serialize data
		memcpy(pBuffer, &Magic, sizeof(uint32_t));						pBuffer+=sizeof(uint32_t); // Packet magic
		memcpy(pBuffer, &connectedClients, sizeof(uint32_t));			pBuffer+=sizeof(uint32_t); // Number of connected clients

		for(uint32_t i=0;i<MAX_CLIENTS;i++)
		{
			Client_t *Client=&Clients[i];

			if(Client->isConnected)
			{
				memcpy(pBuffer, &Client->clientID, sizeof(uint32_t));			pBuffer+=sizeof(uint32_t); // Client ID
				memcpy(pBuffer, &Client->Camera.Position, sizeof(float)*3);		pBuffer+=sizeof(float)*3; // Camera position
				memcpy(pBuffer, &Client->Camera.Velocity, sizeof(float)*3);		pBuffer+=sizeof(float)*3; // Camera velocity
				memcpy(pBuffer, &Client->Camera.Forward, sizeof(float)*3);		pBuffer+=sizeof(float)*3; // Camera forward (direction)
				memcpy(pBuffer, &Client->Camera.Up, sizeof(float)*3);			pBuffer+=sizeof(float)*3; // Camera up

				// Report status to console
				DBGPRINTF(DEBUG_WARNING, "\033[%d;0H\033[KStatus from 0x%X:%d (ID %d) pos: %0.1f, %0.1f, %0.1f vel: %0.1f, %0.1f, %0.1f dir: %0.1f %0.1f %0.1f",
						  Client->clientID+1,
						  Client->Address, Client->Port, Client->clientID,
						  Client->Camera.Position.x, Client->Camera.Position.y, Client->Camera.Position.z,
						  Client->Camera.Velocity.x, Client->Camera.Velocity.y, Client->Camera.Velocity.z,
						  Client->Camera.Forward.x, Client->Camera.Forward.y, Client->Camera.Forward.z
				);

				// If current time has gone past last hard time+5 seconds, then client has timed out... So remove it.
				if(GetClock()>Client->TTL)
				{
					DBGPRINTF(DEBUG_WARNING, "\033[%d;0H\033[KDisconnected - Timed out.", Client->clientID+1);
					delClient(Client->clientID);
				}
			}
		}

		// Blast collected connected client data back to all connected clients
		for(uint32_t i=0;i<MAX_CLIENTS;i++)
		{
			if(Clients[i].isConnected)
				Network_SocketSend(SendSocket, Buffer, sizeof(Buffer), Clients[i].Address, Clients[i].Port);
		}
	}

	// Done, close sockets and shutdown
	Network_SocketClose(SendSocket);
	Network_SocketClose(RecvSocket);
	Network_Destroy();

	return 0;
}