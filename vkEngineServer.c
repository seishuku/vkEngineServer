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
#include "math/math.h"
#include "utils/list.h"
#include "math/math.h"
#include "network/network.h"
#include "physics/physics.h"
#include "netpacket.h"

MemZone_t *Zone;

#define NUM_ASTEROIDS 1000
RigidBody_t Asteroids[NUM_ASTEROIDS];

uint32_t connectedClients=0;
Client_t Clients[MAX_CLIENTS];

// Current random seed to keep all random numbers on clients the same.
uint32_t CurrentSeed=0;

Socket_t ServerSocket;

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

	// Assign client ID based on first available slot
	for(uint32_t i=0;i<MAX_CLIENTS;i++)
	{
		if(!Clients[i].isConnected)
		{
			newClientID=i;
			break;
		}
	}

	Clients[newClientID].clientID=newClientID;
	Clients[newClientID].Socket=Network_CreateSocket();
	Clients[newClientID].Address=Address;
	Clients[newClientID].Port=Port;
	Clients[newClientID].isConnected=true;
	Clients[newClientID].TTL=GetClock()+30.0;

	// Increment the connected client count
	connectedClients++;

	return newClientID;
}

// Remove client ID from client list
void delClient(uint32_t ID)
{
	if(ID>=MAX_CLIENTS)
		return;

	Network_SocketClose(Clients[ID].Socket);

	memset(&Clients[ID], 0, sizeof(Client_t));
	connectedClients--;
}

void Serialize_uint32(uint8_t **Buffer, uint32_t val)
{
	memcpy(*Buffer, &val, sizeof(uint32_t));
	*Buffer+=sizeof(uint32_t);
}

void Serialize_vec3(uint8_t **Buffer, vec3 val)
{
	memcpy(*Buffer, &val, sizeof(vec3));
	*Buffer+=sizeof(vec3);
}

// Build up random data for skybox and asteroid field
void GenerateSkyParams(void)
{
	// Set up rigid body reps for asteroids
	const float AsteroidFieldMinRadius=50.0f;
	const float AsteroidFieldMaxRadius=1000.0f;
	const float AsteroidMinRadius=0.05f;
	const float AsteroidMaxRadius=40.0f;

	uint32_t i=0, tries=0;

	memset(Asteroids, 0, sizeof(RigidBody_t)*NUM_ASTEROIDS);

	while(i<NUM_ASTEROIDS)
	{
		vec3 RandomDirection=Vec3(RandFloat()*2.0f-1.0f, RandFloat()*2.0f-1.0f, RandFloat()*2.0f-1.0f);
		Vec3_Normalize(&RandomDirection);

		RigidBody_t Asteroid;

		Asteroid.Position=Vec3(
			RandomDirection.x*(RandFloat()*(AsteroidFieldMaxRadius-AsteroidFieldMinRadius))+AsteroidFieldMinRadius,
			RandomDirection.y*(RandFloat()*(AsteroidFieldMaxRadius-AsteroidFieldMinRadius))+AsteroidFieldMinRadius,
			RandomDirection.z*(RandFloat()*(AsteroidFieldMaxRadius-AsteroidFieldMinRadius))+AsteroidFieldMinRadius
		);
		Asteroid.Radius=(RandFloat()*(AsteroidMaxRadius-AsteroidMinRadius))+AsteroidMinRadius;
		Asteroid.Velocity=Vec3b(0.0f);
		Asteroid.Force=Vec3b(0.0f);

		Asteroid.Mass=(1.0f/3000.0f)*(1.33333333f*PI*Asteroid.Radius);
		Asteroid.invMass=1.0f/Asteroid.Mass;

		bool overlapping=false;

		for(uint32_t j=0;j<i;j++)
		{
			if(Vec3_Distance(Asteroid.Position, Asteroids[j].Position)<Asteroid.Radius+Asteroids[j].Radius)
				overlapping=true;
		}

		if(!overlapping)
			Asteroids[i++]=Asteroid;

		tries++;

		if(tries>NUM_ASTEROIDS*NUM_ASTEROIDS)
			break;
	}
	//////
}
//////

double statusSendTime=0.0;
double fieldSendTime=0.0;
double physicsTime=0.0;

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
	CurrentSeed=GetCurrentProcessId();
#else
	// Use process ID for random seed
	CurrentSeed=getpid();
#endif
	DBGPRINTF(DEBUG_INFO, "\033[25;0fAllocating zone memory...\n");
	Zone=Zone_Init(8*1000*1000);

	// Set seed
	srand(CurrentSeed);

	GenerateSkyParams();

	// Clear client list
	memset(&Clients, 0, sizeof(Client_t)*MAX_CLIENTS);

	// Start up network
	Network_Init();

	// Create a new sockets
	ServerSocket=Network_CreateSocket();

	if(ServerSocket==-1)
		return 1;

	// Bind that socket to 0.0.0.0 (any adapter) on port 4545
	if(!Network_SocketBind(ServerSocket, 0, 4545))
		return 1;

	DBGPRINTF(DEBUG_WARNING, "\033[25;0fCurrent seed: %d, waiting for connections...", CurrentSeed);

	// Loop around pulling data that was sent, until you press escape to close.
	bool Done=false;
	while(!Done)
	{
		if(_kbhit())
		{
#ifdef WIN32
			if(_getch()==0x1B)
				Done=true;
#else
			if(getchar()==0x1B)
				Done=true;
#endif
		}

		NetworkPacket_t Packet;

		int32_t length=0;
		uint32_t address=0;
		uint16_t port=0;

		if((length=Network_SocketReceive(ServerSocket, (uint8_t *)&Packet, sizeof(NetworkPacket_t), &address, &port))>0)
		{
			// Handle incoming connections
			if(Packet.PacketMagic==CONNECT_PACKETMAGIC)
			{
				DBGPRINTF(DEBUG_WARNING, "\033[25;0H\033[KConnect from: 0x%X port %d", address, port);

				NetworkPacket_t Response;
				Response.PacketMagic=CONNECT_PACKETMAGIC;
				Response.Connect.Seed=CurrentSeed;
				Response.Connect.Port=port;
				Response.ClientID=addClient(address, port);

				Network_SocketSend(Clients[Response.ClientID].Socket, (uint8_t *)&Response, sizeof(NetworkPacket_t), address, port);
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
					// Copy camera from packet to client's camera.
					Client->Camera.Position=Packet.Camera.Position;
					Client->Camera.Velocity=Packet.Camera.Velocity;
					Client->Camera.Forward=Packet.Camera.Forward;
					Client->Camera.Up=Packet.Camera.Up;

					// Update time to live for client "last time heard" (current time +30 seconds).
					Client->TTL=GetClock()+30.0;
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
		uint8_t StatusBuffer[1024];
		uint8_t FieldBuffer[32767];
		uint8_t *pBuffer;

		const double Sixty=1.0/60.0;
		const double Five=1.0/5.0;

		// Get the current time
		double currentTime=GetClock();

		if(currentTime>statusSendTime)
		{
			statusSendTime=currentTime+Sixty;

			// Clear buffer
			memset(StatusBuffer, 0, sizeof(StatusBuffer));

			pBuffer=StatusBuffer;

			// Serialize data
			Serialize_uint32(&pBuffer, STATUS_PACKETMAGIC); // Magic being sent back to clients is also "status"
			Serialize_uint32(&pBuffer, connectedClients); // Send current connected client count

			for(uint32_t i=0;i<MAX_CLIENTS;i++)
			{
				Client_t *Client=&Clients[i];

				if(Client->isConnected)
				{
					Serialize_uint32(&pBuffer, Client->clientID); // Client's ID
					Serialize_vec3(&pBuffer, Client->Camera.Position); // Client camera position
					Serialize_vec3(&pBuffer, Client->Camera.Velocity); // Client camera velocity
					Serialize_vec3(&pBuffer, Client->Camera.Forward); // Client camera forward (direction)
					Serialize_vec3(&pBuffer, Client->Camera.Up); // Client camera up

					// Report status to console
					DBGPRINTF(DEBUG_WARNING, "\033[%d;0H\033[KStatus from 0x%X:%d (ID %d) pos: %0.1f, %0.1f, %0.1f vel: %0.1f, %0.1f, %0.1f dir: %0.1f %0.1f %0.1f",
							  Client->clientID+1,
							  Client->Address, Client->Port, Client->clientID,
							  Client->Camera.Position.x, Client->Camera.Position.y, Client->Camera.Position.z,
							  Client->Camera.Velocity.x, Client->Camera.Velocity.y, Client->Camera.Velocity.z,
							  Client->Camera.Forward.x, Client->Camera.Forward.y, Client->Camera.Forward.z
					);

					// If current time has past last hard time, then client has timed out... So remove it.
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
					Network_SocketSend(Clients[i].Socket, StatusBuffer, sizeof(StatusBuffer), Clients[i].Address, Clients[i].Port);
			}
		}

		// Update the whole asteroid field at 60FPS? Probably a bad idea, works on loopback network at least.
		if(currentTime>fieldSendTime)
		{
			fieldSendTime=currentTime+Sixty;

			uint32_t asteroidCount=NUM_ASTEROIDS;
			uint32_t Magic=FIELD_PACKETMAGIC;

			memset(FieldBuffer, 0, sizeof(FieldBuffer));

			pBuffer=FieldBuffer;
			Serialize_uint32(&pBuffer, Magic);
			Serialize_uint32(&pBuffer, asteroidCount);

			for(uint32_t i=0;i<asteroidCount;i++)
			{
				Serialize_vec3(&pBuffer, Asteroids[i].Position);
				Serialize_vec3(&pBuffer, Asteroids[i].Velocity);
			}

			for(uint32_t i=0;i<MAX_CLIENTS;i++)
			{
				if(Clients[i].isConnected)
					Network_SocketSend(Clients[i].Socket, FieldBuffer, sizeof(FieldBuffer), Clients[i].Address, Clients[i].Port);
			}
		}

		// Run physics stuff
		{
			const float dt=(float)Sixty;//fTimeStep;

			// If current time has elapsed last set time, then run code
			if(currentTime>physicsTime)
			{
				// reset time to current time + time until next run
				physicsTime=currentTime+Sixty;

				// Get a pointer to the emitter that's providing the positions
				//ParticleEmitter_t *Emitter=List_GetPointer(&ParticleSystem.Emitters, 0);

				//for(uint32_t i=0;i<Emitter->NumParticles;i++)
				//{
				//	if(Emitter->Particles[i].ID!=Emitter->ID)
				//	{
				//		// Get those positions and set the other emitter's positions to those
				//		ParticleSystem_SetEmitterPosition(&ParticleSystem, Emitter->Particles[i].ID, Emitter->Particles[i].pos);

				//		// If this particle is dead, delete that emitter and reset it's ID 
				//		if(Emitter->Particles[i].life<0.0f)
				//		{
				//			ParticleSystem_DeleteEmitter(&ParticleSystem, Emitter->Particles[i].ID);
				//			Emitter->Particles[i].ID=0;
				//		}
				//	}
				//}

				//ParticleSystem_Step(&ParticleSystem, dt);

				// Loop through objects, integrate and check/resolve collisions
				for(int i=0;i<NUM_ASTEROIDS;i++)
				{
					// Run physics integration on the asteroids
					PhysicsIntegrate(&Asteroids[i], dt);

					// Check asteroids against other asteroids
					for(int j=i+1;j<NUM_ASTEROIDS;j++)
						PhysicsSphereToSphereCollisionResponse(&Asteroids[i], &Asteroids[j]);

					for(uint32_t j=0;j<connectedClients;j++)
							PhysicsCameraToSphereCollisionResponse(&Clients[j].Camera, &Asteroids[i]);

					// Check asteroids against projectile particles
					// Emitter '0' on the particle system contains particles that drive the projectile physics
					//ParticleEmitter_t *Emitter=List_GetPointer(&ParticleSystem.Emitters, 0);
					// Loop through all the possible particles
					//for(uint32_t j=0;j<Emitter->NumParticles;j++)
					//{
					//	// If the particle ID matches with the projectile ID, then check collision and respond
					//	if(Emitter->Particles[j].ID!=Emitter->ID)
					//		PhysicsParticleToSphereCollisionResponse(&Emitter->Particles[j], &Asteroids[i]);
					//}
				}
				//////
			}
		}
	}

	// Done, close sockets and shutdown
	Network_SocketClose(ServerSocket);
	Network_Destroy();

	return 0;
}