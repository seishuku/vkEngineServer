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

MemZone_t *zone;

#define NUM_ASTEROIDS 1000
RigidBody_t asteroids[NUM_ASTEROIDS];

uint32_t connectedClients=0;
Client_t clients[MAX_CLIENTS];

// Current random seed to keep all random numbers on clients the same.
uint32_t currentSeed=0;

Socket_t serverSocket;

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
uint32_t addClient(uint32_t address, uint16_t port)
{
	// Lobby is full, can't add more clients
	if(connectedClients>=MAX_CLIENTS)
		return false;

	int newClientID=0;

	// Assign client ID based on first available slot
	for(uint32_t i=0;i<MAX_CLIENTS;i++)
	{
		if(!clients[i].isConnected)
		{
			newClientID=i;
			break;
		}
	}

	clients[newClientID].clientID=newClientID;
	clients[newClientID].socket=Network_CreateSocket();
	clients[newClientID].address=address;
	clients[newClientID].port=port;
	clients[newClientID].isConnected=true;
	clients[newClientID].TTL=GetClock()+30.0;

	// Increment the connected client count
	connectedClients++;

	return newClientID;
}

// Remove client ID from client list
void delClient(uint32_t ID)
{
	if(ID>=MAX_CLIENTS)
		return;

	Network_SocketClose(clients[ID].socket);

	memset(&clients[ID], 0, sizeof(Client_t));
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

void Serialize_vec4(uint8_t **Buffer, vec4 val)
{
	memcpy(*Buffer, &val, sizeof(vec4));
	*Buffer+=sizeof(vec4);
}

// Build up random data for skybox and asteroid field
void GenerateWorld(void)
{
	// Set up rigid body reps for asteroids
	const float asteroidFieldMinRadius=50.0f;
	const float asteroidFieldMaxRadius=1000.0f;
	const float asteroidMinRadius=0.05f;
	const float asteroidMaxRadius=40.0f;

	uint32_t i=0, tries=0;

	memset(asteroids, 0, sizeof(RigidBody_t)*NUM_ASTEROIDS);

	// Randomly place asteroids in a sphere without any otherlapping.
	while(i<NUM_ASTEROIDS)
	{
		vec3 randomDirection=Vec3(RandFloat()*2.0f-1.0f, RandFloat()*2.0f-1.0f, RandFloat()*2.0f-1.0f);
		Vec3_Normalize(&randomDirection);

		RigidBody_t asteroid={ 0 };

		asteroid.position=Vec3(
			randomDirection.x*RandFloatRange(asteroidFieldMinRadius, asteroidFieldMaxRadius),
			randomDirection.y*RandFloatRange(asteroidFieldMinRadius, asteroidFieldMaxRadius),
			randomDirection.z*RandFloatRange(asteroidFieldMinRadius, asteroidFieldMaxRadius)
		);
		asteroid.radius=RandFloatRange(asteroidMinRadius, asteroidMaxRadius);

		bool overlapping=false;

		for(uint32_t j=0;j<i;j++)
		{
			if(Vec3_Distance(asteroid.position, asteroids[j].position)<asteroid.radius+asteroids[j].radius)
				overlapping=true;
		}

		if(!overlapping)
			asteroids[i++]=asteroid;

		tries++;

		if(tries>NUM_ASTEROIDS*NUM_ASTEROIDS)
			break;
	}
	//////

	for(uint32_t i=0;i<NUM_ASTEROIDS;i++)
	{
		vec3 randomDirection=Vec3(
			RandFloatRange(-1.0f, 1.0f),
			RandFloatRange(-1.0f, 1.0f),
			RandFloatRange(-1.0f, 1.0f)
		);
		Vec3_Normalize(&randomDirection);

		asteroids[i].velocity=Vec3_Muls(randomDirection, RandFloat());
		asteroids[i].force=Vec3b(0.0f);

		asteroids[i].orientation=Vec4(0.0f, 0.0f, 0.0f, 1.0f);
		asteroids[i].angularVelocity=Vec3_Muls(randomDirection, RandFloat());

		asteroids[i].mass=(1.0f/3000.0f)*(1.33333333f*PI*asteroids[i].radius);
		asteroids[i].invMass=1.0f/asteroids[i].mass;

		asteroids[i].inertia=0.4f*asteroids[i].mass*(asteroids[i].radius*asteroids[i].radius);
		asteroids[i].invInertia=1.0f/asteroids[i].inertia;
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
	currentSeed=GetCurrentProcessId();
#else
	// Use process ID for random seed
	currentSeed=getpid();
#endif
	DBGPRINTF(DEBUG_INFO, "\033[25;0fAllocating zone memory...\n");
	zone=Zone_Init(8*1000*1000);

	// Set seed
	srand(currentSeed);

	GenerateWorld();

	// Clear client list
	memset(&clients, 0, sizeof(Client_t)*MAX_CLIENTS);

	// Start up network
	Network_Init();

	// Create a new sockets
	serverSocket=Network_CreateSocket();

	if(serverSocket==-1)
		return 1;

	// Bind that socket to 0.0.0.0 (any adapter) on port 4545
	if(!Network_SocketBind(serverSocket, 0, 4545))
		return 1;

	DBGPRINTF(DEBUG_WARNING, "\033[25;0fCurrent seed: %d, waiting for connections...", currentSeed);

	// Loop around pulling data that was sent, until you press escape to close.
	bool done=false;
	while(!done)
	{
		if(_kbhit())
		{
#ifdef WIN32
			if(_getch()==0x1B)
				done=true;
#else
			if(getchar()==0x1B)
				done=true;
#endif
		}

		NetworkPacket_t packet;

		int32_t length=0;
		uint32_t address=0;
		uint16_t port=0;

		if((length=Network_SocketReceive(serverSocket, (uint8_t *)&packet, sizeof(NetworkPacket_t), &address, &port))>0)
		{
			// Handle incoming connections
			if(packet.packetMagic==CONNECT_PACKETMAGIC)
			{
				DBGPRINTF(DEBUG_WARNING, "\033[25;0H\033[KConnect from: 0x%X port %d", address, port);

				NetworkPacket_t response;
				response.packetMagic=CONNECT_PACKETMAGIC;
				response.connect.seed=currentSeed;
				response.connect.port=port;
				response.clientID=addClient(address, port);

				Network_SocketSend(clients[response.clientID].socket, (uint8_t *)&response, sizeof(NetworkPacket_t), address, port);
			}
			// Handle disconnections
			else if(packet.packetMagic==DISCONNECT_PACKETMAGIC)
			{
				DBGPRINTF(DEBUG_WARNING, "\033[%d;0H\033[KDisconnect from: #%d 0x%X:%d",
						  packet.clientID+1,
						  packet.clientID,
						  address,
						  port
				);
				delClient(packet.clientID);
			}
			// Handle status reports
			else if(packet.packetMagic==STATUS_PACKETMAGIC)
			{
				Client_t *client=&clients[packet.clientID];

				if(client->isConnected)
				{
					// Copy camera from packet to client's camera.
					client->camera.body.position=packet.camera.position;
					client->camera.body.velocity=packet.camera.velocity;
					client->camera.body.orientation=packet.camera.orientation;

					// Update time to live for client "last time heard" (current time +30 seconds).
					client->TTL=GetClock()+30.0;
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
		uint8_t statusBuffer[1024];
		uint8_t fieldBuffer[32767];
		uint8_t *pBuffer;

		const double sixty=1.0/60.0;
		const double five=1.0/5.0;

		// Get the current time
		double currentTime=GetClock();

		if(currentTime>statusSendTime)
		{
			statusSendTime=currentTime+sixty;

			// Clear buffer
			memset(statusBuffer, 0, sizeof(statusBuffer));

			pBuffer=statusBuffer;

			// Serialize data
			Serialize_uint32(&pBuffer, STATUS_PACKETMAGIC); // Magic being sent back to clients is also "status"
			Serialize_uint32(&pBuffer, connectedClients); // Send current connected client count

			for(uint32_t i=0;i<MAX_CLIENTS;i++)
			{
				Client_t *client=&clients[i];

				if(client->isConnected)
				{
					Serialize_uint32(&pBuffer, client->clientID); // Client's ID
					Serialize_vec3(&pBuffer, client->camera.body.position); // Client camera position
					Serialize_vec3(&pBuffer, client->camera.body.velocity); // Client camera velocity
					Serialize_vec4(&pBuffer, client->camera.body.orientation); // Client camera forward (direction)

					// Report status to console
					DBGPRINTF(DEBUG_WARNING, "\033[%d;0H\033[KStatus from 0x%X:%d (ID %d) pos: %0.1f, %0.1f, %0.1f vel: %0.1f, %0.1f, %0.1f orientation: %0.1f %0.1f %0.1f %0.1f",
							  client->clientID+1,
							  client->address, client->port, client->clientID,
							  client->camera.body.position.x, client->camera.body.position.y, client->camera.body.position.z,
							  client->camera.body.velocity.x, client->camera.body.velocity.y, client->camera.body.velocity.z,
							  client->camera.body.orientation.x, client->camera.body.orientation.y, client->camera.body.orientation.z, client->camera.body.orientation.w
					);

					// If current time has past last hard time, then client has timed out... So remove it.
					if(GetClock()>client->TTL)
					{
						DBGPRINTF(DEBUG_WARNING, "\033[%d;0H\033[KDisconnected - Timed out.", client->clientID+1);
						delClient(client->clientID);
					}
				}
			}

			// Blast collected connected client data back to all connected clients
			for(uint32_t i=0;i<MAX_CLIENTS;i++)
			{
				if(clients[i].isConnected)
					Network_SocketSend(clients[i].socket, statusBuffer, sizeof(statusBuffer), clients[i].address, clients[i].port);
			}
		}

		// Update the whole asteroid field at 60FPS? Probably a bad idea, works on loopback network at least.
		if(currentTime>fieldSendTime)
		{
			fieldSendTime=currentTime+sixty;

			uint32_t asteroidCount=NUM_ASTEROIDS;
			uint32_t magic=FIELD_PACKETMAGIC;

			memset(fieldBuffer, 0, sizeof(fieldBuffer));

			pBuffer=fieldBuffer;
			Serialize_uint32(&pBuffer, magic);
			Serialize_uint32(&pBuffer, asteroidCount);

			for(uint32_t i=0;i<asteroidCount;i++)
			{
				Serialize_vec3(&pBuffer, asteroids[i].position);
				Serialize_vec3(&pBuffer, asteroids[i].velocity);
			}

			for(uint32_t i=0;i<MAX_CLIENTS;i++)
			{
				if(clients[i].isConnected)
					Network_SocketSend(clients[i].socket, fieldBuffer, sizeof(fieldBuffer), clients[i].address, clients[i].port);
			}
		}

		// Run physics stuff
		{
			const float dt=(float)sixty;//fTimeStep;

			// If current time has elapsed last set time, then run code
			if(currentTime>physicsTime)
			{
				// reset time to current time + time until next run
				physicsTime=currentTime+sixty;

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
					PhysicsIntegrate(&asteroids[i], dt);

					// Check asteroids against other asteroids
					for(int j=i+1;j<NUM_ASTEROIDS;j++)
						PhysicsSphereToSphereCollisionResponse(&asteroids[i], &asteroids[j]);

					for(uint32_t j=0;j<connectedClients;j++)
						PhysicsSphereToSphereCollisionResponse(&clients[j].camera.body, &asteroids[i]);

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
	Network_SocketClose(serverSocket);
	Network_Destroy();

	return 0;
}