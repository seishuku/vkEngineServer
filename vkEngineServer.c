#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <signal.h>
#include <time.h>
#include "system/system.h"
#include "math/math.h"
#include "physics/physics.h"
#include "utils/bvh.h"
#include "camera/camera.h"
#include "network/network.h"
#include "network/net_protocol.h"
#include "network/server_network.h"
#include "entitylist.h"
#include "asteroids.h"

#define SERVER_PORT				4545
#define SERVER_TICK_RATE		60.0
#define SERVER_TICK_DT			(1.0/SERVER_TICK_RATE)

MemZone_t *zone=NULL;
EntityList_t    entityList;
BVH_t           bvh;

static Camera_t playerBodies[NET_MAX_CLIENTS];

#define MAX_EMITTERS 1000
 
typedef struct
{
    RigidBody_t body;
    uint32_t entityID;
    float life;
} PhyParticleEmitter_t;
 
static PhyParticleEmitter_t emitters[MAX_EMITTERS];
 
#define MAX_MANIFOLDS 10000

static struct
{
    Entity_t *objA, *objB;
    CollisionManifold_t manifold;
} manifoldList[MAX_MANIFOLDS];

static uint32_t numManifolds=0;
static bool     running=true;

double GetClock(void)
{
    struct timespec ts;

    if(!clock_gettime(CLOCK_MONOTONIC, &ts))
        return ts.tv_sec+(double)ts.tv_nsec/1000000000.0;

    return 0.0;
}

uint32_t AddServerEmitter(vec3 position, vec3 velocity, float life)
{
    for(uint32_t i=0;i<MAX_EMITTERS;i++)
    {
        if(emitters[i].life>0.0f)
            continue;
 
        emitters[i].body.position=position;
        emitters[i].body.velocity=velocity;
        emitters[i].life=life;
        emitters[i].entityID=EntityList_Add(&entityList, &emitters[i].body, true, 0, 0, 0, ENTITYOBJECTTYPE_PROJECTILE, NULL);
 
        return emitters[i].entityID;
    }
 
    DBGPRINTF(DEBUG_WARNING, "AddServerEmitter: emitter pool full\n");
    return NET_INVALID_ID;
}
 
static void RemoveServerEmitter(uint32_t index)
{
    if(index>=MAX_EMITTERS)
        return;
 
    if(emitters[index].entityID!=NET_INVALID_ID)
    {
        // Notify all clients to destroy this projectile
        NetEvent_t ev={
            .type=NETEVENT_DESTROY,
            .destroy={ .id=emitters[index].entityID },
        };
        ServerNetwork_BroadcastEvent(&ev);
 
        EntityList_Remove(&entityList, emitters[index].entityID);
        emitters[index].entityID=NET_INVALID_ID;
    }
 
    emitters[index].life=0.0f;
}

uint32_t AddPlayer(uint32_t clientID)
{
    if(clientID>=NET_MAX_CLIENTS)
        return NET_INVALID_ID;

    CameraInit(&playerBodies[clientID], Vec3b(0.0f), Vec3(0.0f, 1.0f, 0.0f), Vec3(0.0f, 0.0f, -1.0f));

	// Reusing modelID to track clientID
    uint32_t id=EntityList_Add(&entityList, &playerBodies[clientID].body, true, clientID, 0, 0, ENTITYOBJECTTYPE_PLAYER, NULL);

    DBGPRINTF(DEBUG_INFO, "Player %d added, entity ID %d\n", clientID, id);

    return id;
}

void RemovePlayer(uint32_t clientID, uint32_t entityID)
{
    if(clientID>=NET_MAX_CLIENTS)
        return;

    if(entityID!=NET_INVALID_ID)
        EntityList_Remove(&entityList, entityID);

    memset(&playerBodies[clientID], 0, sizeof(Camera_t));

    DBGPRINTF(DEBUG_INFO, "Player %d removed\n", clientID);
}

static void HandleSignal(int sig)
{
    (void)sig;
    DBGPRINTF(DEBUG_INFO, "\nServer shutting down...\n");
    running=false;
}

static void TestCollision(Entity_t *objA, Entity_t *objB)
{
    CollisionManifold_t manifold=PhysicsCollision(objA->body, objB->body);

    if(manifold.contactCount>0&&numManifolds<MAX_MANIFOLDS)
    {
        manifoldList[numManifolds].objA=objA;
        manifoldList[numManifolds].objB=objB;
        manifoldList[numManifolds].manifold=manifold;
        numManifolds++;
    }
}

static void PhysicsTick(float dt)
{
	for(uint32_t i=0;i<MAX_EMITTERS;i++)
	{
		if(emitters[i].entityID==NET_INVALID_ID)
			continue;  // slot not in use

		if(emitters[i].life>0.0f)
			emitters[i].life-=dt;
		else
			RemoveServerEmitter(i);
	}

	for(uint32_t i=0;i<entityList.entityCount;i++)
	{
        Entity_t *entity=&entityList.entities[i];

		// TODO: Should this be done anyway, even though the client updates the physics for it's player?
        if(entity->objectType!=ENTITYOBJECTTYPE_PLAYER)
            PhysicsIntegrate(entity->body, dt);
    }

    // Broadphase
    memset(manifoldList, 0, sizeof(manifoldList));
    numManifolds=0;

    EntityList_RecalculateBounds(&entityList);
    BVH_Build(&bvh, &entityList);
    BVH_Test(&bvh, &entityList, TestCollision);

    // Narrowphase and response
    for(uint32_t i=0;i<numManifolds;i++)
    {
        CollisionManifold_t *manifold=&manifoldList[i].manifold;
        Entity_t *objA=manifoldList[i].objA;
        Entity_t *objB=manifoldList[i].objB;

        for(uint32_t j=0;j<manifold->contactCount;j++)
        {
            float impactSpeed=PhysicsResolveCollision(manifold->a, manifold->b, manifold->contacts[j]);

            if(impactSpeed<2.0f)
				continue;

            // Asteroid-asteroid: physics response only, no special handling
            if(objA->objectType==ENTITYOBJECTTYPE_FIELD&&objB->objectType==ENTITYOBJECTTYPE_FIELD)
                continue;

            // Any collision involving a player - send impulse event to owning client
            if(objA->objectType==ENTITYOBJECTTYPE_PLAYER||objB->objectType==ENTITYOBJECTTYPE_PLAYER)
            {
                Entity_t *player=(objA->objectType==ENTITYOBJECTTYPE_PLAYER)?objA:objB;
                ServerNetwork_SendPlayerImpulse(player->ID, player->body->position, player->body->velocity);
            }

            // Projectile hits asteroid - split
			// TODO: This feels dumb, should just do the split on server and just spawn the new asteroids
            // if((objA->objectType==ENTITYOBJECTTYPE_PROJECTILE&&objB->objectType==ENTITYOBJECTTYPE_FIELD)||
            //    (objB->objectType==ENTITYOBJECTTYPE_PROJECTILE&&objA->objectType==ENTITYOBJECTTYPE_FIELD))
            // {
            //     Entity_t *asteroid=(objA->objectType==ENTITYOBJECTTYPE_FIELD)?objA:objB;

            //     uint32_t rngSnapshot=GetRandomSeed();
            //     uint32_t asteroidIndex=UINT32_MAX;

            //     for(uint32_t k=0;k<numAsteroids;k++)
            //     {
            //         if(asteroid->body==&asteroids[k])
            //         {
            //             asteroidIndex=k;
            //             break;
            //         }
            //     }

            //     if(asteroidIndex==UINT32_MAX)
            //         continue;

            //     uint32_t parentID=asteroid->ID;

            //     SplitAsteroid(asteroidIndex, manifold->contacts[j], impactSpeed);

            //     NetEvent_t ev={ .type=NETEVENT_SPLIT, .split={ .parentID=parentID, .rngSnapshot=rngSnapshot, .contactPoint=manifold->contacts[j].position, .contactNormal=manifold->contacts[j].normal, .impactSpeed=impactSpeed } };
            //     ServerNetwork_BroadcastEvent(&ev);
            // }
        }
    }

    EntityList_Rebuild(&entityList);
}

// ============================================================
// World setup
// ============================================================
static void GenerateWorld(uint32_t seed)
{
    RandomSeed(seed);

    ResetAsteroids();

    for(uint32_t i=0;i<numAsteroids;i++)
    {
		// Reusing modelID to track asteroid variants
        EntityList_Add(&entityList, &asteroids[i], true, asteroidModels[i].variant, 0, 0, ENTITYOBJECTTYPE_FIELD, NULL);
    }

    DBGPRINTF(DEBUG_INFO, "World generated: %d asteroids, seed %u\n", numAsteroids, seed);
}

int main(int argc, char **argv)
{
    DBGPRINTF(DEBUG_INFO, "Allocating zone memory...\n");
    zone=Zone_Init(8*1000*1000);

    if(!zone)
    {
        DBGPRINTF(DEBUG_ERROR, "Failed to allocate zone memory.\n");
        return 1;
    }

    uint16_t port=SERVER_PORT;
    uint32_t seed=(uint32_t)GetClock();

    if(argc>=2)
        port=(uint16_t)atoi(argv[1]);

    if(argc>=3)
        seed=(uint32_t)atoi(argv[2]);

    signal(SIGINT, HandleSignal);
    signal(SIGTERM, HandleSignal);

    memset(playerBodies, 0, sizeof(playerBodies));

	for(uint32_t i=0;i<MAX_EMITTERS;i++)
	{
		emitters[i].life=-1.0f;		// No life

		const float radius=0.5f;
		const float mass=(1.0f/3000.0f)*(1.33333333f*PI*10.0f)*10.0f;
		const float inertia=0.4f*mass*(10.0f*10.0f);

		emitters[i].body=(RigidBody_t)
		{
			.position=Vec3b(0.0f),

			.velocity=Vec3b(0.0f),
			.force=Vec3b(0.0f),
			.mass=mass,
			.invMass=1.0f/mass,

			.orientation=Vec4(0.0f, 0.0f, 0.0f, 1.0f),
			.angularVelocity=Vec3b(0.0f),
			.inertia=inertia,
			.invInertia=1.0f/inertia,

			.restitution=1.0f,
			.friction=0.0f,

			.type=RIGIDBODY_SPHERE,
			.radius=radius,
		};
	}

	DBGPRINTF(DEBUG_INFO, "vkEngine Dedicated Server\nPort: %d  Seed: %u\n", port, seed);

    if(!EntityList_Init(&entityList))
    {
        DBGPRINTF(DEBUG_ERROR, "Failed to init entity list.\n");
        return 1;
    }

    GenerateWorld(seed);

    if(!ServerNetwork_Init(port, &entityList, seed))
    {
        DBGPRINTF(DEBUG_ERROR, "Failed to init server network.\n");
        EntityList_Destroy(&entityList);
        return 1;
    }

    DBGPRINTF(DEBUG_INFO, "Server running at %g Hz\n", SERVER_TICK_RATE);

    double lastTick=GetClock();
    double accumulator=0.0;

    while(running)
    {
        double now=GetClock();
        double elapsed=now-lastTick;
        lastTick=now;

        if(elapsed>0.25)
            elapsed=0.25;

        accumulator+=elapsed;

        while(accumulator>=SERVER_TICK_DT)
        {
            PhysicsTick((float)SERVER_TICK_DT);
            accumulator-=SERVER_TICK_DT;

			DBGPRINTF(DEBUG_INFO, "Network average sent: %0.3fk received: %0.3fk            \r", Network_AvgBytesSent/1024.0f, Network_AvgBytesReceived/1024.0f);
			Network_ResetCounters();
        }

        ServerNetwork_Update(now);

        double tickEnd=GetClock();
        double remaining=SERVER_TICK_DT-(tickEnd-now);

        if(remaining>0.0)
            SleepMS((uint32_t)(remaining*1000.0));
    }

    ServerNetwork_Destroy();
    EntityList_Destroy(&entityList);

    DBGPRINTF(DEBUG_INFO, "Server shutdown complete.\n");

    return 0;
}
