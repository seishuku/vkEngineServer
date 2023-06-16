TARGET=vkEngineServer

# math
#OBJS+=math/math.o
#OBJS+=math/matrix.o
#OBJS+=math/quat.o
#OBJS+=math/vec2.o
#OBJS+=math/vec3.o
#OBJS+=math/vec4.o

# physics
#OBJS+=particle/particle.o
#OBJS+=physics/physics.o

# camera
#OBJS+=camera/camera.o

# network
OBJS+=network/network.o

# core stuff
OBJS+=utils/genid.o
OBJS+=utils/list.o
OBJS+=vkEngineServer.o

CC=gcc
CFLAGS=-Wall -Wno-missing-braces -Wextra -O3 -std=gnu17 -I/usr/X11/include
LDFLAGS=-Wold-style-definition -L/usr/X11/lib -lm -lpthread

all: $(TARGET) $(SHADERS)

debug: CFLAGS=-Wall -Wno-missing-braces -Wextra -msse3 -DDEBUG -D_DEBUG -g -ggdb -O1 -std=gnu17 -I/usr/X11/include
debug: $(TARGET) $(SHADERS)

$(TARGET): $(OBJS)
	$(CC) -o $@ $(OBJS) $(LDFLAGS)

%.o: %.c
	$(CC) -c $(CFLAGS) -o $@ $<

%.frag.spv: %.frag
	glslc --target-env=vulkan1.2 -O $< -o $@

%.vert.spv: %.vert
	glslc --target-env=vulkan1.2 -O $< -o $@

%.geom.spv: %.geom
	glslc --target-env=vulkan1.2 -O $< -o $@

%.comp.spv: %.comp
	glslc --target-env=vulkan1.2 -O $< -o $@

clean:
	$(RM) $(TARGET) $(OBJS) $(SHADERS)
