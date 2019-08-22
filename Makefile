MJ_PATH := $(HOME)/.mujoco/mujoco200

INC		:= -Iinclude -I$(MJ_PATH)/include -L$(MJ_PATH)/bin $(MJ_PATH)/bin/libglfw.so.3

CFLAGS	:= -Wall -Wextra -O3 -std=c++11 -pthread
LDFLAGS	:= -shared -Lsrc

CC 		:= g++
LIBS	:= -lmujoco200 -lGL -lglew 

default:
	$(CC) $(CFLAGS) testSim.cpp src/ik.cpp src/mujSimulation.cpp $(INC) $(LIBS) -o testSim
	$(CC) -c src/ik.cpp $(CFLAGS) $(INC) $(LIBS) -fPIC -o ik.o
	$(CC) -c src/mujSimulation.cpp $(CFLAGS) $(INC) $(LIBS) -fPIC -o mujsimulation.o

libmujsimulation:
	$(CC) -shared -o libmujsimulation.so mujsimulation.o ik.o $(CFLAGS) $(INC) $(LIBS)

all: default libmujsimulation

clean:
	rm -f testSim libcassie_ik.so ik.o libmujsimulation.so mujsimulation.o