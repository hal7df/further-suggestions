#tiltdemo
#g++ -Wall -o ./tiltdemo.c.o -c ./tiltdemo.cpp
#g++ -Wall ./tiltdemo.c.o  -o ./tiltdemo -lfreenect_sync -lpthread -lfreenect -lusb-1.0

#glpclview
#g++ -Wall -o ./glpclview.c.o -c ./glpclview.cpp
#g++ -Wall ./glpclview.c.o -o ./glpclview -lfreenect_sync -lfreenect -lGLU -lGL -lSM -lICE -lX11 -lXext -lglut -lXmu -lXi -lpthread -lusb-1.0

#tracker
#g++ -Wall -o ./tracker.c.o -c ./tracker.cpp
#g++ -Wall ./tracker.c.o -o ./tracker -lfreenect_sync -lfreenect -lGLU -lGL -lSM -lICE -lX11 -lXext -lglut -lXmu -lXi -lpthread -lusb-1.0

CC=g++
CFLAGS=-c -Wall -O3 -I/usr/local/include/libfreenect
LDFLAGS=-L/usr/local/lib -lfreenect
SOURCES=tiltdemo.cpp
OBJECTS=$(SOURCES:%.cpp=%.o)
EXECUTABLE=tiltdemo
 
all: $(SOURCES) $(EXECUTABLE)
 
$(EXECUTABLE): $(OBJECTS)
	$(CC) $(LDFLAGS) $(OBJECTS) -o $@
 
.cpp.o:
	$(CC) $(CFLAGS) $< -o $@
 
clean :
	rm -f *.o $(EXECUTABLE)
