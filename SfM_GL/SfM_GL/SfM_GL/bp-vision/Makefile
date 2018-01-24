INCDIR = -I.
DBG    = -g
OPT    = -O3
CPP    = g++
CFLAGS = $(DBG) $(OPT) $(INCDIR)
LINK   = -lm 

.cpp.o:
	$(CPP) $(CFLAGS) -c $< -o $@

all: restore stereo noise

restore: restore.cpp 
	$(CPP) $(CFLAGS) -o restore restore.cpp $(LINK)

stereo: stereo.cpp 
	$(CPP) $(CFLAGS) -o stereo stereo.cpp $(LINK)

noise: noise.cpp 
	$(CPP) $(CFLAGS) -o noise noise.cpp $(LINK)

clean:
	/bin/rm -f restore stereo noise *.o

clean-all: clean
	/bin/rm -f *~ 



