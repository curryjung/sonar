CC = g++
SRCS = mainHoi.cpp Loadcell.cpp Sonar.cpp ManualMode.cpp ATSCom.cpp CircularQueue.cpp
PROG = EvarCart
LIBS = -lpthread

$(PROG):$(SRCS)
	$(CC) -o $(PROG) $(SRCS) $(LIBS) 

clean:
	rm -f $(PROG)