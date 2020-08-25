# Set compiler to use
GG=g++
CC=c++
CFLAGS=-g -I. -I/opt/etherlab/include -fpermissive -std=c++11 -w -Wwrite-strings -I/usr/include/libxml2/
LDFLAGS= -lpthread  

LDFLAGS_BOOST=-L/usr/local/lib -lboost_system -lboost_thread
DEBUG=0


all:: VCFServer_cpp.exe 



VCFServer_cpp.exe: ecat.o Rubiserver.o EcatMgr_HA.o HybridAutomata.o  
	$(GG) -o $@ $^ $(CFLAGS) /opt/etherlab/lib/libethercat.a $(LDFLAGS) 


%.o: %.c $(DEPS)
	$(CC) -c -o $@ $< $(CFLAGS)

%.o: %.cc $(DEPS)
	$(CC) -c -o $@ $< $(CFLAGS)

%.o: %.cpp $(DEPS)
	$(CC) -c -o $@ $< $(CFLAGS)

clean: 
	rm -f *.exe *.o *~ *.stackdump *_middleman .*.swo .*.swp
