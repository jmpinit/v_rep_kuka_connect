MODNAME=core
MODSO=$(MODNAME).so

LUA_CFLAGS=-I/usr/local/include/lua-5.1
#LDFLAGS=$(pkg-config lua5.1 --libs) -ltinyxml
LDFLAGS=-llua.5.1 -lm -ltinyxml
INCLUDES=$(LUA_CFLAGS)
CFLAGS=-std=c++11

CCOPT= -fomit-frame-pointer
CCWARN= -Wall
SOCFLAGS= -g -fPIC $(CCOPT) $(CCWARN) $(INCLUDES) $(CFLAGS)
SOLDFLAGS= -fPIC $(LDFLAGS)

all: $(MODSO)

install: $(MODSO)
	cp $(MODSO) ..

clean:
	rm *.so *.o

$(MODNAME).o: $(MODNAME).cpp udp_server.h
	$(CXX) $(SOCFLAGS) -c -o $@ $<

$(MODSO): $(MODNAME).o
	$(CXX) $(SOLDFLAGS) -shared -o $@ $<

.PHONY: all install clean

