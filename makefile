SOURCEDIR := src
SOURCE := $(shell find $(SOURCEDIR) -name '*.cc')

all:
	g++ $(SOURCE) -o particle_filter.o `pkg-config --cflags --libs opencv` -lboost_filesystem -lboost_system

clean:
	$(RM) particle_filter.o
